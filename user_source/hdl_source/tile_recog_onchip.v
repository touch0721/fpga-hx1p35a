`timescale 1ns / 1ps
// ============================================================================
//  tile_recog_onchip.v — 麻将牌识别【阶段 1 + 整排切分】
// ----------------------------------------------------------------------------
//  位置：ISP 输出域（与 frame_diff_onchip 同域、同像素流），**不碰 HDMI 时序域**
//
//  数据源：
//    · 像素流：I_y_now / I_col4（4 像素/拍，ISP 的 s1 级，与 valid/last/user 严格同拍）
//    · 合并框：frame_diff_onchip 的上一帧结果 ★【像素】坐标，不是块坐标 ★
//              （frame_diff 里 c_x1 = 块<<3、c_x2 = (块<<3)+7）
//
//  处理链：
//    ① 坐标重建：行末用 I_pix_last 归零拍计数（免疫"每行拍数"差异）
//    ② ★整排切分★：紧贴成一排的牌会被 frame_diff 的邻域投票（MARGIN=4 块 = 32 像素）
//       合并成【一个】大框，而牌间没有缝、投影谷值法也无效。解法是靠几何反推：
//       牌立着时 宽:高 ≈ 0.708 ≈ 12/17，框高就是一张牌的高，故一张牌的宽 ≈ 0.708*h，
//       张数 N ≈ w / (0.708*h)，再按 N 等宽切分（余数按 Bresenham 分给前几份）。
//       ★ 全程无乘法器、无除法器：定标全是移位加法；求 N 用重复减法；
//         求每份宽用一次 8 拍移位除法（w ≤ 240 块，8 位够）。
//    ③ ROI 门控（按切出来的子框，块粒度）
//    ④ 前景判定 + 4 桶色系分类（色系码由 ISP 用比较器算好，零乘法）
//    ⑤ 每槽累加 + 帧末锁存（像素数 / 颜色签名 / 空白率 / 宽高比）
//    ⑥ 按 I_sel 输出【选中槽】的特征与其矩形（供 OSD 显示 + 画黄框）
//       —— 只传一张牌的几十根线，而不是 16 槽的 800 多根
//
//  ★DSP★ 本模块一个算术 * 都没有（TD 会把 * 映射成 DSP，本器件上限 40、基线已用 33）。
// ============================================================================
module tile_recog_onchip #(
    parameter integer IMG_W     = 1920,
    parameter integer IMG_H     = 1080,
    parameter integer NBOX      = 14,       // 输入框数（★ 固定 14 个牌位：主框 13 张 + 右侧小框 1）
    parameter integer NSLOT     = 16,       // 切分后槽数（一排最多 9 张，留余量）
    parameter integer MAX_TILE   = 10,      // 一个框最多切成几张（防误切）
    parameter [7:0]   Y_DARK    = 8'd150,   // Y 低于它算"暗图案"
    parameter integer BLANK_PCT = 2,        // 空白率阈值（百分比）
    parameter integer MIN_BLK   = 3,        // 子框每边至少几个块（=24 像素）
    parameter integer GXOFF     = 0,        // 块坐标微调（与 frame_diff 相位对齐）
    parameter integer GYOFF     = 0,
    // ---- 阶段 2：细粒度数子图（筒数 / 条数）----
    //   4x4 像素 = 1 格；一张立牌 ≈ 17x23 格，一个筒子 ≈ 5 格
    parameter integer BMP_W     = 20,       // 局部位图宽（格）：20 格 = 80 像素
    parameter integer BMP_H     = 28,       // 局部位图高（格）：28 格 = 112 像素（= 牌位高）
    parameter [4:0]   CELL_THR  = 5'd4,     // 一格 16 像素里 >= 4 个前景像素 -> 算占据
    parameter integer DOT_MIN   = 2,        // 连通块 >= 这么多格才计入（滤孤立噪点）
    parameter [6:0]   BAR_MIN_V = 7'd4,     // 列投影峰的最小高度（像素）
    parameter integer BAR_MIN_W = 2,        // 峰宽下限（像素）
    parameter integer BAR_MIN_G = 2         // 两峰之间的间隙下限（像素）
) (
    input  wire        I_pix_clk,
    input  wire        I_rst_n,
    input  wire        I_en,                // 与 frame_diff 同一个使能（SW4）

    input  wire        I_pix_valid,
    input  wire        I_pix_last,          // ★ 行末标记（ISP 用它递增行计数器）
    input  wire        I_pix_user,          // ★ 帧首标记
    input  wire [31:0] I_y_now,
    input  wire [7:0]  I_col4,              // 4 像素色系码（2 位/像素）：0红1绿2蓝3无彩

    input  wire [NBOX-1:0]    I_bv,         // frame_diff 的合并框（★ 像素坐标）
    input  wire [12*NBOX-1:0] I_bx1,
    input  wire [12*NBOX-1:0] I_bx2,
    input  wire [12*NBOX-1:0] I_by1,
    input  wire [12*NBOX-1:0] I_by2,

    input  wire [3:0]  I_sel,               // 显示哪个槽（来自 mixer 的轮播，准静态）

    output reg         O_sel_v,
    output reg  [21:0] O_sel_pixcnt,
    output reg  [31:0] O_sel_colsig,        // [31:24]红 [23:16]绿 [15:8]蓝 [7:0]无彩
    output reg         O_sel_blank,
    output reg         O_sel_ar_ok,
    output reg  [11:0] O_sel_x1, O_sel_x2,  // ★ 像素坐标（与 frame_diff 同约定）
    output reg  [11:0] O_sel_y1, O_sel_y2,

    output reg  [3:0]  O_hit_slot,
    output reg  [4:0]  O_nslot,             // 诊断：本帧切出几个子框
    output reg         O_done,

    // ---- 阶段 2：细粒度计数结果（都属于【选中槽】那一张牌）----
    output reg  [4:0]  O_sel_dots,          // 连通块数（筒候选），饱和 15
    output reg  [4:0]  O_sel_bars,          // 列投影峰数（条候选），饱和 15
    output reg  [8:0]  O_sel_cells,         // 位图里被占据的格数（诊断）
    output reg  [8:0]  O_sel_maxc           // 最大连通块的格数（诊断）
);
    localparam integer LN_MAX   = IMG_H - 1;
    localparam integer CNT_W    = 22;
    localparam integer NUM_W    = 28;
    localparam [3:0]   SLOT_LAST= NSLOT - 1;
    localparam integer DENS_SH  = 3;        // 4 像素/拍 * BLANK_PCT(2) = 8 = 1<<3
    localparam integer BIDX_W   = 8;        // 块坐标位宽

    // ======================================================================
    //  ① 坐标重建
    //     ★ user 那一拍必须【组合性】当成本帧第 0 拍（寄存器要等时钟沿才清零，
    //       而这一拍就是要算的像素；否则每帧漏一拍）
    //     ★ 行末用 I_pix_last 归零拍计数：不要求"每行拍数恰好 = IMG_W/4"
    // ======================================================================
    reg  [9:0]  s_lb;
    reg  [10:0] s_line;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin s_lb <= 10'd0; s_line <= 11'd0; end
        else if (I_pix_valid) begin
            if (I_pix_user)      begin s_lb <= 10'd1; s_line <= 11'd0;          end
            else if (I_pix_last) begin s_lb <= 10'd0; s_line <= s_line + 11'd1; end
            else                      s_lb <= s_lb + 10'd1;
        end
    end
    wire s_vend = I_pix_valid && I_pix_last && (s_line == LN_MAX[10:0]);

    wire [9:0]  s_lb_c   = I_pix_user ? 10'd0 : s_lb;
    wire [10:0] s_line_c = I_pix_user ? 11'd0 : s_line;
    wire [7:0]  s_bc     = s_lb_c[9:1];             // 块列 0..239
    wire [7:0]  s_brow   = s_line_c[10:3];          // 块行 0..134
    wire [8:0]  s_bc_g   = {1'b0, s_bc}   + GXOFF[8:0];
    wire [8:0]  s_brow_g = {1'b0, s_brow} + GYOFF[8:0];

    // ======================================================================
    //  输入合并框：像素 -> 块（取 [10:3]；不能写成 I_bx1[..+:12][10:3]）
    // ======================================================================
    wire [BIDX_W*NBOX-1:0] b_x1, b_x2, b_y1, b_y2;
    genvar gi;
    generate
        for (gi = 0; gi < NBOX; gi = gi + 1) begin : BXC
            wire [11:0] t1 = I_bx1[12*gi +: 12];
            wire [11:0] t2 = I_bx2[12*gi +: 12];
            wire [11:0] t3 = I_by1[12*gi +: 12];
            wire [11:0] t4 = I_by2[12*gi +: 12];
            assign b_x1[BIDX_W*gi +: BIDX_W] = t1[10:3];
            assign b_x2[BIDX_W*gi +: BIDX_W] = t2[10:3];
            assign b_y1[BIDX_W*gi +: BIDX_W] = t3[10:3];
            assign b_y2[BIDX_W*gi +: BIDX_W] = t4[10:3];
        end
    endgenerate

    // ======================================================================
    //  ② 整排切分 FSM（帧末跑；产出 NSLOT 个子框，块坐标）
    // ======================================================================
    reg  [NSLOT-1:0]        s_sv;
    reg  [BIDX_W*NSLOT-1:0] s_sx1, s_sx2, s_sy1, s_sy2;

    localparam [2:0] SP_IDLE=3'd0, SP_CALC=3'd1, SP_NDIV=3'd2,
                     SP_DIVI=3'd3, SP_LAYO=3'd4, SP_DONE=3'd5;

    reg [2:0]  s_sp;
    reg [3:0]  s_j;                 // 当前输入框
    reg [4:0]  s_oc;                // 已输出子框数
    reg [7:0]  s_cx1, s_cx2, s_cy1, s_cy2;
    reg [7:0]  s_cur;               // 当前子框起点（块列）
    reg [7:0]  s_wblk;
    reg [15:0] s_rem;               // w*17 + h*6
    reg [13:0] s_h12;               // h*12
    reg [4:0]  s_n;                 // N：切几张
    reg [3:0]  s_di;                // 除法迭代计数 0..8
    reg [7:0]  s_dq, s_drem;        // 商/余（wblk / N）
    reg [3:0]  s_lo;                // 分配子框计数
    reg [4:0]  s_err;               // 余数分配累加器
    reg [7:0]  s_len;               // 本份长度

    wire [7:0] c_x1 = b_x1[BIDX_W*s_j +: BIDX_W];
    wire [7:0] c_x2 = b_x2[BIDX_W*s_j +: BIDX_W];
    wire [7:0] c_y1 = b_y1[BIDX_W*s_j +: BIDX_W];
    wire [7:0] c_y2 = b_y2[BIDX_W*s_j +: BIDX_W];

    // ★ 本份长度/终点/进位数都用【组合】算，不能在同一个 always 里用刚赋值的
    //   非阻塞值（那样会晚一拍，边界全错）
    wire [4:0]  lay_err = ((s_err + {1'b0, s_drem}) >= {1'b0, s_n})
                        ? (s_err + {1'b0, s_drem} - {1'b0, s_n})
                        : (s_err + {1'b0, s_drem});
    wire [7:0]  lay_len = ((s_err + {1'b0, s_drem}) >= {1'b0, s_n}) ? (s_dq + 8'd1) : s_dq;
    wire [7:0]  lay_x2  = s_cur + lay_len - 8'd1;
    wire [7:0]  lay_nx  = s_cur + lay_len;

    integer wi;                      // ★ 切分 FSM 自己的循环变量
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_sp <= SP_IDLE; s_oc <= 5'd0; s_j <= 4'd0; s_n <= 5'd0;
            s_sv <= {NSLOT{1'b0}}; O_nslot <= 5'd0;
        end else begin
            case (s_sp)
                SP_IDLE: if (s_fst == F_DONE) begin
                    // ★ 切分必须排在"帧末锁存特征"【之后】：
                    //   本帧的像素是用【上一帧】切出的门控累加的，所以帧末要先把
                    //   特征和那个门控一起锁存（显示才对得上），然后再用本帧的合并框
                    //   重切门控、留给下一帧用。顺序颠倒会让"显示矩形"和"特征"差一帧。
                    s_sv <= {NSLOT{1'b0}};          // 每帧重新切
                    s_oc <= 5'd0;  s_j <= 4'd0;
                    s_sp <= SP_CALC;
                end
                // ---- 取下一个有效输入框，算 w*17 + h*6 与 h*12 ----
                SP_CALC: begin
                    if (s_j >= NBOX[3:0]) s_sp <= SP_DONE;
                    else if (!I_bv[s_j])  s_j  <= s_j + 4'd1;
                    else begin
                        s_cx1 <= c_x1; s_cx2 <= c_x2; s_cy1 <= c_y1; s_cy2 <= c_y2;
                        s_wblk<= (c_x2 >= c_x1) ? (c_x2 - c_x1 + 8'd1) : 8'd0;
                        // w*17 = 16w + w ;  h*12 = 8h + 4h ;  h*6 = 4h + 2h
                        s_rem <= ({4'd0, (c_x2 - c_x1 + 8'd1)} << 4) + {4'd0, (c_x2 - c_x1 + 8'd1)}
                               + ({4'd0, (c_y2 - c_y1 + 8'd1)} << 2) + ({4'd0, (c_y2 - c_y1 + 8'd1)} << 1);
                        s_h12 <= ({4'd0, (c_y2 - c_y1 + 8'd1)} << 3) + ({4'd0, (c_y2 - c_y1 + 8'd1)} << 2);
                        s_n   <= 5'd0;
                        s_sp  <= SP_NDIV;
                    end
                end
                // ---- 求张数 N：反复减 h*12（等价于 (w*17 + h*6) / (h*12) 的四舍五入）----
                SP_NDIV: begin
                    if ((s_n < MAX_TILE[4:0]) && (s_rem >= {2'd0, s_h12})) begin
                        s_rem <= s_rem - {2'd0, s_h12};
                        s_n   <= s_n + 5'd1;
                    end else begin
                        if (s_n == 5'd0) s_n <= 5'd1;   // 至少一张
                        s_di  <= 4'd8;                  // 除法从最高位开始
                        s_dq  <= 8'd0;
                        s_drem<= 8'd0;
                        s_sp  <= SP_DIVI;
                    end
                end
                // ---- 每份宽 = wblk / N（8 拍移位减法除法，N 已确定）----
                SP_DIVI: begin
                    if (s_di == 4'd0) begin
                        s_lo  <= 4'd0;
                        s_err <= 5'd0;
                        s_cur <= s_cx1;
                        s_sp  <= SP_LAYO;
                    end else begin
                        // rem = (rem<<1) | wblk[di-1]
                        if ({(s_drem[6:0]), s_wblk[s_di - 4'd1]} >= {3'd0, s_n}) begin
                            s_drem <= {(s_drem[6:0]), s_wblk[s_di - 4'd1]} - {3'd0, s_n};
                            s_dq[s_di - 4'd1] <= 1'b1;
                        end else begin
                            s_drem <= {(s_drem[6:0]), s_wblk[s_di - 4'd1]};
                            s_dq[s_di - 4'd1] <= 1'b0;
                        end
                        s_di <= s_di - 4'd1;
                    end
                end
                // ---- 按 N 份分配：长度 = 商(+1)，余数用累加器摊给前几份 ----
                SP_LAYO: begin
                    if (s_lo >= s_n[3:0]) begin
                        s_sp <= SP_CALC;                // 本框切完，取下一个框
                        s_j  <= s_j + 4'd1;
                    end else if (s_oc > SLOT_LAST) begin
                        s_sp <= SP_DONE;                // 槽用完了
                    end else begin
                        // 写入第 s_oc 个子框（显式 for + 索引比较；用组合算出的长度）
                        for (wi = 0; wi < NSLOT; wi = wi + 1) begin
                            if (wi == s_oc[3:0]) begin
                                s_sv [wi]        <= 1'b1;
                                s_sx1[BIDX_W*wi +: BIDX_W] <= s_cur;
                                s_sx2[BIDX_W*wi +: BIDX_W] <= lay_x2;
                                s_sy1[BIDX_W*wi +: BIDX_W] <= s_cy1;
                                s_sy2[BIDX_W*wi +: BIDX_W] <= s_cy2;
                            end
                        end
                        s_cur <= lay_nx;
                        s_err <= lay_err;
                        s_oc  <= s_oc + 5'd1;
                        s_lo  <= s_lo + 4'd1;
                    end
                end
                SP_DONE: begin
                    O_nslot <= s_oc;
                    s_sp    <= SP_IDLE;
                end
                default: s_sp <= SP_IDLE;
            endcase
        end
    end

    // ======================================================================
    //  ③ ROI 门控（★ C1 瘦身：只对【选中槽】做，不再 16 槽并行比一遍）
    //     OSD 本来就只显示选中槽，阶段 2 也是这么做的 —— 省掉 16 组比较与
    //     16 槽累加器（16 x (5x22 + 20) = 2080 FF）+ 帧末 16 选 1 多路器。
    //     换槽后下一帧就出新数（轮播 1.5 秒，够）。
    // ======================================================================
    localparam [3:0] SLOTMAX = NSLOT - 1;                       // 合法槽号上限
    wire [3:0]  sel_i  = (I_sel <= SLOTMAX) ? I_sel : 4'd0;     // 越界保护
    //   ★ 不能写 NSLOT[3:0]：对 integer 参数做部分选择会得到 x（实测过，整个模块
    //     会一直按 0 号槽工作）
    wire [7:0]  sb_x1  = s_sx1[BIDX_W*sel_i +: BIDX_W];         // 选中槽子框（块坐标）
    wire [7:0]  sb_x2  = s_sx2[BIDX_W*sel_i +: BIDX_W];
    wire [7:0]  sb_y1  = s_sy1[BIDX_W*sel_i +: BIDX_W];
    wire [7:0]  sb_y2  = s_sy2[BIDX_W*sel_i +: BIDX_W];

    wire s_hit_sel = s_sv[sel_i]
                  && (s_bc_g   >= {1'b0, sb_x1})
                  && (s_bc_g   <= {1'b0, sb_x2})
                  && (s_brow_g >= {1'b0, sb_y1})
                  && (s_brow_g <= {1'b0, sb_y2});

    // 诊断输出：选中槽命中就给槽号，没命中给 15（原先是"最低命中槽"，
    // 单槽累加后这个量只对选中槽有意义）
    always @(*) O_hit_slot = s_hit_sel ? sel_i : 4'd15;

    // ======================================================================
    //  ④ 前景判定 + 4 桶色系分类
    // ======================================================================
    wire [3:0] s_fg, s_bin_r, s_bin_g, s_bin_b, s_bin_k;
    genvar li;
    generate
        for (li = 0; li < 4; li = li + 1) begin : LANE
            wire [7:0] y  = I_y_now[8*li +: 8];
            wire [1:0] cc = I_col4 [2*li +: 2];
            wire s_colored = (cc != 2'd3);
            assign s_fg[li]    = (y < Y_DARK) || s_colored;
            assign s_bin_k[li] = s_fg[li] && (cc == 2'd3);
            assign s_bin_r[li] = s_fg[li] && (cc == 2'd0);
            assign s_bin_g[li] = s_fg[li] && (cc == 2'd1);
            assign s_bin_b[li] = s_fg[li] && (cc == 2'd2);
        end
    endgenerate

    wire [2:0] s_n_fg = {2'd0,s_fg[0]}    + {2'd0,s_fg[1]}    + {2'd0,s_fg[2]}    + {2'd0,s_fg[3]};
    wire [2:0] s_n_r  = {2'd0,s_bin_r[0]} + {2'd0,s_bin_r[1]} + {2'd0,s_bin_r[2]} + {2'd0,s_bin_r[3]};
    wire [2:0] s_n_g  = {2'd0,s_bin_g[0]} + {2'd0,s_bin_g[1]} + {2'd0,s_bin_g[2]} + {2'd0,s_bin_g[3]};
    wire [2:0] s_n_b  = {2'd0,s_bin_b[0]} + {2'd0,s_bin_b[1]} + {2'd0,s_bin_b[2]} + {2'd0,s_bin_b[3]};
    wire [2:0] s_n_k  = {2'd0,s_bin_k[0]} + {2'd0,s_bin_k[1]} + {2'd0,s_bin_k[2]} + {2'd0,s_bin_k[3]};

    // ======================================================================
    //  ⑤ 累加（★ C1 瘦身：只有【选中槽】一份，帧末处理完后 s_clr 清零）
    // ======================================================================
    reg [CNT_W-1:0] s_cnt;                  // 前景像素数
    reg [CNT_W-1:0] s_cr_c, s_cg_c, s_cb_c, s_ck_c;   // 4 桶色系计数
    reg [19:0]      s_beat;                 // 命中拍数（算空白率用）
    reg s_clr;

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en || s_clr) begin
            s_cnt  <= {CNT_W{1'b0}};
            s_cr_c <= {CNT_W{1'b0}};
            s_cg_c <= {CNT_W{1'b0}};
            s_cb_c <= {CNT_W{1'b0}};
            s_ck_c <= {CNT_W{1'b0}};
            s_beat <= 20'd0;
        end else if (I_pix_valid && s_hit_sel) begin
            s_cnt  <= s_cnt  + {{(CNT_W-3){1'b0}}, s_n_fg};
            s_cr_c <= s_cr_c + {{(CNT_W-3){1'b0}}, s_n_r};
            s_cg_c <= s_cg_c + {{(CNT_W-3){1'b0}}, s_n_g};
            s_cb_c <= s_cb_c + {{(CNT_W-3){1'b0}}, s_n_b};
            s_ck_c <= s_ck_c + {{(CNT_W-3){1'b0}}, s_n_k};
            s_beat <= s_beat + 20'd1;
        end
    end

    // ======================================================================
    //  ⑥ 帧末锁存选中槽特征（★ C1 瘦身：不再逐槽扫 16 遍，直接算选中槽）
    //     顺序不变：锁存排在切分 FSM【之前】（那时子框还是上一帧的，与门控同源）
    // ======================================================================
    localparam [2:0] F_IDLE=3'd0, F_RUN=3'd1, F_DONE=3'd2, F_WAITG=3'd3;
    reg [2:0]  s_fst;
    reg [4:0]  s_step;
    reg [10:0] s_sw, s_sh;
    reg [4:0]  s_msb;
    reg [3:0]  s_shf;
    reg [NUM_W-1:0] s_num, s_den;
    reg        s_blank_f, s_ar_ok_f, s_vld_f;
    reg [7:0]  s_sig_r, s_sig_g, s_sig_b, s_sig_k;

    function [4:0] msb22;
        input [21:0] v;
        integer k;
        begin
            msb22 = 5'd0;
            for (k = 0; k < 22; k = k + 1)
                if (v[k]) msb22 = k[4:0];
        end
    endfunction

    function [7:0] norm8;
        input [21:0] v;
        input [3:0]  sh;
        reg [21:0] t;
        begin
            t = v >> sh;
            norm8 = (t > 22'd255) ? 8'd255 : t[7:0];
        end
    endfunction

    wire [10:0] q_wpx = {s_sw[7:0], 3'b000};
    wire [10:0] q_hpx = {s_sh[7:0], 3'b000};
    wire [15:0] q_w17 = ({5'd0, q_wpx} << 4) + {5'd0, q_wpx};
    wire [15:0] q_h10 = ({5'd0, q_hpx} << 3) + ({5'd0, q_hpx} << 1);
    wire [15:0] q_h14 = ({5'd0, q_hpx} << 4) - ({5'd0, q_hpx} << 1);

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_fst <= F_IDLE; s_step <= 5'd0; s_clr <= 1'b0;
            O_done <= 1'b0;
            O_sel_v <= 1'b0; O_sel_pixcnt <= 22'd0; O_sel_colsig <= 32'd0;
            O_sel_blank <= 1'b0; O_sel_ar_ok <= 1'b0;
            O_sel_x1 <= 12'd0; O_sel_x2 <= 12'd0; O_sel_y1 <= 12'd0; O_sel_y2 <= 12'd0;
        end else begin
            s_clr  <= 1'b0;
            O_done <= 1'b0;
            case (s_fst)
                F_IDLE: if (s_vend) begin               // 帧末立即锁存（切分排在它之后）
                    s_fst <= F_RUN; s_step <= 5'd0;
                end
                F_RUN: begin
                    case (s_step)
                        5'd0: begin
                            s_sw  <= (sb_x2 >= sb_x1) ? (sb_x2 - sb_x1 + 8'd1) : 8'd0;
                            s_sh  <= (sb_y2 >= sb_y1) ? (sb_y2 - sb_y1 + 8'd1) : 8'd0;
                            s_msb <= msb22(s_cnt);
                            O_sel_x1 <= {sb_x1, 3'b000};
                            O_sel_x2 <= {sb_x2, 3'b000} + 12'd7;
                            O_sel_y1 <= {sb_y1, 3'b000};
                            O_sel_y2 <= {sb_y2, 3'b000} + 12'd7;
                            s_step<= 5'd1;
                        end
                        5'd1: begin     // cnt*100 = cnt*64 + cnt*32 + cnt*4（移位加法）
                            s_num <= ({6'd0, s_cnt} << 6)
                                   + ({6'd0, s_cnt} << 5)
                                   + ({6'd0, s_cnt} << 2);
                            s_step<= 5'd2;
                        end
                        5'd2: begin     // 分母 = 命中拍数 * 4 像素/拍 * BLANK_PCT
                            s_den <= {8'd0, s_beat} << DENS_SH;
                            s_step<= 5'd3;
                        end
                        5'd3: begin
                            s_shf     <= (s_msb > 5'd7) ? (s_msb[3:0] - 4'd7) : 4'd0;
                            s_blank_f <= (s_num < s_den);
                            s_ar_ok_f <= (q_w17 >= q_h10) && (q_w17 <= q_h14);
                            s_vld_f   <= s_sv[sel_i] && (s_sw >= MIN_BLK[10:0])
                                                     && (s_sh >= MIN_BLK[10:0])
                                                     && (s_cnt != {CNT_W{1'b0}});
                            s_step    <= 5'd4;
                        end
                        5'd4: begin
                            s_sig_r <= norm8(s_cr_c, s_shf);
                            s_sig_g <= norm8(s_cg_c, s_shf);
                            s_sig_b <= norm8(s_cb_c, s_shf);
                            s_sig_k <= norm8(s_ck_c, s_shf);
                            s_step  <= 5'd5;
                        end
                        default: begin
                            O_sel_v      <= s_vld_f;
                            O_sel_pixcnt <= s_cnt;
                            O_sel_colsig <= {s_sig_r, s_sig_g, s_sig_b, s_sig_k};
                            O_sel_blank  <= s_blank_f;
                            O_sel_ar_ok  <= s_ar_ok_f;
                            s_fst        <= F_WAITG;    // ★ 交棒给阶段 2 计数
                            s_step       <= 5'd0;
                        end
                    endcase
                end
                F_WAITG: if (s_gdone) s_fst <= F_DONE;   // 等 ⑧ 的计数 FSM 跑完
                F_DONE: begin
                    O_done <= 1'b1;
                    s_clr  <= 1'b1;         // 下一拍清累加器（已进消隐期）
                    s_fst  <= F_IDLE;
                    s_step <= 5'd0;
                end
                default: s_fst <= F_IDLE;
            endcase
        end
    end

    // ======================================================================
    //  ⑦ 选中槽的输出已由帧末 FSM 直接写入（见 ⑥），这里不再需要 16 选 1 多路器
    // ======================================================================

    // ======================================================================
    //  ⑧ 阶段 2：细粒度数子图（4x4 格位图数"筒" + 逐像素列投影数"条"）
    // ----------------------------------------------------------------------
    //  ★ 为什么要在识别模块内部自己建位图：这里拿到的是【像素流】（4 像素/拍），
    //    不受 frame_diff 的 8x8 块粒度限制。4x4 像素 = 1 格 -> 一张立牌 ≈ 17x23 格，
    //    一个筒子 ≈ 5 格，数 1..9 够用；圆点是 2D 排布，必须用 2D 位图。
    //  ★ 竹条之间有缝：逐像素列投影的"峰数"就是列方向并排的竖直结构数（更省更稳）。
    //  ★ 只数【选中槽】那一张牌：位图和投影各只有一份（480 位 + 80x7 位），
    //    换槽后下一帧就出新数（轮播 1.5 秒，够）。
    //  ★ 全程没有一个算术 *：坐标全是移位/比较，计数全是累加（规则 4）。
    //  ★ C1 瘦身：帧末先算选中槽特征（6 拍，不再是 16 槽 x 6 拍），再跑这里。
    //  ★ 帧末顺序：计数排在 ⑥"帧末锁存特征"【之后】（那时切分几何才稳定），
    //    一次跑 6 + 1 + (480+24) + 16 + 40 + 41 + 1 ≈ 610 拍；
    //    720p60 的垂直消隐有 30 行 x 320 拍 = 9600 拍，余量充足。
    //  ★ 连通域用【即时重标号】而不是"合并表 + 收尾并查集"：
    //    发现一个格同时挨着两个不同标签时，当场把"输的"标签在上/本两行里
    //    就地改成"赢的"标签，尺寸并过去、输的清零。这样任何时刻标签都是唯一的，
    //    收尾只要数"尺寸 >= DOT_MIN 的标签个数"，不需要第二遍。
    // ======================================================================
    localparam integer PROJ_W = BMP_W * 4;          // ★ 逐像素列投影（C2 未采用：1 格 = 4 列）
    localparam [4:0]   CCL_W  = BMP_W;              // 位图宽（格）
    localparam [4:0]   CCL_H  = BMP_H;              // 位图高（格）
    localparam [4:0]   CCL_WM = CCL_W - 5'd1;       // 最右一列
    localparam [4:0]   CCL_HM = CCL_H - 5'd1;       // 最下一行
    localparam [3:0]   LAB_MAX= 4'd15;              // 标签上限（0 = 空）
    localparam [4:0]   LABMAX5= LAB_MAX;            // 同上，5 位版本（给计数器比较用）
    localparam [5:0]   NLAB   = 6'd16;              // 标签数组长度 = 0..15
    localparam [6:0]   PROJ_W7= PROJ_W;             // 投影列数（7 位）
    localparam [6:0]   PROJ_WM= PROJ_W - 1;
    localparam [6:0]   BARW7  = BAR_MIN_W;
    localparam [6:0]   BARG7  = BAR_MIN_G;
    localparam [8:0]   DOTMIN9= DOT_MIN;

    // ---------- 选中槽的子框（块坐标）-> 4x4 格坐标系 ----------
    //   （sel_i / sb_x1..sb_y2 已经在 ③ 声明，供 ROI 门控共用）
    // 绝对格坐标：一拍 = 一格列（4 像素）；4 行 = 一格行
    wire [10:0] s_ccell = {s_bc_g, 1'b0}   + {10'd0, s_lb_c[0]};
    wire [10:0] s_rcell = {s_brow_g, 1'b0} + {10'd0, s_line_c[2]};
    wire [10:0] s_kx1   = {3'd0, sb_x1, 1'b0};
    wire [10:0] s_ky1   = {3'd0, sb_y1, 1'b0};
    wire [10:0] s_kx2   = {3'd0, sb_x2, 1'b0} + 11'd1;   // 块 x2 覆盖到 2*x2+1 格列
    wire [10:0] s_ky2   = {3'd0, sb_y2, 1'b0} + 11'd1;

    wire s_in_box = s_sv[sel_i]
                 && (s_ccell >= s_kx1) && (s_ccell <= s_kx2)
                 && (s_rcell >= s_ky1) && (s_rcell <= s_ky2);
    wire [10:0] s_rel_cc = s_ccell - s_kx1;
    wire [10:0] s_rel_rr = s_rcell - s_ky1;
    // 位图窗口内的门控（子框比位图大时只数左/上的一块，其余裁掉）
    wire s_in_bmp = s_in_box && (s_rel_cc < CCL_W) && (s_rel_rr < CCL_H);

    // ---------- ① 格行累加器：本格行里每格的前景像素数（4 行 x 4 像素 = 最多 16）----------
    reg  [4:0] s_rcnt [0:BMP_W-1];
    reg  [4:0] s_rowidx;                    // 正在累加/刚结束的格行号（子框内）
    reg        s_row_used;                  // 本格行有没有采到样本
    reg        s_rowok;                     // 行号在位图范围内
    reg        s_commit_d;                  // 格行结束（行末且 line[1:0]==3）后一拍

    wire s_line_end4 = I_pix_valid && I_pix_last && (s_line_c[1:0] == 2'b11);

    integer ci;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (ci = 0; ci < BMP_W; ci = ci + 1) s_rcnt[ci] <= 5'd0;
        end else if (I_pix_valid) begin
            for (ci = 0; ci < BMP_W; ci = ci + 1) begin
                if (s_commit_d)
                    // ★ 本拍同时是"上一格行的收尾"和"新一行第 0 拍"：不能简单清零，
                    //   要写成"这一拍的新贡献"（否则子框从第 0 格列开始时丢一拍）
                    s_rcnt[ci] <= (s_in_bmp && (ci == s_rel_cc)) ? {2'd0, s_n_fg} : 5'd0;
                else if (s_in_bmp && (ci == s_rel_cc))
                    s_rcnt[ci] <= s_rcnt[ci] + {2'd0, s_n_fg};
            end
        end
    end

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_commit_d <= 1'b0; s_row_used <= 1'b0; s_rowok <= 1'b0; s_rowidx <= 5'd0;
        end else begin
            s_commit_d <= s_line_end4;
            if (s_commit_d)                       s_row_used <= 1'b0;
            else if (I_pix_valid && s_in_bmp)     s_row_used <= 1'b1;
            if (I_pix_valid && s_in_box && !s_commit_d) begin
                s_rowidx <= s_rel_rr[4:0];
                s_rowok  <= (s_rel_rr < CCL_H);
            end
        end
    end

    // ---------- ② 位图（4x4 格）：格行结束时把本行阈值化后写进去 ----------
    reg  [BMP_W-1:0] s_bmp_row [0:BMP_H-1];
    reg              s_bmpclr;
    wire [BMP_W-1:0] s_occrow;

    genvar gw;
    generate
        for (gw = 0; gw < BMP_W; gw = gw + 1) begin : OCCROW
            assign s_occrow[gw] = (s_rcnt[gw] >= CELL_THR);
        end
    endgenerate

    integer bi;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (bi = 0; bi < BMP_H; bi = bi + 1) s_bmp_row[bi] <= {BMP_W{1'b0}};
        end else if (s_bmpclr) begin
            for (bi = 0; bi < BMP_H; bi = bi + 1) s_bmp_row[bi] <= {BMP_W{1'b0}};
        end else if (s_commit_d && s_row_used && s_rowok) begin
            for (bi = 0; bi < BMP_H; bi = bi + 1)
                if (bi == s_rowidx) s_bmp_row[bi] <= s_occrow;
        end
    end

    // ---------- ③ 逐像素列投影（索：竹条之间的缝在列方向形成峰谷）----------
    //   ★ 逐像素列（用户指定的粒度）：一格列 = 4 个像素列 = 4 个计数器。
    //     计数上限 = 4 像素/列/行 x BMP_H 行(4 行/格) <= 4*96 = 384 -> 用 9 位会翻倍；
    //     实际一行只贡献 1 个像素/列，故上限 = BMP_H*4 = 96 -> 7 位够。
    reg  [6:0] s_proj [0:PROJ_W-1];
    reg        s_projclr;

    integer pg;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (pg = 0; pg < PROJ_W; pg = pg + 1) s_proj[pg] <= 7'd0;
        end else if (s_projclr) begin
            for (pg = 0; pg < PROJ_W; pg = pg + 1) s_proj[pg] <= 7'd0;
        end else if (I_pix_valid && s_in_bmp) begin
            // 一拍 = 4 个连续像素列；格列 s_rel_cc 对应 4*cc .. 4*cc+3
            for (pg = 0; pg < BMP_W; pg = pg + 1)
                if (pg == s_rel_cc) begin
                    if (s_fg[0]) s_proj[4*pg    ] <= s_proj[4*pg    ] + 7'd1;
                    if (s_fg[1]) s_proj[4*pg + 1] <= s_proj[4*pg + 1] + 7'd1;
                    if (s_fg[2]) s_proj[4*pg + 2] <= s_proj[4*pg + 2] + 7'd1;
                    if (s_fg[3]) s_proj[4*pg + 3] <= s_proj[4*pg + 3] + 7'd1;
                end
        end
    end

    // ---------- ④ 帧末计数 FSM（清位图 -> 连通域 -> 统计 -> 列投影峰）----------
    localparam [3:0] G_IDLE =4'd0, G_CLR  =4'd1, G_SCAN =4'd2, G_ROWEND=4'd3,
                     G_STAT =4'd4, G_PMAX =4'd5, G_PCNT =4'd6, G_LAST  =4'd7;

    reg  [3:0]  s_gst;
    reg  [4:0]  s_cc, s_cr;                 // 连通域扫描的列 / 行
    reg  [4:0]  s_nlab;                     // 已分配标签数
    reg  [4:0]  s_gl;                       // 统计遍历
    reg  [7:0]  s_pcol;                     // 投影扫描列
    reg  [6:0]  s_pmax, s_thr, s_run, s_gap;    // 投影值 7 位
    reg  [4:0]  s_bars;
    reg         s_inhi, s_cand;
    reg  [8:0]  s_dots, s_cells, s_maxc;
    reg  [8:0]  s_sz  [0:15];               // 每个标签的格数（合并时就地累加）
    reg  [3:0]  s_clab [0:BMP_W-1];         // 本行标签（0 = 空）
    reg  [3:0]  s_plab [0:BMP_W-1];         // 上一行标签
    reg  [BMP_W-1:0] s_pocc;                // 上一行占据位
    reg         s_gdone;

    wire [BMP_W-1:0] g_clab_nz;
    generate
        for (gw = 0; gw < BMP_W; gw = gw + 1) begin : CLABNZ
            assign g_clab_nz[gw] = (s_clab[gw] != 4'd0);
        end
    endgenerate

    // 8 邻域：上一行的上左/上/上右 + 本行的左
    wire [4:0] cc_p1 = (s_cc == CCL_WM) ? CCL_WM : (s_cc + 5'd1);
    wire [4:0] cc_m1 = (s_cc == 5'd0)   ? 5'd0   : (s_cc - 5'd1);
    wire       g_occ = s_bmp_row[s_cr][s_cc];
    wire [3:0] g_ul  = ((s_cc != 5'd0)   && s_pocc[cc_m1]) ? s_plab[cc_m1] : 4'd0;
    wire [3:0] g_uc  = (s_pocc[s_cc])                      ? s_plab[s_cc]  : 4'd0;
    wire [3:0] g_ur  = ((s_cc != CCL_WM) && s_pocc[cc_p1]) ? s_plab[cc_p1] : 4'd0;
    wire [3:0] g_lf  = (s_cc != 5'd0) ? s_clab[cc_m1] : 4'd0;

    function [3:0] min_nz4;                 // 取非零候选里的最小标签（全 0 时返回 15）
        input [3:0] a, b, c, d;
        reg   [3:0] m;
        begin
            m = 4'd15;
            if ((a != 4'd0) && (a < m)) m = a;
            if ((b != 4'd0) && (b < m)) m = b;
            if ((c != 4'd0) && (c < m)) m = c;
            if ((d != 4'd0) && (d < m)) m = d;
            min_nz4 = m;
        end
    endfunction

    wire [3:0] g_surv = min_nz4(g_ul, g_uc, g_ur, g_lf);
    wire       g_any  = ((g_ul | g_uc | g_ur | g_lf) != 4'd0);
    // 需要合并的候选：★ 只有"本格被占据"时才谈合并 —— 空格子两侧的邻居
    // 在 8 邻域判定里会同时出现（左上/上右），不 gate 就会把两根竹条并成一根
    wire g_rl_ul = g_occ && (g_ul != 4'd0) && (g_ul != g_surv);
    wire g_rl_uc = g_occ && (g_uc != 4'd0) && (g_uc != g_surv) && (g_uc != g_ul);
    wire g_rl_ur = g_occ && (g_ur != 4'd0) && (g_ur != g_surv) && (g_ur != g_ul) && (g_uc != g_ur);
    wire g_rl_lf = g_occ && (g_lf != 4'd0) && (g_lf != g_surv) && (g_lf != g_ul) && (g_lf != g_uc) && (g_lf != g_ur);

    wire [8:0] g_szadd = (g_rl_ul ? s_sz[g_ul] : 9'd0) + (g_rl_uc ? s_sz[g_uc] : 9'd0)
                       + (g_rl_ur ? s_sz[g_ur] : 9'd0) + (g_rl_lf ? s_sz[g_lf] : 9'd0);
    wire [3:0] g_newlab = !g_occ ? 4'd0
                        : (!g_any ? ((s_nlab < LAB_MAX) ? (s_nlab[3:0] + 4'd1) : LAB_MAX)
                                  : g_surv);
    // 投影单元号夹紧（多跑的那一列不是真列，用 0 号列顶替，值由外层选 0）
    wire [6:0] g_pidx = (s_pcol[6:0] <= PROJ_WM) ? s_pcol[6:0] : 7'd0;
    wire [6:0] g_pval = (s_pcol[6:0] <= PROJ_WM) ? s_proj[g_pidx] : 7'd0;

    integer si2;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_gst <= G_IDLE; s_cc <= 5'd0; s_cr <= 5'd0; s_nlab <= 5'd0; s_gl <= 5'd0;
            s_pcol <= 8'd0; s_pmax <= 7'd0; s_thr <= 7'd0; s_run <= 7'd0; s_gap <= 7'd0;
            s_bars <= 5'd0; s_inhi <= 1'b0; s_cand <= 1'b0;
            s_dots <= 9'd0; s_cells <= 9'd0; s_maxc <= 9'd0;
            s_pocc <= {BMP_W{1'b0}}; s_gdone <= 1'b0;
            s_bmpclr <= 1'b0; s_projclr <= 1'b0;
            O_sel_dots <= 5'd0; O_sel_bars <= 5'd0; O_sel_cells <= 9'd0; O_sel_maxc <= 9'd0;
            for (si2 = 0; si2 < NLAB; si2 = si2 + 1) s_sz[si2] <= 9'd0;
            for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                s_clab[si2] <= 4'd0; s_plab[si2] <= 4'd0;
            end
        end else begin
            s_bmpclr  <= 1'b0;
            s_projclr <= 1'b0;
            case (s_gst)
                // ---- 等帧末特征锁存完（F_WAITG），开始清场 ----
                G_IDLE: begin
                    // ★ s_gdone 是"跑完"脉冲：只保持一拍，否则下一帧 F_WAITG 立刻放行，
                    //   计数 FSM 就再也不会启动了
                    if (s_gdone) s_gdone <= 1'b0;
                    if ((s_fst == F_WAITG) && !s_gdone) begin
                    s_cc <= 5'd0; s_cr <= 5'd0; s_gl <= 5'd0; s_nlab <= 5'd0;
                    s_cells <= 9'd0; s_dots <= 9'd0; s_maxc <= 9'd0;
                    s_bars <= 5'd0; s_inhi <= 1'b0; s_run <= 7'd0; s_gap <= 7'd0; s_cand <= 1'b0;
                    s_pcol <= 8'd0; s_pmax <= 7'd0; s_thr <= 7'd0;
                    s_pocc <= {BMP_W{1'b0}};
                    for (si2 = 0; si2 < NLAB; si2 = si2 + 1) s_sz[si2] <= 9'd0;
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                        s_clab[si2] <= 4'd0; s_plab[si2] <= 4'd0;
                    end
                    s_gst <= G_CLR;
                end
                end
                G_CLR: s_gst <= G_SCAN;                     // 清位图/投影（上面已经打了一拍脉冲）
                // ---- 8 邻域连通域标记（一格一拍）----
                G_SCAN: begin
                    // ① 合并：把输掉的标签在上/本两行里就地改成存活标签
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                        if (g_rl_ul && (s_plab[si2] == g_ul)) s_plab[si2] <= g_surv;
                        if (g_rl_uc && (s_plab[si2] == g_uc)) s_plab[si2] <= g_surv;
                        if (g_rl_ur && (s_plab[si2] == g_ur)) s_plab[si2] <= g_surv;
                        if (g_rl_lf && (s_clab[si2] == g_lf)) s_clab[si2] <= g_surv;
                    end
                    // ② 本格标签（写在合并之后，同拍时以后者为准）
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1)
                        if (si2 == s_cc) s_clab[si2] <= g_newlab;
                    // ③ 尺寸：本格计入它的最终标签（新块 = g_newlab，否则 = 存活标签）
                    //    ★ 新块时 g_surv 是 min_nz4 的"全 0 缺省值 15"，不能拿它当标签用
                    for (si2 = 0; si2 < NLAB; si2 = si2 + 1) begin
                        if (g_occ && (si2 == g_newlab))
                            s_sz[si2] <= s_sz[si2] + g_szadd + 9'd1;
                        if (g_rl_ul && (si2 == g_ul)) s_sz[si2] <= 9'd0;
                        if (g_rl_uc && (si2 == g_uc)) s_sz[si2] <= 9'd0;
                        if (g_rl_ur && (si2 == g_ur)) s_sz[si2] <= 9'd0;
                        if (g_rl_lf && (si2 == g_lf)) s_sz[si2] <= 9'd0;
                    end
                    // ④ 新标签 / 占据格数
                    if (g_occ && !g_any && (s_nlab < LAB_MAX)) s_nlab <= s_nlab + 5'd1;
                    if (g_occ) s_cells <= s_cells + 9'd1;
                    if (s_cc == CCL_WM) s_gst <= G_ROWEND;
                    else                s_cc  <= s_cc + 5'd1;
                end
                // ---- 行末：本行标签 -> 上一行 ----
                G_ROWEND: begin
                    s_pocc <= g_clab_nz;
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                        s_plab[si2] <= s_clab[si2];
                        s_clab[si2] <= 4'd0;
                    end
                    s_cc <= 5'd0;
                    if (s_cr == CCL_HM) begin s_gl <= 5'd0; s_gst <= G_STAT; end
                    else begin s_cr <= s_cr + 5'd1; s_gst <= G_SCAN; end
                end
                // ---- 统计：连通块数（>= DOT_MIN 格）与最大块 ----
                G_STAT: begin
                    if (s_sz[s_gl] >= DOTMIN9) s_dots <= s_dots + 9'd1;
                    if (s_sz[s_gl] > s_maxc)   s_maxc <= s_sz[s_gl];
                    if (s_gl == LABMAX5) begin
                        s_pcol <= 8'd0; s_pmax <= 7'd0; s_gst <= G_PMAX;
                    end else s_gl <= s_gl + 5'd1;
                end
                // ---- 列投影：先找最大峰 ----
                G_PMAX: begin
                    if (s_proj[s_pcol[6:0]] > s_pmax) s_pmax <= s_proj[s_pcol[6:0]];
                    if (s_pcol[6:0] == PROJ_WM) begin
                        // 门限 = max(固定底噪, 最大峰的一半)
                        s_thr  <= (s_pmax > (BAR_MIN_V << 1)) ? {1'b0, s_pmax[6:1]} : BAR_MIN_V;
                        s_pcol <= 8'd0;
                        s_bars <= 5'd0; s_inhi <= 1'b0; s_run <= 7'd0; s_gap <= 7'd0; s_cand <= 1'b0;
                        s_gst  <= G_PCNT;
                    end else s_pcol <= s_pcol + 8'd1;
                end
                // ---- 列投影：数峰（峰宽 >= BAR_MIN_W，峰距 < BAR_MIN_G 的两个峰算一个）----
                G_PCNT: begin
                    // 多跑一个单元（s_pcol > PROJ_WM 时当成 0），保证最后一根竹条也能收尾
                    if (g_pval >= s_thr) begin
                        if (!s_inhi) begin
                            s_inhi <= 1'b1;
                            s_run  <= 7'd1;
                            s_cand <= (s_bars == 5'd0) || (s_gap >= BARG7);
                        end else s_run <= s_run + 7'd1;
                        s_gap <= 7'd0;
                    end else begin
                        if (s_inhi) begin
                            s_inhi <= 1'b0;
                            if (s_cand && (s_run >= BARW7) && (s_bars != 5'd15))
                                s_bars <= s_bars + 5'd1;
                        end else if (s_gap != 7'd127) s_gap <= s_gap + 7'd1;
                    end
                    if (s_pcol[6:0] == PROJ_W7) s_gst <= G_LAST;
                    else                   s_pcol <= s_pcol + 8'd1;
                end
                // ---- 锁存结果（属于本帧选中槽那一张牌），并清掉位图/投影给下一帧 ----
                G_LAST: begin
                    O_sel_dots  <= (s_dots > 9'd15) ? 5'd15 : s_dots[4:0];
                    O_sel_bars  <= s_bars;
                    O_sel_cells <= s_cells;
                    O_sel_maxc  <= s_maxc;
                    // ★ 清场必须放在【扫描之后】：位图里装的就是本帧的结果，
                    //   清早了（放 G_IDLE）CCL 只能看到一张空图
                    s_bmpclr    <= 1'b1;
                    s_projclr   <= 1'b1;
                    s_gdone     <= 1'b1;
                    s_gst       <= G_IDLE;
                end
                default: s_gst <= G_IDLE;
            endcase
        end
    end
endmodule
