`timescale 1ns / 1ps
// ============================================================================
//  frame_diff_onchip.v — 基于【背景帧】的物体检测 ★多目标：邻域投票法★
// ----------------------------------------------------------------------------
//  一、检测
//    8×8 块降采样 -> 块平均 Y；与背景帧同位置块平均比较
//    KEY4：I_bg_cap=1 持续抓取背景；=0 冻结背景并检测
//    |本帧块平均 − 背景块平均| > DIFF_THR  ->  该块是"运动块"
//
//  二、多目标分离（邻域投票法，帧内实时，不需要运动图）
//    对每一个运动块 (bx, by)，向 16 个目标各投一票：
//        t_valid[j]==0                          -> 投"新目标"票
//        (bx,by) 落在 t 的邻域之外              -> 投"新目标"票
//        否则                                   -> 不投
//    · 全票（16'hFFFF）      -> 这是一个全新的目标，占用一个空槽
//    · 部分票（>0）          -> 落在"未投票那些目标"的邻域内 -> 扩展它们的 bbox
//    ★ 邻域判定同时用 x 和 y，所以"上下分开、左右重叠"的物体也能分开
//    ★ 帧首清空目标列表（每帧重新检测）
//    ★ 最多 16 个内部目标，对外输出前 4 个
//
//  三、I_en=0（SW4 关）时全部复位
// ============================================================================
module frame_diff_onchip #(
    parameter integer IMG_W    = 1280,
    parameter integer IMG_H    = 720,
    parameter integer BC       = IMG_W/8,          // 块列数 = IMG_W/8
    parameter integer BR       = IMG_H/8,          // 块行数 = IMG_H/8
    parameter [7:0]   DIFF_THR = 8'd12,        // 块平均绝对差阈值
    parameter integer NT       = 32,           // ★ 目标表容量 16 -> 32（放多个物体时表被占满 -> 全不识别）
    parameter integer MAXO     = 8,            // 对外输出方框数
    parameter integer MARGIN   = 2,            // 邻域外扩（块数），使跟踪更稳
    parameter integer MIN_HITS = 4,            // ★ 目标至少被命中几次才算有效（2->4，减少小框）
    parameter integer OPEN_EN  = 1,            // ★ 投票前对运动图做 3×3 开运算（去噪点）
    parameter integer MIN_SIZE = 4,            // ★ 目标最小尺寸（块数）：宽高都要 >= 这个值
    parameter integer HOLD_FRM = 8,           // ★ 输出框保持帧数（防闪烁）
    parameter integer XCOMP = 1,              // ★ 投票坐标 X 补偿（实测标定）
    parameter integer YCOMP = 2               // ★ 投票坐标 Y 补偿（最后一项不能有逗号）
) (
    input  wire        I_pix_clk,
    input  wire        I_rst_n,
    input  wire        I_pix_valid,
    input  wire        I_pix_last,
    input  wire        I_pix_user,
    input  wire [31:0] I_y_now,
    input  wire        I_en,
    input  wire        I_bg_cap,

    output reg  [MAXO-1:0]    O_bv,
    output reg  [12*MAXO-1:0] O_bx1,
    output reg  [12*MAXO-1:0] O_bx2,
    output reg  [12*MAXO-1:0] O_by1,
    output reg  [12*MAXO-1:0] O_by2,

    output reg  [15:0] O_frame_cnt,
    output reg  [11:0] O_mot_cnt,
    output reg  [4:0]  O_obj_cnt,
    // ★ 调试：运动二值图按【坐标查询】而不是流式输出
    //   因为帧差跟 ISP 输出同拍，而 mixer 跟 HDMI 显示同拍，两者差整整一帧，
    //   流式输出会被 mixer 当成"当前帧的图"用 -> 横条纹 + 频闪。
    //   所以要先把位图存下来，mixer 按显示坐标来查。
    input  wire [14:0] I_q_addr,        // 查询地址 = 块行×WPR + 块列/32
    output reg  [31:0] O_q_word,        // 该字（同步读，1 拍延迟）
    output reg         O_bg_rdy
);
    localparam integer LB_MAX = (IMG_W/4) - 1;
    localparam integer LN_MAX = IMG_H - 1;
    localparam integer WPR    = (BC + 31) / 32;   // 每行几个 32 位字

    wire [9:0] s_beat_sum = I_y_now[7:0] + I_y_now[15:8] +
                            I_y_now[23:16] + I_y_now[31:24];

    // ======================================================================
    //  ① 行/拍计数
    // ======================================================================
    reg  [8:0]  s_lb;
    reg  [10:0] s_line;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n)        begin s_lb <= 9'd0; s_line <= 11'd0; end
        else if (!I_en)      begin s_lb <= 9'd0; s_line <= 11'd0; end
        else if (I_pix_user) begin s_lb <= 9'd0; s_line <= 11'd0; end
        else if (I_pix_valid) begin
            if (s_lb == LB_MAX[8:0]) begin
                s_lb <= 9'd0;
                if (s_line == LN_MAX[10:0]) s_line <= 11'd0;
                else                        s_line <= s_line + 11'd1;
            end else s_lb <= s_lb + 9'd1;
        end
    end
    wire s_vend = I_pix_valid && (s_lb == LB_MAX[8:0]) && (s_line == LN_MAX[10:0]);

    wire [7:0]  s_bc    = s_lb[8:1];
    wire        s_bcend = s_lb[0];
    wire [7:0]  s_brow  = s_line[10:3];
    wire        s_lastr = (s_line[2:0] == 3'd7);

    reg s_user_d;
    always @(posedge I_pix_clk) s_user_d <= I_pix_user;
    wire s_fstart = I_pix_user & ~s_user_d;

    // ======================================================================
    //  ② 块累加 -> 块平均
    // ======================================================================
    reg  [9:0]  s_sum_a;
    wire [10:0] s_rowsum = s_sum_a + s_beat_sum;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) s_sum_a <= 10'd0;
        else if (!I_en) s_sum_a <= 10'd0;
        else if (I_pix_valid) begin
            if (s_bcend) s_sum_a <= 10'd0;
            else         s_sum_a <= s_beat_sum;
        end
    end

    reg  [7:0]  s_bc_r, s_brow_r;
    reg  [13:0] s_rs_r;
    reg         s_lastr_r, s_val_r, s_firstr_r;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_bc_r <= 8'd0; s_brow_r <= 8'd0; s_rs_r <= 14'd0;
            s_lastr_r <= 1'b0; s_val_r <= 1'b0; s_firstr_r <= 1'b0;
        end else begin
            s_val_r    <= I_en && I_pix_valid && s_bcend;
            s_bc_r     <= s_bc;
            s_brow_r   <= s_brow;
            s_rs_r     <= {3'd0, s_rowsum};
            s_lastr_r  <= s_lastr;
            s_firstr_r <= (s_line[2:0] == 3'd0);
        end
    end

    reg  [15:0] s_sum_ram [0:BC-1];
    wire [15:0] s_part = s_sum_ram[s_bc_r];
    wire [15:0] s_new  = s_firstr_r ? {2'd0, s_rs_r} : (s_part + {2'd0, s_rs_r});

    reg  [7:0]  s_bc_s, s_brow_s, s_avg_s;
    reg         s_settle_s;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_bc_s <= 8'd0; s_brow_s <= 8'd0; s_avg_s <= 8'd0; s_settle_s <= 1'b0;
        end else begin
            s_bc_s     <= s_bc_r;
            s_brow_s   <= s_brow_r;
            s_avg_s    <= s_new[13:6];
            s_settle_s <= s_val_r && s_lastr_r;
        end
    end
    // ★ 方案 A：块地址用累加（按光栅顺序，完成一个块 +1）
    //   不写死 160/5 也不用乘法器，任何 BC/BR 都对
    reg  [14:0] s_paddr_r;
    always @(posedge I_pix_clk or negedge I_rst_n)
        if (!I_rst_n)         s_paddr_r <= 15'd0;
        else if (s_fstart)    s_paddr_r <= 15'd0;
        else if (s_settle_s)  s_paddr_r <= s_paddr_r + 15'd1;   // 一个块的平均完成
    wire [14:0] s_paddr = s_paddr_r;

    always @(posedge I_pix_clk) begin
        if (I_en && s_val_r) begin
            if (s_lastr_r) s_sum_ram[s_bc_r] <= 16'd0;
            else           s_sum_ram[s_bc_r] <= s_new;
        end
    end

    // ======================================================================
    //  ③ 背景帧
    // ======================================================================
    reg  [7:0] s_bg_ram [0:(BC*BR)-1];
    reg        s_bg_rdy = 1'b0;
    wire       s_bg_write = I_bg_cap | ~s_bg_rdy;

    reg  [14:0] s_paddr_s;
    reg  [7:0]  s_avg_s2;
    reg         s_settle_s2;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_paddr_s <= 15'd0; s_avg_s2 <= 8'd0; s_settle_s2 <= 1'b0;
        end else begin
            s_paddr_s   <= s_paddr;
            s_avg_s2    <= s_avg_s;
            s_settle_s2 <= s_settle_s;
        end
    end

    reg  [7:0] s_bg_val, s_avg_cmp;
    reg        s_settle_cmp;
    always @(posedge I_pix_clk) begin
        s_bg_val     <= s_bg_ram[s_paddr_s];
        s_avg_cmp    <= s_avg_s2;
        s_settle_cmp <= s_settle_s2;
        if (I_en && s_settle_s2 && s_bg_write) s_bg_ram[s_paddr_s] <= s_avg_s2;
    end
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n)   s_bg_rdy <= 1'b0;
        else if (!I_en) s_bg_rdy <= 1'b0;
        else if (s_vend && s_bg_write) s_bg_rdy <= 1'b1;
    end
    always @(posedge I_pix_clk) O_bg_rdy <= s_bg_rdy;

    // ======================================================================
    //  ④ 运动判定（块级）
    // ======================================================================
    wire [7:0] s_da = s_avg_cmp, s_db = s_bg_val;
    wire       s_diff_big = (s_da > s_db) ? ((s_da - s_db) > DIFF_THR)
                                          : ((s_db - s_da) > DIFF_THR);
    wire       s_mot   = s_diff_big && ~s_bg_write && s_bg_rdy;
    wire       s_mot_v = I_en && s_settle_cmp && s_mot;

    reg  [7:0] s_bc_c, s_brow_c;
    always @(posedge I_pix_clk) begin
        s_bc_c <= s_bc_s; s_brow_c <= s_brow_s;
    end

    // ======================================================================
    //  ④b 开运算（先腐蚀后膨胀，3×3）—— 在投票之前去掉孤立噪点
    //      3 个 240 位行位图；每来一个运动块在 mrow_cur 对应位置置 1；
    //      块行切换时对 3 行做腐蚀+膨胀，得到"往回第 2 行"的开运算结果，
    //      再由一个小状态机逐位送进投票（by = brow-2）。
    // ======================================================================
    reg [BC-1:0] mrow_cur, mrow_p1, mrow_p2;   // 运动位（当前行 / 前1行 / 前2行）
    reg [BC-1:0] ero_p2,   ero_p1;             // 腐蚀结果（前2行 / 前1行）
    reg [7:0]    s_brow_d;
    reg          s_rowend;
    always @(posedge I_pix_clk) begin
        s_brow_d <= s_brow_c;
        s_rowend <= (s_brow_c != s_brow_d) && (s_brow_c != 8'd0);
    end

    // 逐位送出用的小状态机
    reg [7:0]  s_obx;          // 正在送的列
    reg [7:0]  s_oby;          // 这一批对应的块行（= 往回第 2 行）
    reg        s_obusy;
    reg [BC-1:0] s_obits;      // 开运算结果（待逐位送出）
    integer ci2;
    wire [BC-1:0] s_ero_mid;   // 中间行腐蚀（先用组合算，见下）

    // 3×3 腐蚀：对 3 行分别做 1×3 腐蚀，再按列与
    genvar gc;

    // ★ 膨胀也必须是 3×3（否则开运算后物体在垂直方向会短 1 块 = 8 像素，
    //   表现就是"框不住物体上下"）：
    //   先垂直膨胀（三行的 3×3 腐蚀结果相或），再横向 1×3 膨胀
    wire [BC-1:0] ero_r2h, ero_r1h, ero_r0h, dil_v, dil_all;
    generate
        for (gc = 0; gc < BC; gc = gc + 1) begin : EROH
            // 三行各自的横向 1×3 腐蚀（垂直方向在 ero_all 里与过）
            wire l2 = (gc == 0)    ? 1'b0 : mrow_p2[gc-1];
            wire r2 = (gc == BC-1) ? 1'b0 : mrow_p2[gc+1];
            wire l1 = (gc == 0)    ? 1'b0 : mrow_p1[gc-1];
            wire r1 = (gc == BC-1) ? 1'b0 : mrow_p1[gc+1];
            wire l0 = (gc == 0)    ? 1'b0 : mrow_cur[gc-1];
            wire r0 = (gc == BC-1) ? 1'b0 : mrow_cur[gc+1];
            assign ero_r2h[gc] = l2 & mrow_p2[gc] & r2;
            assign ero_r1h[gc] = l1 & mrow_p1[gc] & r1;
            assign ero_r0h[gc] = l0 & mrow_cur[gc] & r0;
        end
    endgenerate
    assign ero_all = ero_r2h & ero_r1h & ero_r0h;      // 3×3 腐蚀（完整）
    assign dil_v   = ero_r2h | ero_r1h | ero_r0h;      // ★ 垂直 3 行膨胀
    generate
        for (gc = 0; gc < BC; gc = gc + 1) begin : DIL
            wire l = (gc == 0)    ? 1'b0 : dil_v[gc-1];
            wire r = (gc == BC-1) ? 1'b0 : dil_v[gc+1];
            assign dil_all[gc] = l | dil_v[gc] | r;    // ★ 横向 1×3 膨胀
        end
    endgenerate

    // 行缓存推进 + 收集开运算结果
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            mrow_cur <= {BC{1'b0}}; mrow_p1 <= {BC{1'b0}}; mrow_p2 <= {BC{1'b0}};
            ero_p2 <= {BC{1'b0}}; ero_p1 <= {BC{1'b0}};
        end else if (!I_en || s_fstart || s_vend) begin
            mrow_cur <= {BC{1'b0}}; mrow_p1 <= {BC{1'b0}}; mrow_p2 <= {BC{1'b0}};
            ero_p2 <= {BC{1'b0}}; ero_p1 <= {BC{1'b0}};
        end else begin
            // 运动块写入当前行
            if (s_mot_v) mrow_cur[s_bc_c] <= 1'b1;
            // 块行切换：整行移位 + 记录开运算结果
            if (s_rowend) begin
                mrow_p2  <= mrow_p1;
                mrow_p1  <= mrow_cur;
                mrow_cur <= {BC{1'b0}};
                s_obits  <= OPEN_EN ? dil_all : mrow_p2;   // 关掉开运算就用原始运动位
                s_oby    <= (s_brow_c == 8'd0) ? 8'd0 : (s_brow_c - 8'd1);
                // 注意：参与运算的 mrow_p2 是"往回第 2 行"，所以 by = brow-1-... 见下方说明
            end
        end
    end

    // ★ 逐位送出：**每 2 拍送一个块**
    //   为什么必须留一拍间隙：投票在第 1 拍用当前目标表做判断，更新在第 2 拍才落地。
    //   若两个运动块背靠背投票，第 2 个块读到的还是旧表（第 1 个块的 bbox 扩展
    //   没写进去）-> 投票不准、同一物体被重复建成多个目标。
    //   一行有 240×16 = 3840 拍，240 个块每块 2 拍只需 480 拍，余量极大。
    reg s_obgap;      // 0 = 本拍送块；1 = 间隔拍
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_obusy <= 1'b0; s_obx <= 8'd0; s_obgap <= 1'b0;
        end else if (!I_en) begin
            s_obusy <= 1'b0; s_obx <= 8'd0; s_obgap <= 1'b0;
        end else begin
            if (s_rowend && !s_obusy) begin
                s_obusy <= 1'b1; s_obx <= 8'd0; s_obgap <= 1'b0;
            end else if (s_obusy) begin
                if (s_obgap == 1'b0) begin
                    s_obgap <= 1'b1;                 // 下一拍是间隔拍
                end else begin
                    s_obgap <= 1'b0;                 // 下一拍送下一个块
                    if (s_obx >= (BC-1)) s_obusy <= 1'b0;
                    else                 s_obx <= s_obx + 8'd1;
                end
            end
        end
    end

    // 送出的这一位就是一个"运动块"（坐标 s_obx, s_oby）；间隔拍不送
    wire s_open_mot = s_obusy && !s_obgap && s_obits[s_obx];

    // ★ 运动二值图（调试用）：当前拍的 4 个像素各自查自己所在块的位
    //   像素列 = s_lb*4 + k，块列 = 像素列 >> 3
    //   mrow_p2 对应的是"往回第 2 行"，所以画面会比实际下移 2 个块（16 行），
    //   调试观察无所谓。
    wire [8:0] px0 = {s_lb, 2'b00};                 // 本拍第 0 个像素的列号
    wire [7:0] b0 = px0[8:3];
    wire [7:0] b1 = (px0 + 9'd1) >> 3;
    wire [7:0] b2 = (px0 + 9'd2) >> 3;
    wire [7:0] b3 = (px0 + 9'd3) >> 3;

    // ======================================================================
    //  ④c 运动位图 RAM（调试显示用）：240×135 位，32 位打包 -> 8 字/行 × 135 行
    //      在开运算逐位送出时顺便写进去；mixer 按显示坐标查询
    // ======================================================================
    integer di;

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
        end else if (!I_en || s_fstart || s_vend) begin
        end else begin
            if (s_obusy && !s_obgap) begin
                // 这一拍送出的是块列 s_obx 的运动位
                if (s_obx[4:0] == 5'd31) begin
                end
            end
        end
    end

    // ★ 真正的写入
    always @(posedge I_pix_clk) begin
    end

    // ★ 查询端口（同步读，1 拍延迟）
    
    // ======================================================================
    //  ⑤ 目标表（16 个）
    // ======================================================================
        // (c) 运动位图 RAM（供 SW4 二值图显示）
    // ★ 调试位图已删除（SW4 二值图功能已取消）
    always @(posedge I_pix_clk) O_q_word <= 32'd0;

reg        t_val [0:NT-1];
    reg [9:0]  t_x1  [0:NT-1], t_x2 [0:NT-1];
    reg [9:0]  t_y1  [0:NT-1], t_y2 [0:NT-1];
    reg [11:0] t_hit [0:NT-1];
    reg [5:0]  t_cnt;                  // 已用目标数（NT 最大 63）
    // 投票延迟一拍后使用（与目标表更新对齐）
    reg [7:0]  s_vbx, s_vby;
    reg        s_vote_v;
    reg [NT-1:0] s_vote;               // 每个目标投"新目标"票
    reg [NT-1:0] s_inslot;             // 部分票时：落在邻域内的目标（要扩展的）

    integer vi;
    // ---- 组合投票：每个运动块对 16 个目标投票 ----
    //   ★ 邻域判定必须写成"全加法"形式：
    //     (bx + M >= x1) && (bx <= x2 + M)
    //   若写成 (bx < x1 - M)，当 x1 < M 时 x1-M 变成负数，
    //   而无符号比较会把它当成极大的正数 -> 判定恒真 -> 所有目标都投"新目标"票。
    always @(*) begin
        for (vi = 0; vi < NT; vi = vi + 1) begin
            s_vote[vi] = ~t_val[vi] ||
                         ~(({3'd0, s_vbx} + MARGIN >= {1'd0, t_x1[vi]}) &&
                           ({3'd0, s_vbx}          <= {1'd0, t_x2[vi]} + MARGIN)) ||
                         ~(({3'd0, s_vby} + MARGIN >= {1'd0, t_y1[vi]}) &&
                           ({3'd0, s_vby}          <= {1'd0, t_y2[vi]} + MARGIN));
        end
    end
    wire s_allvote = &s_vote;          // 全票 -> 新目标

    integer ui;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            t_cnt <= 6'd0;
            s_vbx <= 8'd0; s_vby <= 8'd0; s_vote_v <= 1'b0;
            for (ui = 0; ui < NT; ui = ui + 1) begin
                t_val[ui] <= 1'b0;
                t_x1[ui] <= 10'd0; t_x2[ui] <= 10'd0;
                t_y1[ui] <= 10'd0; t_y2[ui] <= 10'd0;
                t_hit[ui] <= 12'd0;
            end
        end else if (!I_en) begin
            t_cnt <= 6'd0;
            for (ui = 0; ui < NT; ui = ui + 1) t_val[ui] <= 1'b0;
        end else begin
            // ---- 帧首清空目标表（每帧重新检测）----
            if (s_fstart) begin
                t_cnt <= 6'd0;
                for (ui = 0; ui < NT; ui = ui + 1) begin
                    t_val[ui] <= 1'b0;
                    t_hit[ui] <= 12'd0;
                end
            end
            // ---- 第 1 拍：锁存"开运算后送出的运动块"坐标（投票用）----
            s_vote_v <= s_open_mot;
            if (s_open_mot) begin
                // ★ 块坐标补偿必须限幅到 0：无符号减法在 s_obx<XCOMP / s_oby<YCOMP 时会下溢
                //   s_oby=1, YCOMP=2 -> 255 → 目标行坐标变成 255 → 屏上出现巨大框
                s_vbx <= (s_obx >= XCOMP[7:0]) ? (s_obx - XCOMP[7:0]) : 8'd0;
                s_vby <= (s_oby >= YCOMP[7:0]) ? (s_oby - YCOMP[7:0]) : 8'd0;
            end
            // ★ 补偿量是实测标定的：
            //   XCOMP=1：开运算逐位送出时列号比真实块列大 1，减 1 后框与物体完全对齐
            //   YCOMP=2：行号同样偏大 2（开运算取的是往回第 2 行），减 2 后上下边界一致
            // ---- 第 2 拍：按投票结果更新目标表 ----
            if (s_vote_v) begin
                if (s_allvote) begin
                    // 全票：新目标（占用第一个空槽）
                    // ★ 用显式循环"按 t_cnt 命中写入"，避免变量索引的非阻塞赋值
                    //   在部分工具里不生效（实测 iverilog 下 t_val[t_cnt]<=1 没写进去，
                    //   导致同一物体的后续块又被判成新目标）
                    if (t_cnt < NT[5:0]) begin
                        for (ui = 0; ui < NT; ui = ui + 1) begin
                            if (ui[4:0] == t_cnt) begin
                                t_val[ui] <= 1'b1;
                                t_x1[ui]  <= {2'd0, s_vbx};
                                t_x2[ui]  <= {2'd0, s_vbx};
                                t_y1[ui]  <= {2'd0, s_vby};
                                t_y2[ui]  <= {2'd0, s_vby};
                                t_hit[ui] <= 12'd1;
                            end
                        end
                        t_cnt <= t_cnt + 6'd1;
                    end
                end else begin
                    // 部分票：对"未投票（=落在邻域内）"的目标扩展 bbox
                    for (ui = 0; ui < NT; ui = ui + 1) begin
                        if (t_val[ui] && !s_vote[ui]) begin
                            if ({2'd0, s_vbx} < t_x1[ui]) t_x1[ui] <= {2'd0, s_vbx};
                            if ({2'd0, s_vbx} > t_x2[ui]) t_x2[ui] <= {2'd0, s_vbx};
                            if ({2'd0, s_vby} < t_y1[ui]) t_y1[ui] <= {2'd0, s_vby};
                            if ({2'd0, s_vby} > t_y2[ui]) t_y2[ui] <= {2'd0, s_vby};
                            t_hit[ui] <= t_hit[ui] + 12'd1;
                        end
                    end
                end
            end
        end
    end

    // ======================================================================
    //  ⑥ 帧末：把有效目标（hits >= MIN_HITS）整体输出到 O_b*
    // ======================================================================
    // ★ 候选目标（块坐标）与已输出框（像素坐标）的重叠判定
    //   候选的像素坐标 = 块坐标 << 3
    // ★ 用变量下标读数组时，必须放在 always @(*) 里；
    //   写成 wire 连续赋值在部分工具/综合器下会得到错误值 -> 屏上出现巨大框/错位框
    reg [11:0] c_x1, c_x2, c_y1, c_y2;
    always @(*) begin
        c_x1 = {4'd0, t_x1[s_scan][7:0]} << 3;
        c_x2 = ({4'd0, t_x2[s_scan][7:0]} << 3) + 12'd7;
        c_y1 = {4'd0, t_y1[s_scan][7:0]} << 3;
        c_y2 = ({4'd0, t_y2[s_scan][7:0]} << 3) + 12'd7;
    end

    reg [5:0]  s_sel;                  // 已选出几个
    reg [5:0]  s_scan;                 // 扫描目标表用（要能到 NT）
    reg [3:0]  s_fo;
    reg [3:0]  s_fo_dbg;
    always @(posedge I_pix_clk) s_fo_dbg <= s_fo;                   // ★ 输出了几个（2 位只能到 3，MAXO=8 会回绕）
    localparam [1:0] FO_IDLE=2'd0, FO_SCAN=2'd1, FO_DONE=2'd2;
    reg [3:0] s_hold [0:MAXO-1];       // ★ 每个输出框的保持计数（2 位装不下 HOLD_FRM=8）
    reg [1:0]  s_fo_st;
    integer qi, ok;

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_fo_st <= FO_IDLE; s_scan <= 6'd0; s_sel <= 6'd0; s_fo <= 4'd0;
            for (qi = 0; qi < MAXO; qi = qi + 1) s_hold[qi] <= 4'd0;
            O_bv <= {MAXO{1'b0}}; O_obj_cnt <= 5'd0;
            for (qi = 0; qi < MAXO; qi = qi + 1) begin
                O_bx1[((qi<<3)+(qi<<2)) +: 12] <= 12'd0; O_bx2[((qi<<3)+(qi<<2)) +: 12] <= 12'd0;
                O_by1[((qi<<3)+(qi<<2)) +: 12] <= 12'd0; O_by2[((qi<<3)+(qi<<2)) +: 12] <= 12'd0;
            end
        end else if (!I_en) begin
            s_fo_st <= FO_IDLE; O_bv <= {MAXO{1'b0}}; O_obj_cnt <= 5'd0;
        end else begin
            case (s_fo_st)
            FO_IDLE: begin
                if (s_vend) begin s_scan <= 6'd0; s_sel <= 6'd0; s_fo <= 4'd0;
                    // ★ 不清 O_bx1..O_by2：防闪烁保持(s_hold)还亮着时要继续用旧坐标，
                    //   清了会让"有效但坐标=0"的框画在左上角 -> 异常残框
                    s_fo_st <= FO_SCAN; end
                else begin
                    // 平时保持上一帧的结果（不在这里清零，否则显示期看不到框）
                end
            end
            FO_SCAN: begin
                if (s_scan >= NT[5:0]) begin s_fo_st <= FO_DONE; end
                else begin
                    // ★ 还要满足最小尺寸（宽、高都至少 MIN_SIZE 个块），滤掉极小的框
                    if (t_val[s_scan] && (t_hit[s_scan] >= MIN_HITS) &&
                        ((t_x2[s_scan] - t_x1[s_scan]) >= (MIN_SIZE-1)) &&
                        ((t_y2[s_scan] - t_y1[s_scan]) >= (MIN_SIZE-1))) begin
                        // ★ 去掉"重叠就扩展已有框"的分支：
                        //   3~4 个物体时它会链式扩展，把多个目标并成一个大框
                        //   （表现为右下巨大方框 / 框位置错乱）。
                        //   现在每个合格目标各占一个框，与 H 计数严格 1:1。
                        if (s_fo < MAXO) begin
                            // ★ 用 case 写固定位段：变量动态 part-select 写 reg 向量
                            //   在部分工具下会写错位（屏上出现巨大框/错位框）
                            O_bv[s_fo] <= 1'b1;
                            case (s_fo)
                            4'd0: begin O_bx1[0*12 +: 12]<=c_x1; O_bx2[0*12 +: 12]<=c_x2; O_by1[0*12 +: 12]<=c_y1; O_by2[0*12 +: 12]<=c_y2; end
                            4'd1: begin O_bx1[1*12 +: 12]<=c_x1; O_bx2[1*12 +: 12]<=c_x2; O_by1[1*12 +: 12]<=c_y1; O_by2[1*12 +: 12]<=c_y2; end
                            4'd2: begin O_bx1[2*12 +: 12]<=c_x1; O_bx2[2*12 +: 12]<=c_x2; O_by1[2*12 +: 12]<=c_y1; O_by2[2*12 +: 12]<=c_y2; end
                            4'd3: begin O_bx1[3*12 +: 12]<=c_x1; O_bx2[3*12 +: 12]<=c_x2; O_by1[3*12 +: 12]<=c_y1; O_by2[3*12 +: 12]<=c_y2; end
                            4'd4: begin O_bx1[4*12 +: 12]<=c_x1; O_bx2[4*12 +: 12]<=c_x2; O_by1[4*12 +: 12]<=c_y1; O_by2[4*12 +: 12]<=c_y2; end
                            4'd5: begin O_bx1[5*12 +: 12]<=c_x1; O_bx2[5*12 +: 12]<=c_x2; O_by1[5*12 +: 12]<=c_y1; O_by2[5*12 +: 12]<=c_y2; end
                            4'd6: begin O_bx1[6*12 +: 12]<=c_x1; O_bx2[6*12 +: 12]<=c_x2; O_by1[6*12 +: 12]<=c_y1; O_by2[6*12 +: 12]<=c_y2; end
                            default: begin O_bx1[7*12 +: 12]<=c_x1; O_bx2[7*12 +: 12]<=c_x2; O_by1[7*12 +: 12]<=c_y1; O_by2[7*12 +: 12]<=c_y2; end
                            endcase
                            s_fo <= s_fo + 4'd1;
                        end
                    end
                    s_scan <= s_scan + 6'd1;
                end
            end
            FO_DONE: begin
                // ★ 防闪烁：本帧没选中的框不立刻熄灭，先减保持计数；
                //   连续 HOLD_FRM 帧都无效才真正熄灭。
                for (qi = 0; qi < MAXO; qi = qi + 1) begin
                    if (qi >= s_fo) begin
                        if (s_hold[qi] == 0) O_bv[qi] <= 1'b0;
                        else begin
                            s_hold[qi] <= s_hold[qi] - 4'd1;
                            // 坐标保持不动（继续显示上一帧的框）
                        end
                    end else begin
                        s_hold[qi] <= HOLD_FRM[3:0];      // 本帧有效 -> 重置保持计数
                    end
                end
                O_obj_cnt <= s_fo;
                s_fo_st <= FO_IDLE;
            end
            default: s_fo_st <= FO_IDLE;
            endcase
        end
    end

    // ======================================================================
    //  ⑦ 诊断
    // ======================================================================
    reg [11:0] s_mot_run;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n)               s_mot_run <= 12'd0;
        else if (!I_en || s_fstart) s_mot_run <= 12'd0;   // 一帧内累计（开运算后的运动块）
        else if (s_open_mot)        s_mot_run <= s_mot_run + 12'd1;
    end
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin O_frame_cnt <= 16'd0; O_mot_cnt <= 12'd0; end
        else if (!I_en) begin O_frame_cnt <= 16'd0; O_mot_cnt <= 12'd0; end
        else if (s_vend) begin
            O_frame_cnt <= O_frame_cnt + 16'd1;
            O_mot_cnt   <= s_mot_run;
        end
    end

endmodule
