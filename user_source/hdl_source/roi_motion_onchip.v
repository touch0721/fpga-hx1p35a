`timescale 1ns / 1ps
// ============================================================================
//  roi_motion_onchip.v — 固定 ROI 的背景帧差分"有没有牌"检测（两路）
// ----------------------------------------------------------------------------
//  用途：牌永远摆在两个固定的矩形框里（主框 ≈ 13 张，右侧小框另有一路）。
//        用【背景帧差分】判断 ROI 里"有没有东西"，作为识别模块的启动触发。
//
//  ★ 为什么比 frame_diff_onchip 省得多：
//    · 只在两个固定 ROI 内做：不做全画面扫描、不做多目标、不需要运动位图 /
//      3x3 开运算 / 邻域投票 / 重叠合并 / 多框输出；
//    · 判据就是一个数：ROI 内"块平均与背景差超阈值"的块个数 >= THR -> 有牌；
//    · 噪点靠"个数阈值"滤掉（孤立一两块进不了阈值），所以开运算整块可以删。
//    · 两个 ROI 行范围相同 -> 行/拍计数与门控逻辑共用一份。
//
//  数据流（ISP 输出域，与 Y 严格同拍，4 像素/拍）：
//    8x8 块：一行 8 像素 = 2 拍；块列累加器把 ROI 的 8 行加起来（共 64 像素）
//      -> ROI 最后一行结束后逐块结算：平均 = 累加 >> 6（纯移位）与背景比
//      -> |差| > DIFF_THR 记 1 个运动块，按主框/小框分别累加
//      -> 帧末与 THR_A / THR_B 比 -> present；连续 HOLD 帧不达标才熄（防抖）
//
//  KEY4（I_bg_cap）：拨上 = 抓背景（把当前 ROI 块平均写进背景），
//                    拨下 = 冻结背景开始检测；没抓过背景前 O_bg_rdy=0、不报 present
//
//  ★ 零乘法：块平均是右移；地址/边界全是常数比较
//  ★ 存储用寄存器不用 ERAM：主框 124 + 小框 9 = 133 个块列，
//    133 x (16 位累加 + 8 位背景) ≈ 3.2K FF —— 器件 FF 只用 47%，
//    用 FF 换掉原来 frame_diff 的 17 块 ERAM 很划算，也免了 RAM 读延迟的麻烦。
// ============================================================================
module roi_motion_onchip #(
    parameter integer IMG_W    = 1280,
    parameter integer IMG_H    = 720,
    parameter [7:0]   DIFF_THR = 8'd40,      // 块平均绝对差阈值（空桌 vs 有牌）
    parameter integer THR_A    = 40,         // 主框：运动【列条】数 >= 它 -> 有牌（主框只有 124 条！）
    parameter integer THR_B    = 3,          // 小框阈值（小框只有 9 条！）
    parameter integer HOLD     = 8,          // 连续多少帧不达标才熄（防抖，约 0.13 秒）
    // ★ 平均的右移位数 = ceil(log2(8 x ROI 高))，让 avg = sum >> AVG_SH 落在 0..255
    //   真设计：ROI 高 112 像素 -> 一个竖条 = 8x112 = 896 像素 -> AVG_SH = 10（÷1024）
    //   注意这是"近似整除"（0.875 倍），全画面统一缩放，只需把 DIFF_THR 按比例调即可
    parameter integer AVG_SH   = 10,
    // 两个 ROI 的像素范围（★ 必须是 8 的倍数对齐，边界落在块边界上）
    parameter integer AX1      = 56,         // 主框左右（块列 7..130）
    parameter integer AX2      = 1047,
    parameter integer BX1      = 1096,       // 小框左右（块列 137..145）
    parameter integer BX2      = 1167,
    parameter integer CY1      = 240,        // 两框共用的行范围（块行 30..43）
    parameter integer CY2      = 351
) (
    input  wire        I_pix_clk,
    input  wire        I_rst_n,
    input  wire        I_pix_valid,
    input  wire        I_pix_last,
    input  wire        I_pix_user,
    input  wire [31:0] I_y_now,              // 4 像素 Y（与 valid/last/user 严格同拍）
    input  wire        I_en,                 // SW4 使能
    input  wire        I_bg_cap,             // KEY4：1 = 抓背景，0 = 冻结并检测

    output reg         O_a_present,          // 主框：有牌
    output reg         O_b_present,          // 小框：有东西
    output reg  [11:0] O_a_count,            // 主框本帧运动块数（诊断 / 现场标阈值）
    output reg  [11:0] O_b_count,            // 小框本帧运动块数
    output reg         O_bg_rdy              // 背景已抓
);
    // ------------------------------------------------------------------
    //  几何常量（全部参数推出，综合期就是常数，不占逻辑）
    //   ★ 注意：不要对 integer 参数做部分选择（会得到 x），所以先定宽再比较
    // ------------------------------------------------------------------
    localparam integer LN_MAX = IMG_H - 1;
    localparam integer BC_A   = (AX2 - AX1 + 1) / 8;   // 主框块列数 = 124
    localparam integer BC_B   = (BX2 - BX1 + 1) / 8;   // 小框块列数 = 9
    localparam integer NBLK   = BC_A + BC_B;           // 块列总数 = 133
    localparam [7:0]   CA_L   = AX1 / 8;               // 主框块列 7
    localparam [7:0]   CA_H   = AX2 / 8;               //          130
    localparam [7:0]   CB_L   = BX1 / 8;               // 小框块列 137
    localparam [7:0]   CB_H   = BX2 / 8;               //          145
    localparam [7:0]   CR_L   = CY1 / 8;               // 块行 30
    localparam [7:0]   CR_H   = CY2 / 8;               //      43
    localparam [7:0]   BBASE  = BC_A;                  // 小框在块列数组里的基址 = 124
    localparam [7:0]   NBLK8  = NBLK;                  // 133
    localparam [7:0]   NBLK_M = NBLK - 1;              // 132
    localparam [11:0]  THRA12 = THR_A;
    localparam [11:0]  THRB12 = THR_B;
    localparam [3:0]   HOLD4  = HOLD;

    // ------------------------------------------------------------------
    //  ① 行/拍计数（行末用 I_pix_last 归零拍计数）
    // ------------------------------------------------------------------
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
    wire        s_vend = I_pix_valid && I_pix_last && (s_line == LN_MAX[10:0]);
    // ★ 帧首那一拍要【组合性】当第 0 拍（寄存器要等时钟沿才清零，否则每帧漏一拍）
    wire [9:0]  s_lb_c = I_pix_user ? 10'd0 : s_lb;
    wire [10:0] s_ln_c = I_pix_user ? 11'd0 : s_line;
    wire [7:0]  s_bcol = s_lb_c[9:1];        // 块列
    wire [7:0]  s_brow = s_ln_c[10:3];       // 块行

    // ------------------------------------------------------------------
    //  ② 采样门控 + 块地址
    // ------------------------------------------------------------------
    wire s_row_ok = (s_brow >= CR_L) && (s_brow <= CR_H);
    wire s_in_a   = s_row_ok && (s_bcol >= CA_L) && (s_bcol <= CA_H);
    wire s_in_b   = s_row_ok && (s_bcol >= CB_L) && (s_bcol <= CB_H);
    wire s_in_any = s_in_a || s_in_b;
    wire [7:0] s_blkaddr = s_in_a ? (s_bcol - CA_L) : (BBASE + (s_bcol - CB_L));
    // ★ 位宽必须够：4 个像素 x 255 = 1020，要 10 位（写 9 位会被截断成 mod 512，
    //   实测亮场 200x4=800 被截成 288，平均就变成 72）
    wire [9:0] s_bsum    = I_y_now[7:0] + I_y_now[15:8] + I_y_now[23:16] + I_y_now[31:24];

    // ROI 最后一行的行末（★ 必须是块行的【最后一行】，否则一个块行会触发 8 次，
    // 每次结算都把只累了一半的 sum 读走又清零 —— 实测就是这个 bug 让平均只有 1/2~1/8）
    wire s_roi_end = I_pix_valid && I_pix_last && (s_brow == CR_H) && (s_ln_c[2:0] == 3'b111);

    // ------------------------------------------------------------------
    //  ③ 列条累加 / 背景 / 结算（同一个 always 块：数组只能有一个写口）
    //   ★ 一个"列条" = 8 像素宽 x ROI 高（真设计 8x112 = 896 像素）。
    //     之所以不按 8x8 块存：ROI 高 112 像素 = 14 个块行，按 (块行,块列) 存要
    //     14x133 = 1862 项、光地址译码就上千 LUT；按列条只存 133 项。
    //     代价：平均是"近似整除"（>>AVG_SH），全屏统一缩放，调 DIFF_THR 即可。
    //   ★ 累加与背景合成一个 26 位数组（[25:8]=累加 18 位，[7:0]=背景），
    //     这样地址译码只做一份（省约 350 LUT）。
    // ------------------------------------------------------------------
    reg  [25:0] s_mem [0:NBLK-1];
    reg         s_cst;                       // 结算中
    reg  [7:0]  s_cs;                        // 结算地址 0..NBLK-1
    reg  [11:0] s_ba, s_bb;                  // 本帧运动列条计数（主框 / 小框）

    // 结算用组合量
    wire [17:0] s_acc  = s_mem[s_cs][25:8];
    wire [7:0]  s_bgc  = s_mem[s_cs][7:0];
    wire [7:0]  s_avgq = s_acc[AVG_SH + 7 -: 8];
    wire [8:0]  s_diff = (s_avgq >= s_bgc) ? ({1'b0, s_avgq} - {1'b0, s_bgc})
                                           : ({1'b0, s_bgc} - {1'b0, s_avgq});
    wire        s_mot  = (s_diff > {1'b0, DIFF_THR});
    wire        s_is_b = (s_cs >= BBASE);

    integer k;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (k = 0; k < NBLK; k = k + 1) s_mem[k] <= 26'd0;
            s_cst <= 1'b0; s_cs <= 8'd0; s_ba <= 12'd0; s_bb <= 12'd0;
            O_a_count <= 12'd0; O_b_count <= 12'd0;
        end else begin
            // ---- 采样：本拍 4 像素的 Y 加进对应列条（背景字段原样保留）----
            if (I_pix_valid && s_in_any && !s_cst) begin
                for (k = 0; k < NBLK; k = k + 1)
                    if (k == s_blkaddr)
                        s_mem[k] <= { s_mem[k][25:8] + {8'd0, s_bsum}, s_mem[k][7:0] };
            end
            // ---- ROI 末行结束：清计数并从 0 号列条开始结算 ----
            if (s_roi_end) begin
                s_cst <= 1'b1; s_cs <= 8'd0; s_ba <= 12'd0; s_bb <= 12'd0;
            end
            // ---- 结算：一列条一拍（抓背景 / 比阈值 / 清累加）----
            else if (s_cst) begin
                // 抓背景：把当前平均写进背景字段（KEY4 拨上期间持续刷新）
                // 否则：背景冻结，|平均-背景| > DIFF_THR 记一个运动列条
                s_mem[s_cs] <= { 18'd0, I_bg_cap ? s_avgq : s_bgc };
                if (!I_bg_cap && O_bg_rdy && s_mot) begin
                    if (s_is_b) s_bb <= s_bb + 12'd1;
                    else        s_ba <= s_ba + 12'd1;
                end
                if (s_cs == NBLK_M) s_cst <= 1'b0;
                else                s_cs  <= s_cs + 8'd1;
            end
            // ---- 帧末锁存计数 ----
            if (s_vend) begin
                O_a_count <= s_ba;
                O_b_count <= s_bb;
            end
        end
    end

    // ------------------------------------------------------------------
    //  ④ 阈值判决 + 防抖
    // ------------------------------------------------------------------
    reg [3:0] s_ha, s_hb;                    // 连续不达标帧数
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            O_a_present <= 1'b0; O_b_present <= 1'b0;
            s_ha <= 4'd0; s_hb <= 4'd0; O_bg_rdy <= 1'b0;
        end else begin
            if (I_bg_cap) O_bg_rdy <= 1'b1;      // 抓过背景就置位
            if (s_vend) begin
                if (s_ba >= THRA12) begin O_a_present <= 1'b1; s_ha <= 4'd0; end
                else if (s_ha >= HOLD4) O_a_present <= 1'b0;
                else s_ha <= s_ha + 4'd1;

                if (s_bb >= THRB12) begin O_b_present <= 1'b1; s_hb <= 4'd0; end
                else if (s_hb >= HOLD4) O_b_present <= 1'b0;
                else s_hb <= s_hb + 4'd1;
            end
        end
    end
endmodule
