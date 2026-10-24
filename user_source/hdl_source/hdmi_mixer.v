module hdmi_mixer #(
    parameter H_OFFSET = 0,
    parameter V_OFFSET = 0,
    parameter IMG_WIDTH = 1600,
    parameter IMG_HEIGHT = 900,
    parameter integer DEBUG_MODE = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,

    // ↓ OSD 数字显示（左上角）
    input wire[11:0]  I_disp_h,        // KEY4 计数器值（被填充行数的历史最大值）
    input wire[11:0]  I_disp_d,        // 左右边界距离 = xr - xl + 1
    // ↓ logo 钩子（本版未接，留给以后搬 anlogic_logo_rom）
    input wire        I_logo_hit,
    input wire[23:0]  I_logo_color,
    // ★ SW1 新功能：显示帧差的二值图（按坐标查询位图）
    input wire        I_fdimg_en,
    output wire[14:0] O_fd_q_addr,      // 查询地址（用早一拍的坐标，数据与 S_x_2d 对齐）
    input  wire[31:0] I_fd_q_word,
    // ↓ 诊断：SW4 打开时画一个固定测试框，用来验证画框路径
    input wire        I_tbox_en,
    // ↓ 帧差法诊断信息（显示在左上角第 3/4 行）
    input wire[15:0]  I_fd_frame_cnt,   // 帧差收到的帧数：拨上 SW4 后必须一直在涨
    input wire[11:0]  I_fd_mot_cnt,     // 上一帧运动块数：恒 0 说明差分/阈值有问题
    input wire        I_fd_osd_en,      // ★ SW3：帧差模式（关识别：OSD/红框/绿框，只留帧差）
    // ↓ 多目标运动方框（SW4 打开时由 frame_diff_onchip 给出，最多 4 个）
    input wire[7:0]   I_mb_v,        // 每个方框的有效位
    input wire[95:0]  I_mb_x1,
    input wire[95:0]  I_mb_x2,
    input wire[95:0]  I_mb_y1,
    input wire[95:0]  I_mb_y2,

    // ★ 麻将牌识别特征（tile_recog_onchip 输出，准静态：每帧帧末更新一次）
    //   全部是 clk0 域的寄存器，本模块在 clk2 域只做"读"，SDC 里是跨域路径因此不计时序；
    //   轮播/选槽逻辑只喂寄存器，不在 S_x_2d -> OSD 像素这条路径上。
    input wire        I_tr_v,        // ★ 以下是【选中槽】的特征（选槽在识别域做）
    input wire[21:0]  I_tr_pixcnt,   // 前景像素数
    input wire[31:0]  I_tr_colsig,   // 颜色签名：[31:24]红 [23:16]绿 [15:8]蓝 [7:0]无彩
    input wire        I_tr_blank,    // 判为白板
    input wire        I_tr_ar_ok,    // 宽高比通过
    input wire[11:0]  I_tr_x1, I_tr_x2, I_tr_y1, I_tr_y2,   // 该槽矩形（像素坐标）
    // ★ 阶段 2：细粒度计数（4x4 格位图数筒 / 逐像素列投影数条），都属选中槽
    input wire[4:0]   I_tr_dots,     // 连通块数（筒候选）
    input wire[4:0]   I_tr_bars,     // 列投影峰数（条候选）
    input wire[8:0]   I_tr_cells,    // 位图占据格数（诊断）
    input wire[8:0]   I_tr_maxc,     // 最大连通块格数（诊断）
    input wire        I_tr_freeze,   // 拨上 = 冻结轮播（停在当前槽）
    // ★ 固定 ROI 检测结果（roi_motion_onchip）：用来给红框染绿 + OSD 显示计数
    input wire        I_a_present,
    input wire        I_b_present,
    input wire [11:0] I_a_count,
    input wire [11:0] I_b_count,
    input wire        I_tr_skipinv,  // 拨上 = 只轮播有效槽（★ 已改接 0，SW3 让给二值化）
    input wire        I_bin_en,      // ★ SW3：二值化显示（暗 = 白，亮 = 黑）
    output wire[3:0]  O_tr_sel,      // 当前看哪个槽（送识别域）

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg        S_window_active_1d;
    reg        S_window_active_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;

    wire S_window_active;
    assign S_window_active = I_video_de &&
                             (S_x >= H_OFFSET) &&
                             (S_x < (H_OFFSET + IMG_WIDTH)) &&
                             (S_y >= V_OFFSET) &&
                             (S_y < (V_OFFSET + IMG_HEIGHT));

    // Read exactly IMG_WIDTH pixels on each of IMG_HEIGHT lines.  The HDMI
    // active area may be larger than the stored frame, in which case remaining
    // pixels are emitted as black borders without consuming DDR data.
    assign O_video_rd_en = S_window_active;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else begin
            if(I_video_user)
                S_y <= 12'd0;
            else if(I_video_de && I_video_last)
                S_y <= S_y + 12'd1;

            if(!I_video_de || I_video_last)
                S_x <= 12'd0;
            else
                S_x <= S_x + 12'd1;
        end
    end

    // ================= OSD 数字显示 =================
    //   屏幕坐标延迟两拍，与 S_video_data_2d 对齐
    reg [11:0] S_x_1d, S_x_2d, S_y_1d, S_y_2d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0; S_x_2d <= 12'd0; S_y_1d <= 12'd0; S_y_2d <= 12'd0;
        end else begin
            S_x_1d <= S_x; S_x_2d <= S_x_1d;
            S_y_1d <= S_y; S_y_2d <= S_y_1d;
        end
    end

    localparam [11:0] OSD_X0 = 12'd8;
    localparam [11:0] OSD_Y0 = 12'd8;
    // ★ 识别结论（在下面的采样器里赋值）：2 = 匹配模板 / 1 = 全白（白板）/ 0 = 都不是
    //   ★ 先声明再用 —— OSD 组合逻辑在它前面，避免"先用后声明"
    localparam [11:0] OSD_NC = 12'd5;      // ★ 每行 5 字：第 0 行 'W'+宽2位+高2位

    //   ★ 一位数字：1 = 识别到（槽 0 = 最左那张，采样交点全为白 -> 白板）；0 = 没识别到
    wire        osd_in_top = (S_y_2d >= OSD_Y0) && (S_y_2d < (OSD_Y0 + 12'd224));
    wire        osd_in_bot = (S_y_2d >= 12'd360) && (S_y_2d < (12'd360 + 12'd144));   // 9 行
    wire osd_area = (S_x_2d >= OSD_X0) && (S_x_2d < (OSD_X0 + OSD_NC*12'd16)) &&
                    (osd_in_top || osd_in_bot);   // ★ 22 行（+Y +C）
    wire [11:0] osd_lx = S_x_2d - OSD_X0;
    wire [11:0] osd_ly = S_y_2d - OSD_Y0;
    wire [2:0]  osd_ci = osd_lx[7:4];      // 第几个字符 0..4（16 像素一个）
    wire [11:0] osd_ly_b = S_y_2d - 12'd360;
    wire [4:0]  osd_rw = osd_in_top ? {1'b0, osd_ly[7:4]} : (5'd14 + {1'b0, osd_ly_b[7:4]});      // 第几行 0..18（5 位）
    wire [3:0]  osd_row= osd_in_top ? osd_ly[3:0] : osd_ly_b[3:0];      // 字内行 0..15
    wire [3:0]  osd_col= osd_lx[3:0];      // 字内列 0..15（0=最左）

    // 双 dabble：12 位二进制 -> 4 位 BCD（每 4 位一个十进制位）
    function [15:0] bcd4;
        input [11:0] v;
        integer i;
        reg [27:0] s;
        begin
            s = {16'd0, v};
            for (i = 0; i < 12; i = i + 1) begin
                if (s[15:12] >= 4'd5) s[15:12] = s[15:12] + 4'd3;
                if (s[19:16] >= 4'd5) s[19:16] = s[19:16] + 4'd3;
                if (s[23:20] >= 4'd5) s[23:20] = s[23:20] + 4'd3;
                if (s[27:24] >= 4'd5) s[27:24] = s[27:24] + 4'd3;
                s = s << 1;
            end
            bcd4 = s[27:12];
        end
    endfunction

    // ★ 双 dabble 是很长的组合链（12 级）。这些量全是【准静态】（每帧才变一次），
    //   所以先寄存一拍再进 OSD 多路器 —— 否则整条链挂在 S_x_2d -> O_hdmi_data 的
    //   像素路径上，实测把 clk2（75 MHz）拖成 SWNS -0.681 ns、24 条违规。
    //   （I_disp_h / I_disp_d 那两行随投影法停用已经没有消费者，不再转换）
    wire [15:0] w_bcd_f  = bcd4(I_fd_frame_cnt[15:0]);        // 帧数（最多 65535）
    wire [15:0] w_bcd_m  = bcd4({4'd0, I_fd_mot_cnt});        // 运动块数
    wire [15:0] w_bcd_dt = bcd4({7'd0, I_tr_dots});           // 连通块数（筒候选）
    wire [15:0] w_bcd_br = bcd4({7'd0, I_tr_bars});           // 列投影峰数（条候选）
    wire [15:0] w_bcd_cl = bcd4({3'd0, I_tr_cells});          // 占据格数
    wire [15:0] w_bcd_mx = bcd4({3'd0, I_tr_maxc});           // 最大连通块格数

    reg  [15:0] bcd_f, bcd_m, bcd_pg, bcd_gg;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            bcd_f  <= 16'd0; bcd_m  <= 16'd0; bcd_pg <= 16'd0; bcd_gg <= 16'd0;
        end else begin
            bcd_f  <= w_bcd_f;
            bcd_m  <= w_bcd_m;
            bcd_pg <= {w_bcd_dt[7:0], w_bcd_br[7:0]};         // 'P' 行：块数 | 峰数
            bcd_gg <= {w_bcd_cl[7:0], w_bcd_mx[7:0]};         // 'G' 行：格数 | 最大块
        end
    end

    // ================= 麻将识别：轮播槽 0..15 =================
    //   SW2 拨上 = 冻结轮播；SW3 拨上 = 只在【无效】槽上快速跳过（停在有效槽）
    //   ★ 这段只驱动寄存器 s_tsel；OSD 读的是 s_tsel，不加深像素路径。
    reg  [27:0] s_cyc;
    reg  [3:0]  s_tsel;
    localparam [27:0] CYC_END = 28'd224_999_999;    // 约 1.5 秒 @150MHz
    wire s_slow = (s_cyc == CYC_END);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin s_cyc <= 28'd0; s_tsel <= 4'd0; end
        else begin
            if (s_slow) s_cyc <= 28'd0;
            else        s_cyc <= s_cyc + 28'd1;
            // SW3 打开时：只在"当前槽无效"时才前进 -> 有效槽会停住不动
            if (s_slow && !I_tr_freeze && (!I_tr_skipinv || !I_tr_v))
                s_tsel <= s_tsel + 4'd1;
        end
    end

    // ★ 选中槽的特征直接来自识别域，这里不再做选槽
    wire [21:0] tr_cnt = I_tr_pixcnt;
    wire [31:0] tr_sig = I_tr_colsig;
    wire        tr_val = I_tr_v;
    wire        tr_blk = I_tr_blank;
    wire        tr_ar  = I_tr_ar_ok;
    assign O_tr_sel = s_tsel;
    // 选中框（像素坐标，识别域已按 frame_diff 约定给出）；宽度按块算，供 A 行显示
    wire [7:0]  tr_wblk = (I_tr_x2[10:3] >= I_tr_x1[10:3])
                        ? (I_tr_x2[10:3] - I_tr_x1[10:3] + 8'd1) : 8'd0;
    wire [15:0] tr_hex4 = (tr_cnt > 22'd65535) ? 16'hFFFF : tr_cnt[15:0];      // 像素数（hex，饱和）
    wire [15:0] tr_hexc = {tr_sig[31:28], tr_sig[23:20], tr_sig[15:12], tr_sig[7:4]}; // 签名每桶高 4 位
    wire [15:0] tr_hexa = {1'b0, s_tsel, 1'b0, tr_ar, tr_blk, tr_val, tr_wblk};      // 槽号/标志/框宽(块)

    // 选字符码：第 0 个是字母，1..4 是数字（★ bcd_* 都是寄存过的，见上面）
    //   ★ 现在显示【选中牌位（槽）】的识别结果（14 个固定牌位一直跑）：
    //       A = 槽号(2 位) + 有效位 + 白板位 + 宽高比位（hex）
    //       P = 连通块数（筒候选）| 列投影峰数（条候选），各 2 位十进制
    //       G = 占据格数 | 最大连通块格数，各 2 位十进制
    reg [7:0]  osd_ch;
    reg [15:0] osd_val;
    reg [3:0]  osd_nib;
    // ★ 黑占比功能【已按用户要求删除】—— 改用投影熵方案
    //   详见 handover/投影熵_近似实现方案.txt

    // ========================================================================
    //  ★ 特征值识别（按用户方案）：红标志 + R(行峰) + C(列峰) + W±5 + H±5
    //    · 37 条参考项（1~34 + 赤五万35/赤五筒36/赤五条37），P 不参与判定（用户取消）
    //    · 串行查表：帧末跑 37 拍，比 37 组并行比较省约 10 倍 LUT
    //    · 输出（4 个 OSD 字符）：
    //        "0014" 这样的 2 位十进制编号 = 唯一命中
    //        "DDDD" = 命中多个（不可分）
    //        "FFFF" = 什么都没命中，也不是全零（白靠全零判 -> 32）
    //    · 全零（R=C=W=H=0）优先判"白" = 32
    // ========================================================================
    // ★ 4 项判据各自独立容差（模拟验证：宽/列差/行差可放到 ±4，高必须 ±3）
    localparam [11:0] WT_TOL = 12'd4;    // 宽 ±4
    localparam [11:0] HT_TOL = 12'd3;    // 高 ±3（放开会和"西/發"冲突）
    localparam [11:0] ZC_TOL = 12'd4;    // 列差 ±4
    localparam [11:0] ZR_TOL = 12'd4;    // 行差 ±4
    reg        mt_run;
    reg  [5:0] mt_i;                                     // 0..36
    reg  [6:0] mt_hits;                                  // 命中项数
    reg  [5:0] mt_id;                                    // 第一个命中的编号
    reg        mt_red; reg [3:0] mt_R, mt_C; reg [7:0] mt_W, mt_H;
    reg  [7:0] mt_zr, mt_zc;                             // ★ 行差 / 列差
    reg  [7:0] mt_ct;                              // ★ 真实列差（O 行）参考值，0 = 不检查
    reg  [7:0] mt_ol;                             // ★ 真实列差(O)下限：cdiff_t 必须 > mt_ol，0 = 不检查
    reg  [7:0] mt_oh;                             // ★ 真实列差(O)上限：cdiff_t 必须 <= mt_oh，0 = 不检查
    reg         mt_ft;                             // ★ 1 = 宽高按 ±2 收紧判（用于三万/九万这类邻近牌）
    reg  [5:0] mt_num;                                  // ★ 该条对应的牌号（1..37，跳过白 32）
    reg  [15:0] res_row;                                 // OSD 4 个字符（nibble 形式）

    // ---- 37 条参考特征（由用户数据表生成）----
    always @(*) begin
        mt_red = 1'b0; mt_R = 4'd0; mt_C = 4'd0; mt_W = 8'd0; mt_H = 8'd0; mt_zr = 8'd0; mt_zc = 8'd0; mt_num = 6'd0; mt_ct = 8'd0; mt_ol = 8'd0; mt_oh = 8'd0; mt_ft = 1'b0;
        case (mt_i)
            6'd0: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd41; mt_H=8'd81; mt_zr=8'd37; mt_zc=8'd56; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd1; end // 1 yiman
            6'd1: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd3; mt_W=8'd41; mt_H=8'd83; mt_zr=8'd35; mt_zc=8'd60; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd2; end // 2 erman
            6'd2: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd41; mt_H=8'd84; mt_zr=8'd33; mt_zc=8'd63; mt_ct=8'd62; mt_ft=1'b1; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd3; end // 3 sanwan
            6'd3: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd46; mt_H=8'd84; mt_zr=8'd42; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd4; end // 4 siwan
            6'd4: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd41; mt_H=8'd87; mt_zr=8'd43; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd5; end // 5 wuman
            6'd5: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd41; mt_H=8'd82; mt_zr=8'd33; mt_zc=8'd63; mt_ct=8'd71; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd6; end // 6 liuwan
            6'd6: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd36; mt_H=8'd80; mt_zr=8'd34; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd7; end // 7 qiwan
            6'd7: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd37; mt_H=8'd68; mt_zr=8'd32; mt_zc=8'd60; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd8; end // 8 bawan
            6'd8: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd2; mt_W=8'd37; mt_H=8'd87; mt_zr=8'd33; mt_zc=8'd63; mt_ct=8'd0; mt_ft=1'b1; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd9; end // 9 jiuwan
            6'd9: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd56; mt_H=8'd66; mt_zr=8'd56; mt_zc=8'd0; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd10; end // 10 yitong
            6'd10: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd2; mt_W=8'd37; mt_H=8'd86; mt_zr=8'd38; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd11; end // 11 ertong
            6'd11: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd56; mt_H=8'd86; mt_zr=8'd28; mt_zc=8'd0; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd12; end // 12 santong
            6'd12: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd2; mt_W=8'd56; mt_H=8'd87; mt_zr=8'd56; mt_zc=8'd0; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd60; mt_num=6'd13; end // 13 sitong
            6'd13: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd55; mt_H=8'd88; mt_zr=8'd56; mt_zc=8'd0; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd14; end // 14 wutong
            6'd14: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd2; mt_W=8'd45; mt_H=8'd87; mt_zr=8'd46; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd15; end // 15 liutong
            6'd15: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd54; mt_H=8'd85; mt_zr=8'd46; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd16; end // 16 qitong
            6'd16: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd44; mt_H=8'd87; mt_zr=8'd44; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd17; end // 17 batong
            6'd17: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd3; mt_W=8'd56; mt_H=8'd87; mt_zr=8'd56; mt_zc=8'd0; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd18; end // 18 jiutong
            6'd18: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd2; mt_W=8'd45; mt_H=8'd70; mt_zr=8'd41; mt_zc=8'd62; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd19; end // 19 yitiao
            6'd19: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd2; mt_W=8'd16; mt_H=8'd84; mt_zr=8'd16; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd20; end // 20 ertiao
            6'd20: begin mt_red=1'b0; mt_R=4'd3; mt_C=4'd2; mt_W=8'd54; mt_H=8'd84; mt_zr=8'd28; mt_zc=8'd41; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd21; end // 21 santiao
            6'd21: begin mt_red=1'b0; mt_R=4'd2; mt_C=4'd2; mt_W=8'd43; mt_H=8'd86; mt_zr=8'd31; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd22; end // 22 sitiao
            6'd22: begin mt_red=1'b1; mt_R=4'd3; mt_C=4'd1; mt_W=8'd54; mt_H=8'd87; mt_zr=8'd42; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd23; end // 23 wutiao
            6'd23: begin mt_red=1'b0; mt_R=4'd3; mt_C=4'd2; mt_W=8'd53; mt_H=8'd86; mt_zr=8'd43; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd24; end // 24 liutiao
            6'd24: begin mt_red=1'b0; mt_R=4'd3; mt_C=4'd2; mt_W=8'd49; mt_H=8'd56; mt_zr=8'd39; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd25; end // 25 qitiao
            6'd25: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd2; mt_W=8'd55; mt_H=8'd86; mt_zr=8'd52; mt_zc=8'd0; mt_ct=8'd0; mt_ol=8'd60; mt_oh=8'd0; mt_num=6'd26; end // 26 batiao
            6'd26: begin mt_red=1'b1; mt_R=4'd3; mt_C=4'd3; mt_W=8'd49; mt_H=8'd87; mt_zr=8'd38; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd27; end // 27 jiutiao
            6'd27: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd36; mt_H=8'd71; mt_zr=8'd37; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd28; end // 28 dong
            6'd28: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd45; mt_H=8'd54; mt_zr=8'd37; mt_zc=8'd61; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd29; end // 29 nan
            6'd29: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd41; mt_H=8'd55; mt_zr=8'd39; mt_zc=8'd45; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd30; end // 30 xi
            6'd30: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd47; mt_H=8'd46; mt_zr=8'd37; mt_zc=8'd54; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd31; end // 31 bei
            6'd31: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd43; mt_H=8'd51; mt_zr=8'd37; mt_zc=8'd45; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd33; end // 33 fa
            6'd32: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd1; mt_W=8'd36; mt_H=8'd35; mt_zr=8'd35; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd34; end // 34 zhong
            6'd33: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd2; mt_W=8'd41; mt_H=8'd55; mt_zr=8'd39; mt_zc=8'd45; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd30; end // 30 xi_alt (P=0102)
            6'd34: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd1; mt_W=8'd36; mt_H=8'd27; mt_zr=8'd35; mt_zc=8'd63; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd34; end // 34 zhong_alt (H=27)
            6'd35: begin mt_red=1'b1; mt_R=4'd1; mt_C=4'd1; mt_W=8'd41; mt_H=8'd81; mt_zr=8'd37; mt_zc=8'd56; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd1; end // 1 yiman_alt (P=0101)
            6'd36: begin mt_red=1'b0; mt_R=4'd1; mt_C=4'd1; mt_W=8'd43; mt_H=8'd51; mt_zr=8'd32; mt_zc=8'd45; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd33; end // 33 fa_alt (zr=32)
            default: begin mt_red=1'b0; mt_R=4'd0; mt_C=4'd0; mt_W=8'd0; mt_H=8'd0; mt_ct=8'd0; mt_ol=8'd0; mt_oh=8'd0; mt_num=6'd0; end
        endcase
    end

    // ---- 单条比较：红 + R + C + |W-We|<=5 + |H-He|<=5 ----
    // ★ 框位系数：越靠右的框实测尺寸略大，参考宽高按框号加偏移
    //   框0/1 -> +0，框2/3 -> +1，框4/5 -> +2
    // ★★ 按【二万】实测（以框0 为基准）标定的每框系数
    //    wh_coef：宽高各加（越靠右实测越大）
    //    zc_ofs ：列差参考值各减（越靠右实测列差越小）
    reg  [2:0]  wh_coef;
    reg  [2:0]  zc_ofs;
    always @(*) begin
        case (box_sel)
            4'd0:  begin wh_coef = 3'd0; zc_ofs = 3'd0; end
            4'd1:  begin wh_coef = 3'd1; zc_ofs = 3'd0; end
            4'd2:  begin wh_coef = 3'd2; zc_ofs = 3'd0; end
            4'd3:  begin wh_coef = 3'd2; zc_ofs = 3'd0; end
            4'd4:  begin wh_coef = 3'd3; zc_ofs = 3'd2; end
            4'd5:  begin wh_coef = 3'd3; zc_ofs = 3'd2; end
            4'd6:  begin wh_coef = 3'd3; zc_ofs = 3'd2; end
            4'd7:  begin wh_coef = 3'd1; zc_ofs = 3'd2; end   // ★ zc_ofs 与框6 一致 = 2
            4'd8:  begin wh_coef = 3'd2; zc_ofs = 3'd6; end
            4'd9:  begin wh_coef = 3'd2; zc_ofs = 3'd4; end
            4'd10: begin wh_coef = 3'd0; zc_ofs = 3'd3; end
            4'd11: begin wh_coef = 3'd0; zc_ofs = 3'd4; end
            4'd12: begin wh_coef = 3'd0; zc_ofs = 3'd2; end
            default: begin wh_coef = 3'd0; zc_ofs = 3'd2; end   // 框13
        endcase
    end
    wire [1:0]  box_coef  = wh_coef[1:0];        // 兼容原引用
    // 原映射（恢复时把上面那行注释掉、解开下面这块）：
    // wire [1:0] box_coef = (box_sel <= 4'd1)  ? 2'd0 :   // 框 0,1
    //                       (box_sel <= 4'd3)  ? 2'd1 :   // 框 2,3
    //                       (box_sel <= 4'd7)  ? 2'd3 :   // 框 4,5,6,7
    //                       (box_sel <= 4'd9)  ? 2'd2 :   // 框 8,9
    //                       (box_sel <= 4'd11) ? 2'd1 :   // 框 10,11
    //                                           2'd0;     // 框 12
    wire [11:0] mt_wref   = {4'd0, mt_W} + {10'd0, box_coef};
    wire [11:0] mt_href   = {4'd0, mt_H} + {10'd0, box_coef};
    wire [11:0] mt_dw = (pb_w_d > mt_wref) ? (pb_w_d - mt_wref) : (mt_wref - pb_w_d);
    wire [11:0] mt_dh = (pb_h_d > mt_href) ? (pb_h_d - mt_href) : (mt_href - pb_h_d);
    // ★ 实测约定（用户确认）：P 行前两位 = R(行峰)、后两位 = C(列峰)
    //   而硬件内部 pb_npc 数的是"列扫描里的峰"、pb_npr 数的是"行扫描里的峰"，
    //   两者与参考表的 R/C 正好对调，所以这里交叉比较。
    // ★ 直接比较：硬件 cdiff(列差) <-> 表里第一个 Z 值 mt_zc；rdiff(行差) <-> 第二个 mt_zr
    //   参考值为 0 表示该项测量不可靠 -> 不加条件（用户要求）
    // ★ 列差参考值也加同一个框位系数
    wire [7:0] mt_zcref = (mt_zc > 8'd60) ? mt_zc : (mt_zc - {5'd0, zc_ofs});   // ★ 按框号递减；63 饱和保持
    wire [7:0] mt_dzc = (cdiff > mt_zcref) ? (cdiff - mt_zcref) : (mt_zcref - cdiff);
    wire [7:0] mt_dzr = (rdiff > mt_zr) ? (rdiff - mt_zr) : (mt_zr - rdiff);
    wire       mt_zok = ((mt_zc == 8'd0) || (mt_dzc <= ZC_TOL[7:0])) &&
                        ((mt_zr == 8'd0) || (mt_dzr <= ZR_TOL[7:0]));
    // ★ 第 3 判据：真实列差（O 行）±3，参考值 0 表示不检查
    wire [7:0] mt_dct = (cdiff_t > mt_ct) ? (cdiff_t - mt_ct) : (mt_ct - cdiff_t);
    wire       mt_cok = (mt_ct == 8'd0) || (mt_dct <= 8'd3);
    wire       mt_ook = (mt_ol == 8'd0) || (cdiff_t > mt_ol);   // ★ O 下限判据
    wire       mt_ohk = (mt_oh == 8'd0) || (cdiff_t <= mt_oh);   // ★ O 上限判据
    wire [11:0] mt_wtol = mt_ft ? 12'd2 : WT_TOL;   // ★ 三万/九万 用 ±2 收紧
    wire       mt_hok = (mt_dh <= (mt_ft ? 12'd2 : HT_TOL));   // ★ 高：±3（或 ±2）
    wire       mt_wok = (mt_dw <= mt_wtol);                           // ★ 宽：±4（或 ±2）
    // ★ 判据顺序：红/R/C（硬） → 先宽高 W ±3 → 再行列差 Z ±3
    wire mt_match = (pb_red_d == mt_red) && (pb_npc_d == {3'd0, mt_R}) && (pb_npr_d == {3'd0, mt_C})
                 && mt_wok && mt_hok && mt_cok && mt_ook && mt_ohk && mt_zok;

    // ---- 帧末（投影扫描结束那一拍）启动 ----
    wire mt_go = pb_latch_d;                            // ★ 等宽高锁存完成后再启动查表
    // 编号 1..37 -> 十位/个位（不用除法：tens*10 = tens<<3 + tens<<1）
    wire [3:0] mt_ten = (mt_id < 6'd10) ? 4'd0 : (mt_id < 6'd20) ? 4'd1 :
                        (mt_id < 6'd30) ? 4'd2 : 4'd3;
    wire [3:0] mt_uni = mt_id[3:0] - (mt_ten << 3) - (mt_ten << 1);
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            mt_run <= 1'b0; mt_i <= 6'd0; mt_hits <= 7'd0; mt_id <= 6'd0; res_row <= 16'hFFFF;
        end else if (mt_go) begin
            mt_run <= 1'b1; mt_i <= 6'd0; mt_hits <= 7'd0; mt_id <= 6'd0;
        end else if (mt_run) begin
            if (mt_match) begin
                mt_hits <= mt_hits + 7'd1;
                if (mt_hits == 7'd0) mt_id <= mt_num;      // ★ 编号直接来自表项（白 32 不在表里，index+1 会错位）
            end
            if (mt_i == 6'd36) begin              // ★ 表有 34 条（索引 0..33），赤宝牌不区分，到 35 就收尾
                mt_run <= 1'b0;
                // ---- 出结论 ----
                if ((pb_npr_d == 7'd0) && (pb_npc_d == 7'd0))         // ★ 全零优先判"白" = 32（不被伪项干扰）
                    res_row <= 16'h0032;
                else if (mt_hits == 7'd1)                         // 唯一命中 -> 两位十进制编号
                    res_row <= {4'd0, 4'd0, mt_ten, mt_uni};
                else if (mt_hits > 7'd1)                          // 命中多个 -> 不可分
                    res_row <= 16'hDDDD;
                else                                              // 其它 -> 没识别出
                    res_row <= 16'hFFFF;
            end else begin
                mt_i <= mt_i + 6'd1;
            end
        end
    end

    // ★ 绿框长宽（OSD 显示）：宽 = x2-x1+1、高 = y2-y1+1
    wire [11:0] pb_w12 = (pb_x2 >= pb_x1) ? (pb_x2 - pb_x1 + 12'd1) : 12'd0;
    wire [11:0] pb_h12 = (pb_y2 >= pb_y1) ? (pb_y2 - pb_y1 + 12'd1) : 12'd0;
    wire [15:0] bcd_ww = bcd4(pb_w_d);           // ★ 锁存宽（与 matcher 同源）           // [7:4]=十位 [3:0]=个位
    wire [15:0] bcd_hh = bcd4(pb_h_d);           // ★ 锁存高（与 matcher 同源）
    wire [15:0] bcd_npc = bcd4({5'd0, pb_npc_d});        // 列峰数（竖直投影）2 位十进制
    wire [11:0] red_q  = {1'b0, pb_redc_d} >> RED_SHIFT; // ★ 量化值 q = 红像素数 / 8（0..644）
    wire [15:0] bcd_red = bcd4(red_q);                   // 3 位十进制（百/十/个）
    wire [15:0] red_row = {3'd0, pb_red, bcd_red[11:0]}; // 结论(1 位) + 计数三位十进制
    wire [15:0] bcd_npr = bcd4({5'd0, pb_npr_d});
    wire [15:0] bcd_hc  = bcd4({2'd0, h_col_pct});       // 列投影熵 %
    wire [15:0] bcd_hr  = bcd4({2'd0, h_row_pct});       // 行投影熵 %
    // ★ F 行 = 覆盖列数 / 覆盖行数（至少含 1 个暗像素的列/行个数）
    //   注意：Σcol 恒等于 Σrow（都是暗像素总数），所以不能拿两个总和当一对显示
    wire [15:0] bcd_we  = bcd4({9'd0, cov_c_d});         // 覆盖列数 0~56（迟滞后）
    wire [15:0] bcd_he  = bcd4({8'd0, cov_r_d});         // 覆盖行数 0~92（迟滞后）
    // ★ Z 行显示：max/min 各自饱和到 99（两位十进制能显示的最大值）
    // ★ Z 行 = 列投影差(max-min) / 行投影差(max-min)
    wire [7:0]  cdiff = (cmax_d >= cmin_d) ? (cmax_d - cmin_d) : 8'd0;
    wire [7:0]  rdiff = (rmax_d >= rmin_d) ? (rmax_d - rmin_d) : 8'd0;
    wire [7:0]  cdiff_sh = (cdiff > 8'd99) ? 8'd99 : cdiff;
    wire [7:0]  rdiff_sh = (rdiff > 8'd99) ? 8'd99 : rdiff;
    wire [15:0] bcd_cmx = bcd4({2'd0, cdiff_sh});        // 列差值
    wire [15:0] bcd_cmn = bcd4({2'd0, rdiff_sh});
    // ★ O 行：行差的真实值（7 位，比 Z 多 1 位；行投影最大只有 56 不会饱和）

    wire [15:0] bcd_rt0 = bcd4(cdiff_t);
    // ★ 两个框的结果显示：'D' + 框号(00/01) + 牌号(2 位)
    // ★ res_row 本身已经是 BCD（如 0x0011 = "0011"），直接送显示，不能再转
    //   未锁定(FFFF / 非 1..34)显示 FF
    wire [7:0] d0 = (res_b[0][7:0] <= 8'h37) ? res_b[0][7:0] : 8'hFF;
    wire [7:0] d1 = (res_b[1][7:0] <= 8'h37) ? res_b[1][7:0] : 8'hFF;
    wire [7:0] d2 = (res_b[2][7:0] <= 8'h37) ? res_b[2][7:0] : 8'hFF;
    wire [7:0] d3 = (res_b[3][7:0] <= 8'h37) ? res_b[3][7:0] : 8'hFF;
    wire [7:0] d4 = (res_b[4][7:0] <= 8'h37) ? res_b[4][7:0] : 8'hFF;
    wire [7:0] d5 = (res_b[5][7:0] <= 8'h37) ? res_b[5][7:0] : 8'hFF;
    wire [7:0] d6 = (res_b[6][7:0] <= 8'h37) ? res_b[6][7:0] : 8'hFF;
    wire [7:0] d7 = (res_b[7][7:0] <= 8'h37) ? res_b[7][7:0] : 8'hFF;
    wire [7:0] d8 = (res_b[8][7:0] <= 8'h37) ? res_b[8][7:0] : 8'hFF;
    wire [7:0] d9  = (res_b[9][7:0]  <= 8'h37) ? res_b[9][7:0]  : 8'hFF;
    wire [7:0] d10 = (res_b[10][7:0] <= 8'h37) ? res_b[10][7:0] : 8'hFF;
    wire [7:0] d11 = (res_b[11][7:0] <= 8'h37) ? res_b[11][7:0] : 8'hFF;
    wire [7:0] d12 = (res_b[12][7:0] <= 8'h37) ? res_b[12][7:0] : 8'hFF;
    wire [7:0] d13 = (res_b[13][7:0] <= 8'h37) ? res_b[13][7:0] : 8'hFF;
    reg         y_state;                                 // ★ 役状态（必须先声明，y_row 用到它）
    wire [15:0] y_row = {4'd0, y_state, 8'h00};
    wire [15:0] t_row = bcd4(bin_thr);                   // ★ C 行：当前 BIN_THR          // Y + 状态 + 役编号(暂 00)
    wire [15:0] bcd_dred  = bcd4({2'd0, d_red_d});
    wire [15:0] bcd_ddark = bcd4({2'd0, d_dark_d});
    always @(*) begin
        case (osd_rw)
            // 第 0 行：'W' + 宽(2 位十进制) + 高(2 位十进制)
            5'd0:    begin osd_ch = 8'h57; osd_val = {bcd_ww[7:0],  bcd_hh[7:0] };  end
            // 第 1 行：'P' + R(行峰数) + C(列峰数)，各 2 位十进制   ← 例 0103 = 行1 列3
            5'd1:    begin osd_ch = 8'h50; osd_val = {bcd_npc[7:0], bcd_npr[7:0]};  end
            // 第 2 行：'R' + 红色结论(1 = 有红 / 0 = 没有) + 红量(2 位十进制) + 0
            5'd2:    begin osd_ch = 8'h52; osd_val = red_row;                        end
            // 第 3 行：'N' + 识别编号（2 位十进制）/ DDDD 不可分 / FFFF 没识别出
            5'd3:    begin osd_ch = 8'h4E; osd_val = res_row;                        end
            // 第 4 行：'E' + 列投影熵% + 行投影熵%（各 2 位十进制）
            // 第 5 行：'F' + 有效宽度(列像素) + 有效高度(行像素)
            // 第 6 行：'Z' + 列投影最大值 + 列投影最小值
            5'd20:   begin osd_ch = 8'h59; osd_val = y_row; end
            5'd21:   begin osd_ch = 8'h43; osd_val = {4'd0, t_row[11:0]}; end
            5'd22:   begin
                if (I_fd_osd_en)      begin osd_ch = 8'h48; osd_val = bcd4({4'd0, objc_s}); end    // H = 物体个数
                else              begin osd_ch = 8'h20; osd_val = 16'hFFFF; end
            end   // C = BIN_THR   // Y 役状态
            5'd18:   begin osd_ch = 8'h5A; osd_val = {bcd_cmx[7:0], bcd_cmn[7:0]}; end
            // 第 7 行：'D' + 锁定标志 + 锁定值十位 + 锁定值个位 + 0   （后面跟连续计数见 E 行）
            5'd4:    begin osd_ch = 8'h44; osd_val = {4'd0, 4'd0, d0}; end   // D 00
            5'd5:    begin osd_ch = 8'h44; osd_val = {4'd0, 4'd1, d1}; end   // D 01
            5'd6:    begin osd_ch = 8'h44; osd_val = {4'd0, 4'd2, d2}; end   // D 02
            5'd7:    begin osd_ch = 8'h44; osd_val = {4'd0, 4'd3, d3}; end   // D 03
            5'd8:    begin osd_ch = 8'h44; osd_val = {4'd0, 4'd4, d4}; end   // D 04
            5'd9:    begin osd_ch = 8'h44; osd_val = {4'd0, 4'd5, d5}; end   // D 05
            5'd10:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd6, d6}; end   // D 06
            5'd11:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd7, d7}; end   // D 07
            5'd12:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd8, d8}; end   // D 08
            5'd13:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd9, d9}; end   // D 09
            5'd14:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd10, d10}; end   // D 10
            5'd15:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd11, d11}; end   // D 11
            5'd16:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd12, d12}; end   // D 12
            5'd17:   begin osd_ch = 8'h44; osd_val = {4'd0, 4'd13, d13}; end   // D 13
            
            // 第 6 行：'O' + 行差真实值（3 位十进制，0~127）
            default: begin osd_ch = 8'h4F; osd_val = {4'd0, bcd_rt0[11:8], bcd_rt0[7:0]}; end   // O 真实列差
        endcase
        if (osd_ci != 3'd0) begin                 // 第 1 个字符是字母，其余是数字
            osd_nib = osd_val[{4'd4 - osd_ci, 2'b00} +: 4];
            osd_ch  = (osd_nib > 4'd9) ? (8'h37 + osd_nib) : (8'h30 + osd_nib);
        end
    end

    wire [15:0] osd_bits;
    osd_char_lib u_osd_ch (
        .I_char    (osd_ch),
        .I_row     (osd_row),
        .O_row_bits(osd_bits)
    );

    // 列选位：O_row_bits 的 bit15 = 最左像素
    wire osd_pix = osd_bits[4'd15 - osd_col];
    // ★ OSD【已按用户要求关闭】：置 1 即恢复 7 行文本（F/M/N/C/A/P/G）
    //   关掉后 bcd4 那几串双 dabble 与字符 ROM 都成了死逻辑，会被综合剪掉
    localparam OSD_EN = 1'b1;   // ★ 打开：只显示一位识别结果（1 = 识别到 / 0 = 没识别到）
    wire osd_hit = OSD_EN && osd_area && osd_pix && (!I_fd_osd_en || (osd_rw == 5'd22));
    localparam [23:0] OSD_COLOR = 24'hfff200;    // 黄色

    // ================= 多目标运动方框（青色，1 像素边 × 最多 4 个）=================
    localparam [23:0] MBOX_COL = 24'h00ffff;     // 青色
    // ★ 4 个框并行判断，或起来
    function box_hit;
        input [11:0] x1, x2, y1, y2;
        input [11:0] cx, cy;
        begin
            box_hit = ((cy == y1) || (cy == y2)) && (cx >= x1) && (cx <= x2) ||
                      ((cx == x1) || (cx == x2)) && (cy >= y1) && (cy <= y2);
        end
    endfunction
    // ---- 帧差二值图查询（SW4 二值图已取消：这段只剩地址输出，用移位避免乘法器）----
    wire [11:0] q_bx = S_x_1d >> 3;
    wire [11:0] q_by = S_y_1d >> 3;
    assign O_fd_q_addr = ({7'd0, q_by[7:0]} << 3) + {10'd0, q_bx[7:5]};   // q_by*8（移位，不用 DSP）
    wire q_bit = I_fd_q_word[q_bx[4:0]];

    // ========================================================================
    //  ★ 跨时钟域同步：帧差模块跑在【摄像头域 S_csi_rx_clk】，
    //     而本模块在【显示像素域 S_hdmi_pixel_clk】。
    //     方框坐标直接用会被采到不一致的多位值 -> 屏上出现巨大/极小框。
    //     做法：用帧计数变化作为“新一帧结果就绪”的标志，在显示域锁存一份稳定副本。
    //     源端每帧只改一次并保持 ~16 ms，所以锁存时数据必定稳定。
    // ========================================================================
    reg [15:0] fd_cnt_d;
    reg [7:0]  mbv_s;
    reg [95:0] mbx1_s, mbx2_s, mby1_s, mby2_s;
    reg [7:0]  objc_s;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            fd_cnt_d <= 16'd0;
            mbv_s <= 8'd0; mbx1_s <= 96'd0; mbx2_s <= 96'd0;
            mby1_s <= 96'd0; mby2_s <= 96'd0; objc_s <= 8'd0;
        end else begin
            fd_cnt_d <= I_fd_frame_cnt;
            if (I_fd_frame_cnt != fd_cnt_d) begin      // 帧差更新了新一帧结果
                mbv_s  <= I_mb_v;
                mbx1_s <= I_mb_x1;  mbx2_s <= I_mb_x2;
                mby1_s <= I_mb_y1;  mby2_s <= I_mb_y2;
                objc_s <= I_disp_h;
            end
        end
    end
    // 以下方框判断全部用锁存后的副本
    wire mbox_hit =                     (mbv_s[0] && box_hit(mbx1_s[0*12 +: 12], mbx2_s[0*12 +: 12],
                                          mby1_s[0*12 +: 12], mby2_s[0*12 +: 12], S_x_2d, S_y_2d)) ||
                    (mbv_s[1] && box_hit(mbx1_s[1*12 +: 12], mbx2_s[1*12 +: 12],
                                          mby1_s[1*12 +: 12], mby2_s[1*12 +: 12], S_x_2d, S_y_2d)) ||
                    (mbv_s[2] && box_hit(mbx1_s[2*12 +: 12], mbx2_s[2*12 +: 12],
                                          mby1_s[2*12 +: 12], mby2_s[2*12 +: 12], S_x_2d, S_y_2d)) ||
                    (mbv_s[3] && box_hit(mbx1_s[3*12 +: 12], mbx2_s[3*12 +: 12],
                                          mby1_s[3*12 +: 12], mby2_s[3*12 +: 12], S_x_2d, S_y_2d)) ||
                    (mbv_s[4] && box_hit(mbx1_s[4*12 +: 12], mbx2_s[4*12 +: 12],
                                          mby1_s[4*12 +: 12], mby2_s[4*12 +: 12], S_x_2d, S_y_2d)) ||
                    (mbv_s[5] && box_hit(mbx1_s[5*12 +: 12], mbx2_s[5*12 +: 12],
                                          mby1_s[5*12 +: 12], mby2_s[5*12 +: 12], S_x_2d, S_y_2d)) ||
                    (mbv_s[6] && box_hit(mbx1_s[6*12 +: 12], mbx2_s[6*12 +: 12],
                                          mby1_s[6*12 +: 12], mby2_s[6*12 +: 12], S_x_2d, S_y_2d)) ||
                    (mbv_s[7] && box_hit(mbx1_s[7*12 +: 12], mbx2_s[7*12 +: 12],
                                          mby1_s[7*12 +: 12], mby2_s[7*12 +: 12], S_x_2d, S_y_2d));

    // ★ 诊断用固定测试框：x=100..399, y=100..199（白色 1 像素边）
    //   只要 SW4 打开就一定画出来 —— 用来判断"画框路径"本身通不通
    localparam [11:0] TBOX_X1 = 12'd100, TBOX_X2 = 12'd399;
    localparam [11:0] TBOX_Y1 = 12'd100, TBOX_Y2 = 12'd199;
    wire tbox_top = (S_y_2d == TBOX_Y1) || (S_y_2d == TBOX_Y2);
    wire tbox_v   = (S_y_2d >= TBOX_Y1) && (S_y_2d <= TBOX_Y2);
    wire tbox_h   = (S_x_2d >= TBOX_X1) && (S_x_2d <= TBOX_X2);
    wire tbox_hit = I_tbox_en &&
                    ((tbox_top && tbox_h) ||
                     (tbox_v && ((S_x_2d == TBOX_X1) || (S_x_2d == TBOX_X2))));

    // ★ 麻将识别：切分后【选中】那张牌的框（黄色 1 像素边）
    //   青色框 = frame_diff 的合并框（整排）；黄框 = 切分出来的单张
    //   ★ 识别模块已解除例化，I_tr_v 恒 0 -> 这个框自然不画（也顺手加了 I_tr_v 门控：
    //     否则坐标全 0 时会在屏幕原点留一个像素点）
    localparam [23:0] SBOX_COL = 24'hffff00;
    wire sbox_hit = I_tr_v && box_hit(I_tr_x1, I_tr_x2, I_tr_y1, I_tr_y2, S_x_2d, S_y_2d);

    // ★ 牌位分割线（用户方案第 3 条）：在 14 个牌位之间画【固定红色竖线】
    //   主框 13 个牌位：x = 56 + 76k（k = 0..13，共 14 条线：左边界 + 12 条间隔 + 右边界）
    //   右侧第 14 个牌位（小框）：x = 1096 和 1168 两条线
    //   线只画在牌位高度范围内 y = 240..351；颜色纯红、1 像素宽
    //   ★ 线的位置在下面 div_main / div_sub 里是【写死的常量】（x = 56 + 76k），
    //     改间距就直接改那些常量（DIV_X0 / DIV_PITCH 只是留个公式备忘）
    localparam [11:0] DIV_X0    = 12'd56;      // 公式备忘：第 1 条线 = DIV_X0
    localparam [11:0] DIV_PITCH = 12'd76;      //            相邻两条线间距 = DIV_PITCH
    localparam [11:0] DIV_Y1    = 12'd240;     // 线的上下范围
    localparam [11:0] DIV_Y2    = 12'd351;
    localparam [23:0] DIV_COL   = 24'hff0000;  // 纯红

    wire div_y = (S_y_2d >= DIV_Y1) && (S_y_2d <= DIV_Y2);
    // 主框内 14 条线：★ 第 8 条起（含小框两条）全部右移 15 像素（用户标定结果）
    //   1..7  : 56,132,208,284,360,436,512
    //   8..14 : 588+15,664+15,740+15,816+15,892+15,968+15,1044+15
    //   15..16: 1096+15, 1168+15
    // ★ 从框6 的右边界(603)开始，右侧所有红线统一左移 10 像素
    wire div_main = (S_x_2d == 12'd56)   || (S_x_2d == 12'd132)  || (S_x_2d == 12'd208)  ||
                    (S_x_2d == 12'd284)  || (S_x_2d == 12'd360)  || (S_x_2d == 12'd436)  ||
                                        (S_x_2d == 12'd512)  || (S_x_2d == 12'd593)  || (S_x_2d == 12'd674)  ||   // ★ 框7 右边界起右移 5
                                                            (S_x_2d == 12'd750)  || (S_x_2d == 12'd831)  || (S_x_2d == 12'd907)  ||   // ★ 框9右边界起 +5
                                                            (S_x_2d == 12'd983)  || (S_x_2d == 12'd1059);
    // 右侧第 14 个牌位（小框）的两条线（同样 +15）
    wire div_sub  = (S_x_2d == 12'd1111) || (S_x_2d == 12'd1183);   // ★ 再右移 5
    wire div_hit  = div_y && (div_main || div_sub);

    // ★ 13 张牌的红框（用户要求恢复显示）：主框 x 56..1062, y 240..351
    //   ★ 右边界跟着红线一起拓宽 15 像素（1047 -> 1062）
    localparam [11:0] TBX_X1 = 12'd56,  TBX_X2 = 12'd1062;   // ★ 右边界：-10 再 +5
    localparam [11:0] TBX_Y1 = 12'd240, TBX_Y2 = 12'd351;
    wire tbox13_hit = box_hit(TBX_X1, TBX_X2, TBX_Y1, TBX_Y2, S_x_2d, S_y_2d);

    // ★ SW3 二值化（用户方案第 2 条）：把当前画面按亮度阈值变成黑白
    //   亮度近似 = (R + 2G + B) / 4 —— 只用加法/移位，零乘法（DSP 只剩 7 个）
    //   极性：暗（图案/字）-> 白，亮（牌面/桌面）-> 黑，和识别时代"前景"一致
    //   ★ 阈值 BIN_THR 现场调：太小 -> 全是白；太大 -> 全是黑
    localparam [7:0] BIN_THR = 8'd150;                    // 默认值（KEY2 可调）
    reg  [7:0]  bin_thr;                                  // ★ KEY2 可调的亮度阈值
    // ★ KEY3（复用 I_tr_freeze 端口 = 原 SW2，wrapper 改接 ~I_button[2]）：复位绿框 + 回到框0
    reg         k3_d, k3_dd, k3_req;
    wire        k3_press = k3_d && !k3_dd;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin k3_d <= 1'b0; k3_dd <= 1'b0; k3_req <= 1'b0; end
        else begin
            k3_d  <= I_tr_freeze;      // KEY3（wrapper 改接 ~I_button[2]）
            k3_dd <= k3_d;
            if (k3_press)                k3_req <= 1'b1;
            else if (k3_req && pb_latch) k3_req <= 1'b0;
        end
    end
    reg         k2_d, k2_dd, k2_req;
    wire        k2_press = k2_d && !k2_dd;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            k2_d <= 1'b0; k2_dd <= 1'b0; k2_req <= 1'b0; bin_thr <= BIN_THR;
        end else begin
            k2_d  <= I_tr_skipinv;
            k2_dd <= k2_d;
            if (k2_press)                k2_req <= 1'b1;
            else if (k2_req && pb_latch) k2_req <= 1'b0;
            if (k2_req && pb_latch) begin
                if (bin_thr >= 8'd200) bin_thr <= 8'd100;
                else                   bin_thr <= bin_thr + 8'd5;
            end
        end
    end
    // 亮度 = (R + 2G + B) / 4（加权近似，KEY2 可调阈值）
wire [9:0]  bin_sum = {2'b00, S_video_data_2d[23:16]}                 // R
                    + {1'b0,  S_video_data_2d[15:8], 1'b0}            // 2G
                    + {2'b00, S_video_data_2d[7:0]};                  // B
wire [7:0]  bin_y   = bin_sum[9:2];                                   // /4
wire       bin_dark = (bin_y < bin_thr);

    // ========================================================================
    // ========================================================================
    //  ★ 模板识别【已按用户要求删除】（16x22 采样矩阵 + 汉明距离比对 + 结果位）
    //    保留：SW3 二值化、投影求包围盒 + 绿色字符框、OSD 显示框的长宽
    // ========================================================================

    //  ★ 框 0 的水平/竖直投影（只投【框内】的二值暗像素）—— 用来把字符/图案框住
    //    · 竖直投影（按列）：每列数框内暗像素 -> 字符的 x1 / x2
    //    · 水平投影（按行）：每行数框内暗像素 -> 字符的 y1 / y2
    //    · 计数器只 2 位（0/1/>=2）：>=2 个暗像素算"这列/行有东西"，抗单点噪点，
    //      而且比 7 位计数器省 3/4 寄存器
    //    · 框末（框的下一行行首）串行扫 76+112 = 188 拍求 min/max（消隐 9600 拍，够）
    //    ★ 以后接其它框：把 PBX1..PBY2 换成"当前框"坐标，包围盒每框一份即可
    // ========================================================================
    localparam [11:0] PBX1 = 12'd66,  PBX2 = 12'd121;    // ★ 框 0 四边内缩 10 px
    localparam [11:0] PBY1 = 12'd250, PBY2 = 12'd341;    //    框范围 56..131 / 240..351
    // ★★ 双框轮询：每帧切换 box_sel，用同一套投影引擎先看框 0 再看框 1
    //    框0 红框 56..131  内缩 10 -> 66..121
    //    框1 红框 132..207 内缩 10 -> 142..197
    reg  [3:0] box_sel;                                  // 0..9                                  // 0..5 = 框0..框5
    // 框 k 的窗口：红框 x1 = 56 + 76k，内缩 10 -> 窗口 x1 = 66 + 76k（宽 56）
    reg  [11:0] PBX1S, PBX2S;
    always @(*) begin
        case (box_sel)
            4'd0: begin PBX1S = 12'd66;  PBX2S = 12'd121; end
            4'd1: begin PBX1S = 12'd142; PBX2S = 12'd197; end
            4'd2: begin PBX1S = 12'd218; PBX2S = 12'd273; end
            4'd3: begin PBX1S = 12'd294; PBX2S = 12'd349; end
            4'd4: begin PBX1S = 12'd370; PBX2S = 12'd425; end
            4'd5: begin PBX1S = 12'd446; PBX2S = 12'd501; end
            4'd6: begin PBX1S = 12'd522; PBX2S = 12'd577; end
            4'd7: begin PBX1S = 12'd608; PBX2S = 12'd663; end   // ★ 框7 右移 5（原 603..658）
            4'd8: begin PBX1S = 12'd684; PBX2S = 12'd739; end
            4'd9: begin PBX1S = 12'd765; PBX2S = 12'd820; end   // ★ +5
            4'd10:begin PBX1S = 12'd841; PBX2S = 12'd896; end   // ★ +5
            4'd11:begin PBX1S = 12'd917; PBX2S = 12'd972; end   // ★ +5
            4'd12:begin PBX1S = 12'd993; PBX2S = 12'd1048; end   // 框12
            default: begin PBX1S = 12'd1121; PBX2S = 12'd1176; end // ★ 框13（小框 1111..1183）
        endcase
    end
    wire [11:0] PBY1S = PBY1;
    wire [11:0] PBY2S = PBY2;
    reg [15:0] res_b [0:13];                              // ★ 6 个框各自锁存的识别结果
    integer rk;
    reg [11:0] bx1_b [0:13], bx2_b [0:13], by1_b [0:13], by2_b [0:13];   // 各框包围盒
    localparam [7:0]  PBW  = 8'd55;                      // 列范围 0..55（56 列）
    reg        bok_b [0:13];                          // 该框是否找到图案
    localparam [7:0]  PBH  = 8'd91;                      // 行范围 0..91（92 行）
    localparam [1:0]  PB_THR = 2'd2;                     // （旧门限，保留不用）
    // ★ 峰/谷判定（用户方案）：谷->峰 = 左/上边界，峰->谷 = 右/下边界
    //   计数改成 6 位（0..63，饱和 63），才分得出峰和谷
    localparam [5:0]  PB_PEAK = 6'd14;                    // 峰门限：>=8 个暗像素
    localparam [5:0]  PB_VAL  = 6'd5;                    // 谷门限：<=3

    wire pb_in  = (S_x_2d >= PBX1S) && (S_x_2d <= PBX2S) && (S_y_2d >= PBY1S) && (S_y_2d <= PBY2S);
    wire pb_ink = bin_dark;                              // ★ 只投暗像素（二值）
    wire [11:0] pb_xr = S_x_2d - PBX1S;                   // 框内列 0..75
    wire [11:0] pb_yr = S_y_2d - PBY1S;                   // 框内行 0..111
    reg  [5:0]  pb_col [0:75];                           // 竖直投影：每列暗像素数（饱和 63）
    reg  [6:0]  pc7    [0:75];                          // ★ 7 位列投影（不饱和，用于 O 行真实列差）
    reg  [7:0]  cmax7, cmin7, cdiff_t;
    reg  [5:0]  pb_row [0:111];                          // 水平投影：每行暗像素数（饱和 63）
    // ========================================================================
    //  ★ 投影熵（旁挂，不碰上面任何原有逻辑）：只读 pb_col/pb_row 做分桶累加
    //    H = log2(S) - (Σ c·log2c)/S ;  Σ c·log2c ≈ 3n2+14n3+41n4+107n5+265n6+628n7
    //    （6 位饱和 63 的那一档全部落进"≥32"桶，误差约 1~3%，做特征足够）
    // ========================================================================
    reg  [6:0]  en2, en3, en4, en5, en6;                 // 列桶计数（n2..n6；≥32 全归 n6）
    reg  [6:0]  er2, er3, er4, er5, er6;                 // 行桶计数
    reg  [13:0] esum_c, esum_r;                          // S = Σc
    reg  [4:0]  ediv_step;
    reg  [3:0]  et_c, et_r;                              // t = Σ/S （0..7）
    reg  [15:0] ecc_c, ecc_r;                            // 除法被减数
    reg  [6:0]  cov_c, cov_r;
    // ★ D 行（只显示，不动原红判据）：绿框内 红像素数 / 暗像素数
    reg  [12:0] d_red, d_dark;
    reg  [12:0] d_red_l, d_dark_l;                       // ★ 帧末锁存（清零前取样给显示）
    wire [12:0] d_red_sh  = (d_red_l  > 13'd1584) ? 13'd1584 : d_red_l;
    wire [12:0] d_dark_sh = (d_dark_l > 13'd6336) ? 13'd6336 : d_dark_l;
    wire [7:0]  d_red_d   = d_red_sh[11:4];              // /16 -> 0..99
    wire [7:0]  d_dark_d  = d_dark_sh[11:6];             // /64 -> 0..99
    // ========================================================================
    //  ★ N 锁定逻辑
    //    · 连续 6 帧 res_row 相同 -> 锁定该值
    //    · 当 W=5690 且 P=0101（无牌状态）连续 30 帧 -> 解锁复位
    //    D 行显示：'D' + 锁定标志 + 锁定值(2 位) + 连续计数(1 位)
    // ========================================================================
    reg  [15:0] res_prev;
    reg  [2:0]  lock_cnt;                                // 连续相同帧数 0..7（>=6 锁定）
    reg  [4:0]  empty_cnt;                               // 空态连续帧数 0..31（>=30 复位）
    reg  [15:0] lock_val;
    reg         lock_en;
    // ========================================================================
    //  ★ 役判定 第 1 步（TD 兼容写法：无存储器/无循环/无变量下标）
    //     牌号 1..34 -> c1..c34（3 位计数器）
    // ========================================================================
    reg  [2:0]  c1;
    reg  [2:0]  c2;
    reg  [2:0]  c3;
    reg  [2:0]  c4;
    reg  [2:0]  c5;
    reg  [2:0]  c6;
    reg  [2:0]  c7;
    reg  [2:0]  c8;
    reg  [2:0]  c9;
    reg  [2:0]  c10;
    reg  [2:0]  c11;
    reg  [2:0]  c12;
    reg  [2:0]  c13;
    reg  [2:0]  c14;
    reg  [2:0]  c15;
    reg  [2:0]  c16;
    reg  [2:0]  c17;
    reg  [2:0]  c18;
    reg  [2:0]  c19;
    reg  [2:0]  c20;
    reg  [2:0]  c21;
    reg  [2:0]  c22;
    reg  [2:0]  c23;
    reg  [2:0]  c24;
    reg  [2:0]  c25;
    reg  [2:0]  c26;
    reg  [2:0]  c27;
    reg  [2:0]  c28;
    reg  [2:0]  c29;
    reg  [2:0]  c30;
    reg  [2:0]  c31;
    reg  [2:0]  c32;
    reg  [2:0]  c33;
    reg  [2:0]  c34;
    reg         lock_en_d;
    reg  [3:0]  box_sel_y;                                          // box_sel 的上一帧值
    wire        round_start = (box_sel == 4'd0) && (box_sel_y != 4'd0);  // ★ 新一轮开始
    // ★ 评估脉冲：框13 锁定后打一拍（此时牌计数已含框13 那张）
    reg         eval_pend;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)                        eval_pend <= 1'b0;
        else if (add_en && (box_sel == 4'd13)) eval_pend <= 1'b1;   // 框13 锁定
        else                                 eval_pend <= 1'b0;
    end
    // ======================= 和牌引擎：4 面子 + 1 将 =======================
    //  输入 c1..c34（各牌张数）  输出 yform（=1 能和）
    //  每轮尝试：复制 -> 减将 -> 拆分(34 步) -> 结论；失败换"顺子优先"，再失败换将牌
    reg [2:0] wk1,wk2,wk3,wk4,wk5,wk6,wk7,wk8,wk9,wk10,wk11,wk12,wk13,wk14,wk15,wk16,wk17;
    reg [2:0] wk18,wk19,wk20,wk21,wk22,wk23,wk24,wk25,wk26,wk27,wk28,wk29,wk30,wk31,wk32,wk33,wk34;
    reg [5:0] epr;      // 将牌 1..34
    reg [5:0] eix;      // 段内步 1..9（字牌 1..7）
    reg [1:0] esg;      // 0 万 1 筒 2 条 3 字
    reg [2:0] eph;      // 0 idle 1 copy 2 subpair 3 split 4 conclude
    reg       eps;      // 0 刻子优先 1 顺子优先
    reg       ebd;      // 本轮失败
    reg       ewn;      // 判定能和
    reg       est;      // 启动

    wire [2:0] wpick =
          (epr==6'd1)?wk1 :(epr==6'd2)?wk2 :(epr==6'd3)?wk3 :(epr==6'd4)?wk4 :(epr==6'd5)?wk5
        : (epr==6'd6)?wk6 :(epr==6'd7)?wk7 :(epr==6'd8)?wk8 :(epr==6'd9)?wk9 :(epr==6'd10)?wk10
        : (epr==6'd11)?wk11:(epr==6'd12)?wk12:(epr==6'd13)?wk13:(epr==6'd14)?wk14:(epr==6'd15)?wk15
        : (epr==6'd16)?wk16:(epr==6'd17)?wk17:(epr==6'd18)?wk18:(epr==6'd19)?wk19:(epr==6'd20)?wk20
        : (epr==6'd21)?wk21:(epr==6'd22)?wk22:(epr==6'd23)?wk23:(epr==6'd24)?wk24:(epr==6'd25)?wk25
        : (epr==6'd26)?wk26:(epr==6'd27)?wk27:(epr==6'd28)?wk28:(epr==6'd29)?wk29:(epr==6'd30)?wk30
        : (epr==6'd31)?wk31:(epr==6'd32)?wk32:(epr==6'd33)?wk33:wk34;

    wire [5:0] t0 = (esg==2'd0) ? eix : (esg==2'd1) ? (6'd9 + eix) : (esg==2'd2) ? (6'd18 + eix) : (6'd27 + eix);
    wire [5:0] t1 = t0 + 6'd1;
    wire [5:0] t2 = t0 + 6'd2;
    wire [2:0] va =
          (t0==6'd1)?wk1 :(t0==6'd2)?wk2 :(t0==6'd3)?wk3 :(t0==6'd4)?wk4 :(t0==6'd5)?wk5
        : (t0==6'd6)?wk6 :(t0==6'd7)?wk7 :(t0==6'd8)?wk8 :(t0==6'd9)?wk9 :(t0==6'd10)?wk10
        : (t0==6'd11)?wk11:(t0==6'd12)?wk12:(t0==6'd13)?wk13:(t0==6'd14)?wk14:(t0==6'd15)?wk15
        : (t0==6'd16)?wk16:(t0==6'd17)?wk17:(t0==6'd18)?wk18:(t0==6'd19)?wk19:(t0==6'd20)?wk20
        : (t0==6'd21)?wk21:(t0==6'd22)?wk22:(t0==6'd23)?wk23:(t0==6'd24)?wk24:(t0==6'd25)?wk25
        : (t0==6'd26)?wk26:(t0==6'd27)?wk27:(t0==6'd28)?wk28:(t0==6'd29)?wk29:(t0==6'd30)?wk30
        : (t0==6'd31)?wk31:(t0==6'd32)?wk32:(t0==6'd33)?wk33:wk34;
    wire [2:0] vb =
          (t1==6'd1)?wk1 :(t1==6'd2)?wk2 :(t1==6'd3)?wk3 :(t1==6'd4)?wk4 :(t1==6'd5)?wk5
        : (t1==6'd6)?wk6 :(t1==6'd7)?wk7 :(t1==6'd8)?wk8 :(t1==6'd9)?wk9 :(t1==6'd10)?wk10
        : (t1==6'd11)?wk11:(t1==6'd12)?wk12:(t1==6'd13)?wk13:(t1==6'd14)?wk14:(t1==6'd15)?wk15
        : (t1==6'd16)?wk16:(t1==6'd17)?wk17:(t1==6'd18)?wk18:(t1==6'd19)?wk19:(t1==6'd20)?wk20
        : (t1==6'd21)?wk21:(t1==6'd22)?wk22:(t1==6'd23)?wk23:(t1==6'd24)?wk24:(t1==6'd25)?wk25
        : (t1==6'd26)?wk26:(t1==6'd27)?wk27:(t1==6'd28)?wk28:(t1==6'd29)?wk29:(t1==6'd30)?wk30
        : (t1==6'd31)?wk31:(t1==6'd32)?wk32:(t1==6'd33)?wk33:wk34;
    wire [2:0] vc =
          (t2==6'd1)?wk1 :(t2==6'd2)?wk2 :(t2==6'd3)?wk3 :(t2==6'd4)?wk4 :(t2==6'd5)?wk5
        : (t2==6'd6)?wk6 :(t2==6'd7)?wk7 :(t2==6'd8)?wk8 :(t2==6'd9)?wk9 :(t2==6'd10)?wk10
        : (t2==6'd11)?wk11:(t2==6'd12)?wk12:(t2==6'd13)?wk13:(t2==6'd14)?wk14:(t2==6'd15)?wk15
        : (t2==6'd16)?wk16:(t2==6'd17)?wk17:(t2==6'd18)?wk18:(t2==6'd19)?wk19:(t2==6'd20)?wk20
        : (t2==6'd21)?wk21:(t2==6'd22)?wk22:(t2==6'd23)?wk23:(t2==6'd24)?wk24:(t2==6'd25)?wk25
        : (t2==6'd26)?wk26:(t2==6'd27)?wk27:(t2==6'd28)?wk28:(t2==6'd29)?wk29:(t2==6'd30)?wk30
        : (t2==6'd31)?wk31:(t2==6'd32)?wk32:(t2==6'd33)?wk33:wk34;

    reg [5:0] ez;                                  // 剩余总数
    always @(*) begin
        ez = wk1+wk2+wk3+wk4+wk5+wk6+wk7+wk8+wk9+wk10+wk11+wk12+wk13+wk14+wk15+wk16+wk17
           + wk18+wk19+wk20+wk21+wk22+wk23+wk24+wk25+wk26+wk27+wk28+wk29+wk30+wk31+wk32+wk33+wk34;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            eph<=3'd0; epr<=6'd1; eix<=6'd1; esg<=2'd0; eps<=1'b0; ebd<=1'b0; ewn<=1'b0;
            wk1<=0;wk2<=0;wk3<=0;wk4<=0;wk5<=0;wk6<=0;wk7<=0;wk8<=0;wk9<=0;wk10<=0;
            wk11<=0;wk12<=0;wk13<=0;wk14<=0;wk15<=0;wk16<=0;wk17<=0;wk18<=0;wk19<=0;wk20<=0;
            wk21<=0;wk22<=0;wk23<=0;wk24<=0;wk25<=0;wk26<=0;wk27<=0;wk28<=0;wk29<=0;wk30<=0;
            wk31<=0;wk32<=0;wk33<=0;wk34<=0;
        end else case (eph)
        // --- 0 等待启动 ---
        3'd0: if (est) begin
            wk1<=c1;wk2<=c2;wk3<=c3;wk4<=c4;wk5<=c5;wk6<=c6;wk7<=c7;wk8<=c8;wk9<=c9;wk10<=c10;
            wk11<=c11;wk12<=c12;wk13<=c13;wk14<=c14;wk15<=c15;wk16<=c16;wk17<=c17;wk18<=c18;
            wk19<=c19;wk20<=c20;wk21<=c21;wk22<=c22;wk23<=c23;wk24<=c24;wk25<=c25;wk26<=c26;
            wk27<=c27;wk28<=c28;wk29<=c29;wk30<=c30;wk31<=c31;wk32<=c32;wk33<=c33;wk34<=c34;
            epr<=6'd1; eps<=1'b0; ewn<=1'b0; eph<=3'd2;
        end
        // --- 1 重新拷贝（换将牌/换顺序时重试）---
        3'd1: begin
            wk1<=c1;wk2<=c2;wk3<=c3;wk4<=c4;wk5<=c5;wk6<=c6;wk7<=c7;wk8<=c8;wk9<=c9;wk10<=c10;
            wk11<=c11;wk12<=c12;wk13<=c13;wk14<=c14;wk15<=c15;wk16<=c16;wk17<=c17;wk18<=c18;
            wk19<=c19;wk20<=c20;wk21<=c21;wk22<=c22;wk23<=c23;wk24<=c24;wk25<=c25;wk26<=c26;
            wk27<=c27;wk28<=c28;wk29<=c29;wk30<=c30;wk31<=c31;wk32<=c32;wk33<=c33;wk34<=c34;
            eph <= 3'd2;
        end
        // --- 2 减将牌 ---
        3'd2: begin
            if (wpick >= 3'd2) begin
                case (epr)
                6'd1:wk1<=wk1-3'd2;  6'd2:wk2<=wk2-3'd2;  6'd3:wk3<=wk3-3'd2;  6'd4:wk4<=wk4-3'd2;
                6'd5:wk5<=wk5-3'd2;  6'd6:wk6<=wk6-3'd2;  6'd7:wk7<=wk7-3'd2;  6'd8:wk8<=wk8-3'd2;
                6'd9:wk9<=wk9-3'd2;  6'd10:wk10<=wk10-3'd2;6'd11:wk11<=wk11-3'd2;6'd12:wk12<=wk12-3'd2;
                6'd13:wk13<=wk13-3'd2;6'd14:wk14<=wk14-3'd2;6'd15:wk15<=wk15-3'd2;6'd16:wk16<=wk16-3'd2;
                6'd17:wk17<=wk17-3'd2;6'd18:wk18<=wk18-3'd2;6'd19:wk19<=wk19-3'd2;6'd20:wk20<=wk20-3'd2;
                6'd21:wk21<=wk21-3'd2;6'd22:wk22<=wk22-3'd2;6'd23:wk23<=wk23-3'd2;6'd24:wk24<=wk24-3'd2;
                6'd25:wk25<=wk25-3'd2;6'd26:wk26<=wk26-3'd2;6'd27:wk27<=wk27-3'd2;6'd28:wk28<=wk28-3'd2;
                6'd29:wk29<=wk29-3'd2;6'd30:wk30<=wk30-3'd2;6'd31:wk31<=wk31-3'd2;6'd32:wk32<=wk32-3'd2;
                6'd33:wk33<=wk33-3'd2;default:wk34<=wk34-3'd2;
                endcase
                esg<=2'd0; eix<=6'd1; ebd<=1'b0; eph<=3'd3;
            end else ebd <= 1'b1;
            if (wpick < 3'd2) eph <= 3'd4;
        end
        // --- 3 拆分（每拍一步：万1-9、筒1-9、条1-9、字1-7）---
        3'd3: begin
            if (ebd) eph <= 3'd4;
            else if ((esg == 2'd3) && (eix > 6'd7)) eph <= 3'd4;
            else if (eix > 6'd9) begin esg <= esg + 2'd1; eix <= 6'd1; end
            else if (esg == 2'd3) begin
                if (va >= 3'd3) begin
                    case (t0)
                    6'd28:wk28<=wk28-3'd3;6'd29:wk29<=wk29-3'd3;6'd30:wk30<=wk30-3'd3;
                    6'd31:wk31<=wk31-3'd3;6'd32:wk32<=wk32-3'd3;6'd33:wk33<=wk33-3'd3;
                    default:wk34<=wk34-3'd3;
                    endcase
                    eix <= eix + 6'd1;
                end else if (va != 3'd0) ebd <= 1'b1;
                else eix <= eix + 6'd1;
            end else if ((eix >= 6'd8) || (eps == 1'b0)) begin
                if (va >= 3'd3) begin
                    case (t0)
                    6'd1:wk1<=wk1-3'd3;6'd2:wk2<=wk2-3'd3;6'd3:wk3<=wk3-3'd3;6'd4:wk4<=wk4-3'd3;
                    6'd5:wk5<=wk5-3'd3;6'd6:wk6<=wk6-3'd3;6'd7:wk7<=wk7-3'd3;6'd8:wk8<=wk8-3'd3;
                    6'd9:wk9<=wk9-3'd3;6'd10:wk10<=wk10-3'd3;6'd11:wk11<=wk11-3'd3;6'd12:wk12<=wk12-3'd3;
                    6'd13:wk13<=wk13-3'd3;6'd14:wk14<=wk14-3'd3;6'd15:wk15<=wk15-3'd3;6'd16:wk16<=wk16-3'd3;
                    6'd17:wk17<=wk17-3'd3;6'd18:wk18<=wk18-3'd3;6'd19:wk19<=wk19-3'd3;6'd20:wk20<=wk20-3'd3;
                    6'd21:wk21<=wk21-3'd3;6'd22:wk22<=wk22-3'd3;6'd23:wk23<=wk23-3'd3;6'd24:wk24<=wk24-3'd3;
                    6'd25:wk25<=wk25-3'd3;6'd26:wk26<=wk26-3'd3;6'd27:wk27<=wk27-3'd3;
                    default:wk27<=wk27-3'd3;
                    endcase
                    eix <= eix + 6'd1;
                end else if (va != 3'd0) begin
                    if ((eix < 6'd8) && (vb >= 3'd1) && (vc >= 3'd1)) begin
                        case (t0)
                        6'd1:begin wk1<=wk1-3'd1;wk2<=wk2-3'd1;wk3<=wk3-3'd1; end
                        6'd2:begin wk2<=wk2-3'd1;wk3<=wk3-3'd1;wk4<=wk4-3'd1; end
                        6'd3:begin wk3<=wk3-3'd1;wk4<=wk4-3'd1;wk5<=wk5-3'd1; end
                        6'd4:begin wk4<=wk4-3'd1;wk5<=wk5-3'd1;wk6<=wk6-3'd1; end
                        6'd5:begin wk5<=wk5-3'd1;wk6<=wk6-3'd1;wk7<=wk7-3'd1; end
                        6'd6:begin wk6<=wk6-3'd1;wk7<=wk7-3'd1;wk8<=wk8-3'd1; end
                        6'd7:begin wk7<=wk7-3'd1;wk8<=wk8-3'd1;wk9<=wk9-3'd1; end
                        6'd10:begin wk10<=wk10-3'd1;wk11<=wk11-3'd1;wk12<=wk12-3'd1; end
                        6'd11:begin wk11<=wk11-3'd1;wk12<=wk12-3'd1;wk13<=wk13-3'd1; end
                        6'd12:begin wk12<=wk12-3'd1;wk13<=wk13-3'd1;wk14<=wk14-3'd1; end
                        6'd13:begin wk13<=wk13-3'd1;wk14<=wk14-3'd1;wk15<=wk15-3'd1; end
                        6'd14:begin wk14<=wk14-3'd1;wk15<=wk15-3'd1;wk16<=wk16-3'd1; end
                        6'd15:begin wk15<=wk15-3'd1;wk16<=wk16-3'd1;wk17<=wk17-3'd1; end
                        6'd16:begin wk16<=wk16-3'd1;wk17<=wk17-3'd1;wk18<=wk18-3'd1; end
                        6'd19:begin wk19<=wk19-3'd1;wk20<=wk20-3'd1;wk21<=wk21-3'd1; end
                        6'd20:begin wk20<=wk20-3'd1;wk21<=wk21-3'd1;wk22<=wk22-3'd1; end
                        6'd21:begin wk21<=wk21-3'd1;wk22<=wk22-3'd1;wk23<=wk23-3'd1; end
                        6'd22:begin wk22<=wk22-3'd1;wk23<=wk23-3'd1;wk24<=wk24-3'd1; end
                        6'd23:begin wk23<=wk23-3'd1;wk24<=wk24-3'd1;wk25<=wk25-3'd1; end
                        6'd24:begin wk24<=wk24-3'd1;wk25<=wk25-3'd1;wk26<=wk26-3'd1; end
                        6'd25:begin wk25<=wk25-3'd1;wk26<=wk26-3'd1;wk27<=wk27-3'd1; end
                        default:begin wk27<=wk27-3'd1; end
                        endcase
                        eix <= eix + 6'd1;
                    end else ebd <= 1'b1;
                end else eix <= eix + 6'd1;
            end else begin
                if ((eix < 6'd8) && (va >= 3'd1) && (vb >= 3'd1) && (vc >= 3'd1)) begin
                    case (t0)
                    6'd1:begin wk1<=wk1-3'd1;wk2<=wk2-3'd1;wk3<=wk3-3'd1; end
                    6'd2:begin wk2<=wk2-3'd1;wk3<=wk3-3'd1;wk4<=wk4-3'd1; end
                    6'd3:begin wk3<=wk3-3'd1;wk4<=wk4-3'd1;wk5<=wk5-3'd1; end
                    6'd4:begin wk4<=wk4-3'd1;wk5<=wk5-3'd1;wk6<=wk6-3'd1; end
                    6'd5:begin wk5<=wk5-3'd1;wk6<=wk6-3'd1;wk7<=wk7-3'd1; end
                    6'd6:begin wk6<=wk6-3'd1;wk7<=wk7-3'd1;wk8<=wk8-3'd1; end
                    6'd7:begin wk7<=wk7-3'd1;wk8<=wk8-3'd1;wk9<=wk9-3'd1; end
                    6'd10:begin wk10<=wk10-3'd1;wk11<=wk11-3'd1;wk12<=wk12-3'd1; end
                    6'd11:begin wk11<=wk11-3'd1;wk12<=wk12-3'd1;wk13<=wk13-3'd1; end
                    6'd12:begin wk12<=wk12-3'd1;wk13<=wk13-3'd1;wk14<=wk14-3'd1; end
                    6'd13:begin wk13<=wk13-3'd1;wk14<=wk14-3'd1;wk15<=wk15-3'd1; end
                    6'd14:begin wk14<=wk14-3'd1;wk15<=wk15-3'd1;wk16<=wk16-3'd1; end
                    6'd15:begin wk15<=wk15-3'd1;wk16<=wk16-3'd1;wk17<=wk17-3'd1; end
                    6'd16:begin wk16<=wk16-3'd1;wk17<=wk17-3'd1;wk18<=wk18-3'd1; end
                    6'd19:begin wk19<=wk19-3'd1;wk20<=wk20-3'd1;wk21<=wk21-3'd1; end
                    6'd20:begin wk20<=wk20-3'd1;wk21<=wk21-3'd1;wk22<=wk22-3'd1; end
                    6'd21:begin wk21<=wk21-3'd1;wk22<=wk22-3'd1;wk23<=wk23-3'd1; end
                    6'd22:begin wk22<=wk22-3'd1;wk23<=wk23-3'd1;wk24<=wk24-3'd1; end
                    6'd23:begin wk23<=wk23-3'd1;wk24<=wk24-3'd1;wk25<=wk25-3'd1; end
                    6'd24:begin wk24<=wk24-3'd1;wk25<=wk25-3'd1;wk26<=wk26-3'd1; end
                    6'd25:begin wk25<=wk25-3'd1;wk26<=wk26-3'd1;wk27<=wk27-3'd1; end
                    default:begin wk27<=wk27-3'd1; end
                    endcase
                    eix <= eix + 6'd1;
                end else if (va >= 3'd3) begin
                    case (t0)
                    6'd1:wk1<=wk1-3'd3;6'd2:wk2<=wk2-3'd3;6'd3:wk3<=wk3-3'd3;6'd4:wk4<=wk4-3'd3;
                    6'd5:wk5<=wk5-3'd3;6'd6:wk6<=wk6-3'd3;6'd7:wk7<=wk7-3'd3;
                    6'd10:wk10<=wk10-3'd3;6'd11:wk11<=wk11-3'd3;6'd12:wk12<=wk12-3'd3;
                    6'd13:wk13<=wk13-3'd3;6'd14:wk14<=wk14-3'd3;6'd15:wk15<=wk15-3'd3;
                    6'd16:wk16<=wk16-3'd3;
                    6'd19:wk19<=wk19-3'd3;6'd20:wk20<=wk20-3'd3;6'd21:wk21<=wk21-3'd3;
                    6'd22:wk22<=wk22-3'd3;6'd23:wk23<=wk23-3'd3;6'd24:wk24<=wk24-3'd3;
                    6'd25:wk25<=wk25-3'd3;
                    default:wk26<=wk26-3'd3;
                    endcase
                    eix <= eix + 6'd1;
                end else if (va != 3'd0) ebd <= 1'b1;
                else eix <= eix + 6'd1;
            end
        end
        // --- 4 结论 ---
        default: begin
            if (!ebd && (ez == 6'd0)) begin ewn <= 1'b1; eph <= 3'd0; end
            else if (eps == 1'b0) begin eps <= 1'b1; eph <= 3'd1; end
            else if (epr >= 6'd34) begin eph <= 3'd0; end
            else begin epr <= epr + 6'd1; eps <= 1'b0; eph <= 3'd1; end
        end
        endcase
    end
    // 结果锁存：一旦能和就保持，只有复位(y_clr)才清
    reg yform_l;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)      yform_l <= 1'b0;
        else if (y_clr)    yform_l <= 1'b0;
        else if (ewn)      yform_l <= 1'b1;
    end
    wire yform = yform_l;

    wire        add_en = lock_en && !lock_en_d;
    // ★ res_row 是 BCD（如 0x0033），必须转成二进制牌号：十位*10 + 个位
    wire [5:0]  ad_tens  = {2'd0, res_row[7:4]};
    wire [5:0]  add_tile = (ad_tens << 3) + (ad_tens << 1) + {2'd0, res_row[3:0]};
    integer     ci;

    wire [3:0]  np1 = ((c1>= 3'd2)?4'd1:4'd0)+((c2>= 3'd2)?4'd1:4'd0)+((c3>= 3'd2)?4'd1:4'd0)+((c4>= 3'd2)?4'd1:4'd0)+((c5>= 3'd2)?4'd1:4'd0)+((c6>= 3'd2)?4'd1:4'd0)+((c7>= 3'd2)?4'd1:4'd0);
    wire [3:0]  np2 = ((c8>= 3'd2)?4'd1:4'd0)+((c9>= 3'd2)?4'd1:4'd0)+((c10>= 3'd2)?4'd1:4'd0)+((c11>= 3'd2)?4'd1:4'd0)+((c12>= 3'd2)?4'd1:4'd0)+((c13>= 3'd2)?4'd1:4'd0)+((c14>= 3'd2)?4'd1:4'd0);
    wire [3:0]  np3 = ((c15>= 3'd2)?4'd1:4'd0)+((c16>= 3'd2)?4'd1:4'd0)+((c17>= 3'd2)?4'd1:4'd0)+((c18>= 3'd2)?4'd1:4'd0)+((c19>= 3'd2)?4'd1:4'd0)+((c20>= 3'd2)?4'd1:4'd0)+((c21>= 3'd2)?4'd1:4'd0);
    wire [3:0]  np4 = ((c22>= 3'd2)?4'd1:4'd0)+((c23>= 3'd2)?4'd1:4'd0)+((c24>= 3'd2)?4'd1:4'd0)+((c25>= 3'd2)?4'd1:4'd0)+((c26>= 3'd2)?4'd1:4'd0)+((c27>= 3'd2)?4'd1:4'd0)+((c28>= 3'd2)?4'd1:4'd0);
    wire [3:0]  np5 = ((c29>= 3'd2)?4'd1:4'd0)+((c30>= 3'd2)?4'd1:4'd0)+((c31>= 3'd2)?4'd1:4'd0)+((c32>= 3'd2)?4'd1:4'd0)+((c33>= 3'd2)?4'd1:4'd0)+((c34>= 3'd2)?4'd1:4'd0);
    wire [5:0]  n_pair = np1+np2+np3+np4+np5;
    // 断幺九：只用 2~8 数牌
    wire y_tanyao = (c1==0)&&(c9==0)&&(c10==0)&&(c18==0)&&(c19==0)&&(c27==0)&&
                    (c28==0)&&(c29==0)&&(c30==0)&&(c31==0)&&(c32==0)&&(c33==0)&&(c34==0)&&
                    ((c2|c3|c4|c5|c6|c7|c8)!=0 || (c11|c12|c13|c14|c15|c16|c17)!=0 ||
                     (c20|c21|c22|c23|c24|c25|c26)!=0);
    // 役牌：發(c33) 或 中(c34) 刻子
    wire y_yakuhai = (c33>=3'd3) || (c34>=3'd3);
    // 七対子：7 个对子且无刻子
    wire nt1 = (c1<3'd3)&&(c2<3'd3)&&(c3<3'd3)&&(c4<3'd3)&&(c5<3'd3)&&(c6<3'd3)&&(c7<3'd3);
    wire nt2 = (c8<3'd3)&&(c9<3'd3)&&(c10<3'd3)&&(c11<3'd3)&&(c12<3'd3)&&(c13<3'd3)&&(c14<3'd3);
    wire nt3 = (c15<3'd3)&&(c16<3'd3)&&(c17<3'd3)&&(c18<3'd3)&&(c19<3'd3)&&(c20<3'd3)&&(c21<3'd3);
    wire nt4 = (c22<3'd3)&&(c23<3'd3)&&(c24<3'd3)&&(c25<3'd3)&&(c26<3'd3)&&(c27<3'd3)&&(c28<3'd3);
    wire nt5 = (c29<3'd3)&&(c30<3'd3)&&(c31<3'd3)&&(c32<3'd3)&&(c33<3'd3)&&(c34<3'd3);
    wire no_trip = nt1&&nt2&&nt3&&nt4&&nt5;
    wire y_chiitoi = (n_pair==6'd7) && no_trip;
    // 国士無双：13 种幺九字牌各 >=1
    wire y_kokushi = (c1!=0)&&(c9!=0)&&(c10!=0)&&(c18!=0)&&(c19!=0)&&(c27!=0)&&
                     (c28!=0)&&(c29!=0)&&(c30!=0)&&(c31!=0)&&(c32!=0)&&(c33!=0)&&(c34!=0);
    // 混一色/清一色：只有一种花色有牌
    wire y_man = (c1|c2|c3|c4|c5|c6|c7|c8|c9)!=0;
    wire y_pin = (c10|c11|c12|c13|c14|c15|c16|c17|c18)!=0;
    wire y_sou = (c19|c20|c21|c22|c23|c24|c25|c26|c27)!=0;
    wire y_flush = (((y_man)?4'd1:4'd0)+((y_pin)?4'd1:4'd0)+((y_sou)?4'd1:4'd0))==4'd1;
    wire yaku_any = (yform) || y_tanyao||y_yakuhai||y_chiitoi||y_kokushi||y_flush;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            c1 <= 3'd0;
            c2 <= 3'd0;
            c3 <= 3'd0;
            c4 <= 3'd0;
            c5 <= 3'd0;
            c6 <= 3'd0;
            c7 <= 3'd0;
            c8 <= 3'd0;
            c9 <= 3'd0;
            c10 <= 3'd0;
            c11 <= 3'd0;
            c12 <= 3'd0;
            c13 <= 3'd0;
            c14 <= 3'd0;
            c15 <= 3'd0;
            c16 <= 3'd0;
            c17 <= 3'd0;
            c18 <= 3'd0;
            c19 <= 3'd0;
            c20 <= 3'd0;
            c21 <= 3'd0;
            c22 <= 3'd0;
            c23 <= 3'd0;
            c24 <= 3'd0;
            c25 <= 3'd0;
            c26 <= 3'd0;
            c27 <= 3'd0;
            c28 <= 3'd0;
            c29 <= 3'd0;
            c30 <= 3'd0;
            c31 <= 3'd0;
            c32 <= 3'd0;
            c33 <= 3'd0;
            c34 <= 3'd0;
            y_state <= 1'b0; lock_en_d <= 1'b0;
        end else begin
            lock_en_d <= lock_en;
            // 锁定一张牌 -> 对应计数 +1（case 写法，TD 最稳）
            if (add_en) begin
                case (add_tile)
                    6'd1: if (c1 != 3'd4) c1 <= c1 + 3'd1;
                    6'd2: if (c2 != 3'd4) c2 <= c2 + 3'd1;
                    6'd3: if (c3 != 3'd4) c3 <= c3 + 3'd1;
                    6'd4: if (c4 != 3'd4) c4 <= c4 + 3'd1;
                    6'd5: if (c5 != 3'd4) c5 <= c5 + 3'd1;
                    6'd6: if (c6 != 3'd4) c6 <= c6 + 3'd1;
                    6'd7: if (c7 != 3'd4) c7 <= c7 + 3'd1;
                    6'd8: if (c8 != 3'd4) c8 <= c8 + 3'd1;
                    6'd9: if (c9 != 3'd4) c9 <= c9 + 3'd1;
                    6'd10: if (c10 != 3'd4) c10 <= c10 + 3'd1;
                    6'd11: if (c11 != 3'd4) c11 <= c11 + 3'd1;
                    6'd12: if (c12 != 3'd4) c12 <= c12 + 3'd1;
                    6'd13: if (c13 != 3'd4) c13 <= c13 + 3'd1;
                    6'd14: if (c14 != 3'd4) c14 <= c14 + 3'd1;
                    6'd15: if (c15 != 3'd4) c15 <= c15 + 3'd1;
                    6'd16: if (c16 != 3'd4) c16 <= c16 + 3'd1;
                    6'd17: if (c17 != 3'd4) c17 <= c17 + 3'd1;
                    6'd18: if (c18 != 3'd4) c18 <= c18 + 3'd1;
                    6'd19: if (c19 != 3'd4) c19 <= c19 + 3'd1;
                    6'd20: if (c20 != 3'd4) c20 <= c20 + 3'd1;
                    6'd21: if (c21 != 3'd4) c21 <= c21 + 3'd1;
                    6'd22: if (c22 != 3'd4) c22 <= c22 + 3'd1;
                    6'd23: if (c23 != 3'd4) c23 <= c23 + 3'd1;
                    6'd24: if (c24 != 3'd4) c24 <= c24 + 3'd1;
                    6'd25: if (c25 != 3'd4) c25 <= c25 + 3'd1;
                    6'd26: if (c26 != 3'd4) c26 <= c26 + 3'd1;
                    6'd27: if (c27 != 3'd4) c27 <= c27 + 3'd1;
                    6'd28: if (c28 != 3'd4) c28 <= c28 + 3'd1;
                    6'd29: if (c29 != 3'd4) c29 <= c29 + 3'd1;
                    6'd30: if (c30 != 3'd4) c30 <= c30 + 3'd1;
                    6'd31: if (c31 != 3'd4) c31 <= c31 + 3'd1;
                    6'd32: if (c32 != 3'd4) c32 <= c32 + 3'd1;
                    6'd33: if (c33 != 3'd4) c33 <= c33 + 3'd1;
                    6'd34: if (c34 != 3'd4) c34 <= c34 + 3'd1;
                endcase
            end
            // ★ 每轮开始（框13 -> 框0）清零；空态复位也清
            box_sel_y <= box_sel;
            if (y_clr) begin                      // S_RESET: clear tile counters only
                y_state <= 1'b0;                 // ★ 复位时 Y 一起清零
                c1<=3'd0;  c2<=3'd0;  c3<=3'd0;  c4<=3'd0;  c5<=3'd0;  c6<=3'd0;  c7<=3'd0;
                c8<=3'd0;  c9<=3'd0;  c10<=3'd0; c11<=3'd0; c12<=3'd0; c13<=3'd0; c14<=3'd0;
                c15<=3'd0; c16<=3'd0; c17<=3'd0; c18<=3'd0; c19<=3'd0; c20<=3'd0; c21<=3'd0;
                c22<=3'd0; c23<=3'd0; c24<=3'd0; c25<=3'd0; c26<=3'd0; c27<=3'd0; c28<=3'd0;
                c29<=3'd0; c30<=3'd0; c31<=3'd0; c32<=3'd0; c33<=3'd0; c34<=3'd0;
            end
            // ★ 在"一轮结束"（框13 -> 框0）那一拍判定，不依赖框13 是否锁定
            if (y_eval) y_state <= yaku_any;      // ★ 状态机 S_EVAL 触发评估
        end
    end

    reg         rst_req;
    reg  [6:0]  ff_cnt;                                  // ★ 结果连续 FFFF 的帧数（60 帧触发复位）                                  // ★ 请求回到框0（复位框1）
    // ★ 空态判定：W 在 56±3 且 H 在 90±3（且 P=0101）
    wire [11:0] w_d56 = (pb_w_d > 12'd56) ? (pb_w_d - 12'd56) : (12'd56 - pb_w_d);
    wire [11:0] h_d90 = (pb_h_d > 12'd92) ? (pb_h_d - 12'd92) : (12'd92 - pb_h_d);   // ★ 高目标 92
    // ★ 复位条件：W=5692±3、P=0101、R=0000（无红且红量为 0）
    // ★ 空态：P 行 = 0101、R 行 = 0000、Z 行 = 0000
    wire empty_now = (pb_npc_d == 7'd1) && (pb_npr_d == 7'd1)      // P 0101
                  && !pb_red && (pb_redc_d[12:3] == 10'd0)         // R 0000
                  && (cdiff == 8'd0) && (rdiff == 8'd0);           // Z 0000
    wire        rst_active = (box_sel == 4'd13);          // ★ 到框13 就检测空态（不要求锁定）   // ★ 只有框5 也锁定后才检测空态
    // ★ 只有"有效牌号"才允许锁定：高字节必须为 0（排除 FFFF=没命中、DDDD=不可分）
    wire        n_valid    = (res_row[15:8] == 8'h00) && (res_row[7:0] != 8'h00) && (res_row[7:0] <= 8'h37);
    wire [15:0] n_show     = lock_en ? lock_val : res_row;      // 锁定后就显示锁定值
    wire [3:0]  n_show_hu  = n_show[7:4];                       // 显示低两位（十位）
    wire [3:0]  n_show_un  = n_show[3:0];                       // 个位
    wire [15:0] bcd_lock   = {4'd0, 4'd0, (lock_en ? 4'd1 : 4'd0), lock_cnt};  // 标志 + 计数

    // ========================================================================
    //  ★ 总状态机（复位 + Y 全部由它管理；res_b[] 只是 D 行的数据）
    //     S_SCAN : 扫描 14 个框，每框锁定后前进
    //     S_EVAL : 框13（13 张已收齐）-> 评估 Y + 检测空态
    //     S_EMPTY: 空态计 30 帧
    //     S_RESET: 清 D(res_b) / 绿框 / 牌计数 / Y -> 回 S_SCAN
    // ========================================================================
    localparam [2:0] S_SCAN = 3'd0, S_EVAL = 3'd1, S_EMPTY = 3'd2, S_RESET = 3'd3;
    reg  [2:0] st;
    // ===================== 安路 logo 叠加（SW1 下显示） =====================
    localparam [11:0] LOGO_W = 12'd160;
    localparam [11:0] LOGO_H = 12'd160;
    // ★ ERAM 版（RGB565、半分辨率）：地址 = (y>>1)*128 + (x>>1)，同步读
    wire [13:0] logo_addr = ({2'b00, S_y_2d[10:2], 6'd0}) + {8'd0, S_x_2d[11:2]};   // 40x40、stride 64、4x 放大
    wire [15:0] logo_p565;
    anlogic_logo_rom u_anlogic_logo_rom (
        .I_clk   ( I_clk      ),
        .I_addr  ( logo_addr  ),
        .O_pixel ( logo_p565  )
    );
    wire logo_op = (logo_p565 != 16'd0);                       // 非黑 = 有内容
    wire logo_hit = (S_x_2d < LOGO_W) && (S_y_2d < LOGO_H) && logo_op;
    wire [23:0] logo_rgb = {logo_p565[15:11], 3'b000,
                            logo_p565[10:5],  2'b00,
                            logo_p565[4:0],   3'b000};

    reg        clr_boxes, y_clr, y_eval;
    reg  [1:0] clr_hold;                                 // ★ 清框保持 3 帧
    wire       clr_boxes_w = clr_boxes || (clr_hold != 2'd0);
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st <= S_SCAN; res_prev <= 16'hFFFF; lock_cnt <= 3'd0; empty_cnt <= 5'd0;
            for (rk = 0; rk < 14; rk = rk + 1) res_b[rk] <= 16'hFFFF;
            box_sel <= 4'd0; lock_val <= 16'hFFFF; lock_en <= 1'b0;
            y_clr <= 1'b0; y_eval <= 1'b0;
            if (pb_latch && clr_boxes) clr_boxes <= 1'b0;   // 消费
            // ★ 关键：clr_hold 必须按【帧】递减（不是按时钟），否则只有几十 ns 会错过 pb_i==187
            if (clr_boxes)                        clr_hold <= 2'd3;
            else if (pb_latch && (clr_hold != 2'd0)) clr_hold <= clr_hold - 2'd1;
        end else begin
            clr_boxes <= 1'b0; y_clr <= 1'b0; y_eval <= 1'b0;   // box_vis 是状态位，不能每拍清零 默认单拍
            // ---- 锁定检测：连续 6 帧同一结果 ----
            if (pb_latch) begin
                if (n_valid && (res_row == res_prev)) begin
                    if (lock_cnt != 3'd7) lock_cnt <= lock_cnt + 3'd1;
                    if ((lock_cnt >= 3'd5) && !lock_en) begin
                        lock_en  <= 1'b1;
                        lock_val <= res_row;
                        res_b[box_sel] <= res_row;
                    end
                end else begin
                    res_prev <= res_row;
                    lock_cnt <= 3'd0;
                end
            end
            // ---- 状态机 ----
            if (pb_latch && k3_req) st <= S_RESET;               // KEY3：任何时候复位
            case (st)
                S_SCAN: if (pb_latch) begin
                    if (lock_en && (box_sel != 4'd13)) begin      // 本框锁定 -> 下一个框
                        box_sel  <= box_sel + 4'd1;
                        lock_en  <= 1'b0;
                        lock_cnt <= 3'd0;
                        res_prev <= 16'hFFFF;
                    end else if (lock_en && (box_sel == 4'd13)) begin
                        st <= S_EVAL;                            // ★ 框13 锁定过一次 -> 进入监视
                    end
                end                S_EVAL: if (pb_latch) begin
                    y_eval <= 1'b1; est <= 1'b1;                              // 评估"有无役"
                    // ★ 框13 已识别过一次 -> 持续判断 W/P/R 真空，连续 30 帧才复位
                    if (empty_now) begin
                        if (empty_cnt >= 5'd29) st <= S_RESET;
                        else empty_cnt <= empty_cnt + 5'd1;
                    end else begin
                        empty_cnt <= 5'd0;
                    end
                end                S_EMPTY: if (pb_latch) begin
                    if (!empty_now) begin empty_cnt <= 5'd0; st <= S_EVAL; end
                    else if (empty_cnt >= 5'd30) st <= S_RESET;
                    else empty_cnt <= empty_cnt + 5'd1;
                end
                default: begin                                   // S_RESET
                    for (rk = 0; rk < 14; rk = rk + 1) res_b[rk] <= 16'hFFFF;   // ★ 清所有 D
                    clr_boxes <= 1'b1;
                    y_clr     <= 1'b1;                          // 清 Y + 牌计数
                    box_sel   <= 4'd0;
                    lock_en   <= 1'b0; lock_cnt <= 3'd0; res_prev <= 16'hFFFF;
                    lock_val  <= 16'hFFFF; empty_cnt <= 5'd0;
                    st <= S_SCAN;
                end
            endcase
        end
    end

    // ★ Z 行：列投影 最大/最小（比值 = 最大/最小）
    reg  [7:0]  cmax, cmin;                              // ★ 8 位（0..255）
    reg  [7:0]  cmax_d, cmin_d;
    // ★ Z（行列差）的 4 个极值也做 4 帧平滑（精确 >>2）
    reg  [7:0]  cmx_h1, cmx_h2, cmx_h3, cmn_h1, cmn_h2, cmn_h3;
    reg  [7:0]  rmx_h1, rmx_h2, rmx_h3, rmn_h1, rmn_h2, rmn_h3;
    wire [9:0]  zs_cmax = cmax + cmx_h1 + cmx_h2 + cmx_h3;
    wire [9:0]  zs_cmin = cmin + cmn_h1 + cmn_h2 + cmn_h3;
    wire [9:0]  zs_rmax = rmax + rmx_h1 + rmx_h2 + rmx_h3;
    wire [9:0]  zs_rmin = rmin + rmn_h1 + rmn_h2 + rmn_h3;
    wire [7:0]  cmx_dn = (cmax > cmax_d) ? (cmax - cmax_d) : (cmax_d - cmax);
    wire [7:0]  cmn_dn = (cmin > cmin_d) ? (cmin - cmin_d) : (cmin_d - cmin);
    reg  [7:0]  rmax, rmin, rmax_d, rmin_d;              // ★ 行投影最大/最小
    wire [7:0]  rmx_dn = (rmax > rmax_d) ? (rmax - rmax_d) : (rmax_d - rmax);
    wire [7:0]  rmn_dn = (rmin > rmin_d) ? (rmin - rmin_d) : (rmin_d - rmin);
    // ★ 迟滞：显示值只在变化 > 2 时更新，抑制帧间抖动
    reg  [6:0]  cov_c_d, cov_r_d;
    wire [6:0]  cov_c_dn = (cov_c > cov_c_d) ? (cov_c - cov_c_d) : (cov_c_d - cov_c);
    wire [6:0]  cov_r_dn = (cov_r > cov_r_d) ? (cov_r - cov_r_d) : (cov_r_d - cov_r);
    reg  [7:0]  h_col_pct, h_row_pct;                    // E 行：列熵% / 行熵%
    reg  [6:0]  w_eff, h_eff;                            // F 行：有效宽度 / 有效高度

    // ★ 投影熵的独立 FSM：t = Σ/S（商 0..7），前 8 拍列、后 8 拍行，第 16 拍锁存
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ediv_step <= 5'd17; et_c <= 4'd0; et_r <= 4'd0; ecc_c <= 16'd0; ecc_r <= 16'd0;
            h_col_pct <= 8'd0; h_row_pct <= 8'd0; w_eff <= 7'd0; h_eff <= 7'd0;
        end else if (pb_latch) begin
            ediv_step <= 5'd0; ecc_c <= esumw_c; ecc_r <= esumw_r; et_c <= 4'd0; et_r <= 4'd0;
        end else if (ediv_step < 5'd8) begin
            if (ecc_c >= {2'd0, esum_c}) begin ecc_c <= ecc_c - {2'd0, esum_c}; et_c <= et_c + 4'd1; end
            ediv_step <= ediv_step + 5'd1;
        end else if (ediv_step < 5'd16) begin
            if (ecc_r >= {2'd0, esum_r}) begin ecc_r <= ecc_r - {2'd0, esum_r}; et_r <= et_r + 4'd1; end
            ediv_step <= ediv_step + 5'd1;
        end else if (ediv_step == 5'd16) begin
            // ★ E 行迟滞：变化 > 2 才更新
            if (((ep_c[13:6] > h_col_pct) ? (ep_c[13:6] - h_col_pct) : (h_col_pct - ep_c[13:6])) > 8'd2)
                h_col_pct <= ep_c[13:6];
            if (((ep_r[13:6] > h_row_pct) ? (ep_r[13:6] - h_row_pct) : (h_row_pct - ep_r[13:6])) > 8'd2)
                h_row_pct <= ep_r[13:6];
            w_eff     <= ewc[6:0];   h_eff     <= ewr[6:0];
            ediv_step <= 5'd17;
        end
    end

    // 桶权求和（移位加法，零乘法）
    wire [15:0] esumw_c = ({9'd0,en2}<<1) + {9'd0,en2}
                        + ({9'd0,en3}<<3) + ({9'd0,en3}<<2) + ({9'd0,en3}<<1)
                        + ({9'd0,en4}<<5) + ({9'd0,en4}<<3) + {9'd0,en4}
                        + ({9'd0,en5}<<6) + ({9'd0,en5}<<5) + ({9'd0,en5}<<3) + ({9'd0,en5}<<1) + {9'd0,en5}
                        + ({9'd0,en6}<<9) + ({9'd0,en6}<<6) + ({9'd0,en6}<<5) + ({9'd0,en6}<<4) + ({9'd0,en6}<<2);
    wire [15:0] esumw_r = ({9'd0,er2}<<1) + {9'd0,er2}
                        + ({9'd0,er3}<<3) + ({9'd0,er3}<<2) + ({9'd0,er3}<<1)
                        + ({9'd0,er4}<<5) + ({9'd0,er4}<<3) + {9'd0,er4}
                        + ({9'd0,er5}<<6) + ({9'd0,er5}<<5) + ({9'd0,er5}<<3) + ({9'd0,er5}<<1) + {9'd0,er5}
                        + ({9'd0,er6}<<9) + ({9'd0,er6}<<6) + ({9'd0,er6}<<5) + ({9'd0,er6}<<4) + ({9'd0,er6}<<2);

    // 16*log2(v)：最高位位置 + 其后 3 位查 8 项微表
    function [7:0] elog16;
        input [13:0] v;
        integer k; reg [2:0] m;
        begin
            elog16 = 8'd0;
            if (v != 14'd0) begin
                k = 0;
                if (v[13]) k=13; else if (v[12]) k=12; else if (v[11]) k=11;
                else if (v[10]) k=10; else if (v[9]) k=9; else if (v[8]) k=8;
                else if (v[7]) k=7; else if (v[6]) k=6; else if (v[5]) k=5;
                else if (v[4]) k=4; else if (v[3]) k=3; else if (v[2]) k=2;
                else if (v[1]) k=1;
                if (k >= 3) m = (v >> (k-3)); else m = 3'd0;
                case (m)
                    3'd0: elog16 = (k<<4);        3'd1: elog16 = (k<<4) + 8'd3;
                    3'd2: elog16 = (k<<4) + 8'd5; 3'd3: elog16 = (k<<4) + 8'd7;
                    3'd4: elog16 = (k<<4) + 8'd9; 3'd5: elog16 = (k<<4) + 8'd11;
                    3'd6: elog16 = (k<<4) + 8'd13; default: elog16 = (k<<4) + 8'd15;
                endcase
            end
        end
    endfunction

    // 2^(b/16) 的 16 项微表（16*(2^(b/16)-1)）
    function [4:0] etab2x;
        input [3:0] b;
        begin
            case (b)
                4'd0: etab2x=5'd0;  4'd1: etab2x=5'd1;  4'd2: etab2x=5'd2;  4'd3: etab2x=5'd3;
                4'd4: etab2x=5'd4;  4'd5: etab2x=5'd5;  4'd6: etab2x=5'd6;  4'd7: etab2x=5'd7;
                4'd8: etab2x=5'd8;  4'd9: etab2x=5'd8;  4'd10: etab2x=5'd9; 4'd11: etab2x=5'd10;
                4'd12: etab2x=5'd11;4'd13: etab2x=5'd12;4'd14: etab2x=5'd13;default: etab2x=5'd15;
            endcase
        end
    endfunction

    wire [7:0] els_c = elog16(esum_c), els_r = elog16(esum_r);
    wire [8:0] eh_c  = ({1'b0, els_c} >= {5'd0, et_c, 4'd0}) ? ({1'b0, els_c} - {5'd0, et_c, 4'd0}) : 9'd0;
    wire [8:0] eh_r  = ({1'b0, els_r} >= {5'd0, et_r, 4'd0}) ? ({1'b0, els_r} - {5'd0, et_r, 4'd0}) : 9'd0;
    wire [13:0] ep_c = ({5'd0, eh_c}<<4) + {5'd0, eh_c};                                  // ×17（100/log2 56）
    wire [13:0] ep_r = ({5'd0, eh_r}<<3) + ({5'd0, eh_r}<<2) + ({5'd0, eh_r}<<1) + {5'd0, eh_r}; // ×15
    wire [15:0] ewc  = (16'd16 << eh_c[7:4]) + ({9'd0, etab2x(eh_c[3:0])} << eh_c[7:4]);
    wire [15:0] ewr  = (16'd16 << eh_r[7:4]) + ({9'd0, etab2x(eh_r[3:0])} << eh_r[7:4]);

    reg         pb_scan;
    reg  [7:0]  pb_i;
    reg         pb_gotx, pb_goty;                        // 列/行是否已经找到第一个"峰"
    reg         pb_inpx, pb_inpy;                        // 列/行当前是否在峰内
    reg  [11:0] pb_x1n, pb_x2n, pb_y1n, pb_y2n;          // 本帧扫描出的候选值
    reg        pb_ok_d;                                  // 上一帧是否有效
    wire       pb_ok_now  = pb_gotx && pb_goty;          // 本帧是否找到包围盒
    wire       pb_ok_rise = pb_ok_now && !pb_ok_d;       // ★ 无效->有效的上升沿（只放行一次）
    // ★ 红色检测（在二值化之前，直接用 RGB 判）：框 0 窗口内红像素数 >= RED_THR -> 结论 1
    //   红判据：R >= 90 且 R 比 G 大 40 以上、比 B 大 40 以上（近似"红主导"）
            localparam [9:0] RED_THR_Q = 10'd13;                 // 判有无红：q >= 18（红像素 >= 144）
                                                         //   q = 红像素数/8  -> 实际要求 红像素数 >= 48
    localparam integer RED_SHIFT = 3;                    // ★ 量化比例：q = count / 8
                                                         //   （必须在下面 red_q 之前声明：参数不能先用后声明）
    // ========================================================================
    //  ★ RGB -> Cr（只算红色色度，不算 Y、不算 Cb）
    //    BT.601：Cr - 128 ∝ 0.5R - 0.4187G - 0.0813B
    //    两边 ×16 并取整 -> 16(Cr-128) ≈ 8R - 6.75G - 1.25B
    //    只用移位/加法实现（★ 零乘法）：8R = R<<3、(27G)>>2 = 6.75G、B + B>>2 = 1.25B
    //    再整体加 4096 偏移，避免有符号比较（结果范围约 2057~6136）
    // ★ 白/灰（R=G=B）算出来正好 = 4096（对应 Cr=128，中性）-> 不会误判成红
    // ★ 越红 cr_off 越大：纯红 R=255,G=B=0 -> 6136（Cr≈255）
    // ========================================================================
    wire [8:0]  r9 = {1'b0, S_video_data_2d[23:16]};
    wire [8:0]  g9 = {1'b0, S_video_data_2d[15:8]};
    wire [8:0]  b9 = {1'b0, S_video_data_2d[7:0]};
    wire [12:0] cr_r  = {4'd0, r9} << 3;                                 // 8R（干净，无隐式截断）
    wire [12:0] cr_g  = ({2'b00, g9} << 4) + ({2'b00, g9} << 3)
                      + ({2'b00, g9} << 1) +  {2'b00, g9};               // 27G
    wire [12:0] cr_g4 = cr_g >> 2;                                       // 6.75G
    wire [12:0] cr_b  = {2'b00, b9} + ({2'b00, b9} >> 2);                // 1.25B
    wire [12:0] cr_off = 13'd4096 + cr_r - cr_g4 - cr_b;                 // 带偏移的 Cr 度量
            localparam [12:0] RED_BIAS = 13'd4600;               // red chroma threshold (Cr-128 > 31)
    // ★ 判红只看 Cr 偏置（用户指定：只用 RGB->Cr 这一路，不加任何其它补丁条件）
    wire red_pix = (cr_off > RED_BIAS);
    // ★ 红像素计数：窗口最大 56x92 = 5152 个，13 位够（不再封顶）
    //   OSD 显示的是【量化值】q = 红像素数 >> RED_SHIFT（现在 /8，范围 0..644）
    reg  [12:0] pb_redc;                                 // 本帧窗口内红像素计数（0..5152）
    reg  [12:0] pb_redc_d;                               // 锁存给 OSD
    reg         pb_red;                                  // 结论：1 = 红色够多
    // ★ 峰计数：本帧竖直投影（列）与水平投影（行）各识别到几个峰
    reg  [6:0] pb_npc, pb_npr;                           // 扫描中的计数 0..56 / 0..92
    reg  [6:0] pb_npc_d, pb_npr_d;                       // 锁存给 OSD 显示
    // ★ 一致性锁存：宽/高/红标志 与 峰数 在同一帧锁存，让 matcher 与 OSD 看到同一组特征
    reg  [11:0] pb_w_d, pb_h_d;
    reg         pb_red_d;
    reg         pb_latch, pb_latch_d;                    // 扫描结束后的两级选通                       // 锁存给 OSD 显示                                  // 上一帧是否有效（首次强制刷新）
    // ========================================================================
    //  ★ 4 帧滑动平均（替代边界迟滞）
    //    每帧 4 条边界各存 3 帧历史 -> 和 4 帧 -> >>2 精确除以 4（无近似误差）
    //    历史未满 3 帧时直接用原始候选（避免被 0 拉小）
    // ========================================================================
    reg  [11:0] pb_x1h1, pb_x1h2, pb_x1h3, pb_x2h1, pb_x2h2, pb_x2h3;
    reg  [11:0] pb_y1h1, pb_y1h2, pb_y1h3, pb_y2h1, pb_y2h2, pb_y2h3;
    reg  [1:0]  pb_avg_fill;
    wire        pb_avg_ok = (pb_avg_fill == 2'd3);
    wire [13:0] s_x1 = pb_x1n + pb_x1h1 + pb_x1h2 + pb_x1h3;
    wire [13:0] s_x2 = pb_x2n + pb_x2h1 + pb_x2h2 + pb_x2h3;
    wire [13:0] s_y1 = pb_y1n + pb_y1h1 + pb_y1h2 + pb_y1h3;
    wire [13:0] s_y2 = pb_y2n + pb_y2h1 + pb_y2h2 + pb_y2h3;
    localparam [11:0] PB_HYST = 12'd2;                   // ★ N = 2（用户指定）
    wire [11:0] pb_dx1 = (pb_x1n > pb_x1) ? (pb_x1n - pb_x1) : (pb_x1 - pb_x1n);
    wire [11:0] pb_dx2 = (pb_x2n > pb_x2) ? (pb_x2n - pb_x2) : (pb_x2 - pb_x2n);
    wire [11:0] pb_dy1 = (pb_y1n > pb_y1) ? (pb_y1n - pb_y1) : (pb_y1 - pb_y1n);
    wire [11:0] pb_dy2 = (pb_y2n > pb_y2) ? (pb_y2n - pb_y2) : (pb_y2 - pb_y2n);
    reg  [11:0] pb_x1, pb_x2, pb_y1, pb_y2;              // ★ 字符包围盒（屏幕像素坐标，带迟滞）

    wire pb_end      = S_video_de_2d && (S_x_2d == 12'd0) && (S_y_2d == (PBY2S + 12'd1));
    wire pb_scan_col = pb_scan && (pb_i <= 8'd75);        // 前 76 拍扫列
    wire [7:0] pb_ridx = pb_i - 8'd76;                    // ★ 行扫描索引 0..111
    wire [5:0] pb_cur = pb_scan_col ? pb_col[pb_i[6:0]] : pb_row[pb_ridx[6:0]];
    wire pb_hit_peak = (pb_cur >= PB_PEAK);
    wire pb_hit_val  = (pb_cur <= PB_VAL);
    reg  pb_ok;
    // ★ 字符小框：只在找到内容时画（绿色 1 像素边，用包围盒）
    // ★ 画两个框各自最近一次的包围盒
    // ★ 框13（小框）的红色边框
    wire smbox_hit = box_hit(12'd1111, 12'd1183, TBX_Y1, TBX_Y2, S_x_2d, S_y_2d);
    wire box_any_hit =
        (bok_b[0] && box_hit(bx1_b[0], bx2_b[0], by1_b[0], by2_b[0], S_x_2d, S_y_2d)) ||
        (bok_b[1] && box_hit(bx1_b[1], bx2_b[1], by1_b[1], by2_b[1], S_x_2d, S_y_2d)) ||
        (bok_b[2] && box_hit(bx1_b[2], bx2_b[2], by1_b[2], by2_b[2], S_x_2d, S_y_2d)) ||
        (bok_b[3] && box_hit(bx1_b[3], bx2_b[3], by1_b[3], by2_b[3], S_x_2d, S_y_2d)) ||
        (bok_b[4] && box_hit(bx1_b[4], bx2_b[4], by1_b[4], by2_b[4], S_x_2d, S_y_2d)) ||
        (bok_b[5] && box_hit(bx1_b[5], bx2_b[5], by1_b[5], by2_b[5], S_x_2d, S_y_2d)) ||
        (bok_b[6] && box_hit(bx1_b[6], bx2_b[6], by1_b[6], by2_b[6], S_x_2d, S_y_2d)) ||
        (bok_b[7] && box_hit(bx1_b[7], bx2_b[7], by1_b[7], by2_b[7], S_x_2d, S_y_2d)) ||
        (bok_b[8] && box_hit(bx1_b[8], bx2_b[8], by1_b[8], by2_b[8], S_x_2d, S_y_2d)) ||
        (bok_b[9] && box_hit(bx1_b[9], bx2_b[9], by1_b[9], by2_b[9], S_x_2d, S_y_2d)) ||
        (bok_b[10] && box_hit(bx1_b[10], bx2_b[10], by1_b[10], by2_b[10], S_x_2d, S_y_2d)) ||
        (bok_b[11] && box_hit(bx1_b[11], bx2_b[11], by1_b[11], by2_b[11], S_x_2d, S_y_2d)) ||
        (bok_b[12] && box_hit(bx1_b[12], bx2_b[12], by1_b[12], by2_b[12], S_x_2d, S_y_2d)) ||
        (bok_b[13] && box_hit(bx1_b[13], bx2_b[13], by1_b[13], by2_b[13], S_x_2d, S_y_2d));

    integer pi, pj;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (pi = 0; pi < 76; pi = pi + 1)  pb_col[pi] <= 2'd0;
            for (pi = 0; pi < 76; pi = pi + 1)  pc7[pi] <= 7'd0;
            for (pj = 0; pj < 112; pj = pj + 1) pb_row[pj] <= 2'd0;
            pb_scan <= 1'b0; pb_i <= 8'd0; pb_gotx <= 1'b0; pb_goty <= 1'b0;
            pb_inpx <= 1'b0; pb_inpy <= 1'b0; pb_ok <= 1'b0; pb_ok_d <= 1'b0;
            pb_x1n <= 12'd0; pb_x2n <= 12'd0; pb_y1n <= 12'd0; pb_y2n <= 12'd0;
            pb_npc <= 7'd0; pb_npr <= 7'd0; pb_npc_d <= 7'd0; pb_npr_d <= 7'd0;
            pb_redc <= 13'd0; pb_redc_d <= 13'd0; pb_red <= 1'b0;
            d_red <= 13'd0; d_dark <= 13'd0; d_red_l <= 13'd0; d_dark_l <= 13'd0;
            pb_w_d <= 12'd0; pb_h_d <= 12'd0; pb_red_d <= 1'b0;
            pb_latch <= 1'b0; pb_latch_d <= 1'b0;
            cov_c_d <= 7'd0; cov_r_d <= 7'd0;
            cmax<=8'd0; cmin<=8'hFF; cmax_d<=8'd0; cmin_d<=8'hFF; rmax<=8'd0; rmin<=8'hFF; rmax_d<=8'd0; rmin_d<=8'hFF;
            pb_x1 <= 12'd0; pb_x2 <= 12'd0; pb_y1 <= 12'd0; pb_y2 <= 12'd0;
            pb_avg_fill <= 2'd0;                 // box_sel 由锁定 FSM 块复位
            for (rk = 0; rk < 14; rk = rk + 1) begin bx1_b[rk]<=0; bx2_b[rk]<=0; by1_b[rk]<=0; by2_b[rk]<=0; bok_b[rk]<=1'b0; end
            pb_x1h1<=0; pb_x1h2<=0; pb_x1h3<=0; pb_x2h1<=0; pb_x2h2<=0; pb_x2h3<=0;
            cmx_h1<=0; cmx_h2<=0; cmx_h3<=0; cmn_h1<=0; cmn_h2<=0; cmn_h3<=0;
            cmax7 <= 8'd0; cmin7 <= 8'd255; cdiff_t <= 8'd0;
            rmx_h1<=0; rmx_h2<=0; rmx_h3<=0; rmn_h1<=0; rmn_h2<=0; rmn_h3<=0;
            pb_y1h1<=0; pb_y1h2<=0; pb_y1h3<=0; pb_y2h1<=0; pb_y2h2<=0; pb_y2h3<=0;
        end else begin
            // ---- ★ 红色计数（二值化之前，只数框 0 窗口内的红像素）----
            if (S_video_de_2d && pb_in && red_pix && (pb_redc < 13'd5152))
                pb_redc <= pb_redc + 13'd1;              // ★ 计到窗口满为止，不封顶
            // ---- 框内投影累加（饱和到 63）----
            // ---- ★ 两级选通：第 1 拍 pb_x1.. 更新，第 2 拍把 5 个特征统一锁存 ----
            pb_latch   <= pb_scan && (pb_i == 8'd187);
            pb_latch_d <= pb_latch;
            if (pb_latch) begin
                // ★ 覆盖列/行的显示迟滞
                // ★ 4 帧平均（历史未满时直接用当前值）
                if (pb_avg_ok) begin
                    cmax_d <= zs_cmax[9:2]; cmin_d <= zs_cmin[9:2];
                    rmax_d <= zs_rmax[9:2]; rmin_d <= zs_rmin[9:2];
                end else begin
                    cmax_d <= cmax; cmin_d <= cmin; rmax_d <= rmax; rmin_d <= rmin;
                end
                cmx_h3<=cmx_h2; cmx_h2<=cmx_h1; cmx_h1<=cmax;
                cmn_h3<=cmn_h2; cmn_h2<=cmn_h1; cmn_h1<=cmin;
                rmx_h3<=rmx_h2; rmx_h2<=rmx_h1; rmx_h1<=rmax;
                rmn_h3<=rmn_h2; rmn_h2<=rmn_h1; rmn_h1<=rmin;
                // ★ 行差真实值（未平滑，7 位）
                cdiff_t <= (cmax7 >= cmin7) ? (cmax7 - cmin7) : 8'd0;   // ★ 真实列差（7 位计数器）
                if (cov_c_dn > 7'd2) cov_c_d <= cov_c;
                if (cov_r_dn > 7'd2) cov_r_d <= cov_r;
                pb_w_d   <= pb_w12;                      // 宽（此时 pb_x1/pb_x2 已是本帧新值）
                pb_h_d   <= pb_h12;                      // 高
                pb_red_d <= pb_red;                      // 红标志（本帧 pb_end 时已更新）
            end
            // ---- ★ D 行统计：绿框内的红/暗像素（只显示）----
            if (S_video_de_2d && pb_ok &&
                (S_x_2d >= pb_x1) && (S_x_2d <= pb_x2) &&
                (S_y_2d >= pb_y1) && (S_y_2d <= pb_y2)) begin
                if (red_pix)  d_red  <= d_red  + 13'd1;
                if (bin_dark) d_dark <= d_dark + 13'd1;
            end
            if (S_video_de_2d && pb_in && pb_ink) begin
                for (pi = 0; pi < 76; pi = pi + 1)
                    if (pi == pb_xr[6:0]) begin
                        pb_col[pi] <= (pb_col[pi] == 6'd63) ? 6'd63 : pb_col[pi] + 6'd1;
                        if (pc7[pi] != 7'd127) pc7[pi] <= pc7[pi] + 7'd1;   // 真实列计数（不饱和）
                    end
                for (pj = 0; pj < 112; pj = pj + 1)
                    if (pj == pb_yr[6:0])
                        pb_row[pj] <= (pb_row[pj] == 6'd63) ? 6'd63 : pb_row[pj] + 6'd1;
            end
            // ---- 框末：启动扫描 ----
            if (pb_end) begin
                // ★ 熵：每帧清零分桶与 S
                en2<=7'd0; en3<=7'd0; en4<=7'd0; en5<=7'd0; en6<=7'd0; esum_c<=14'd0; cov_c<=7'd0; cmax<=8'd0; cmin<=8'hFF;
                cmax7<=8'd0; cmin7<=8'hFF;   // ★ 真实列差的极值也要每帧清零
                er2<=7'd0; er3<=7'd0; er4<=7'd0; er5<=7'd0; er6<=7'd0; esum_r<=14'd0; cov_r<=7'd0; rmax<=8'd0; rmin<=8'hFF;
                // ★ 红色结论：窗口内红像素 >= RED_THR -> pb_red = 1
                pb_redc_d <= pb_redc;
                pb_red    <= (({3'd0, pb_redc} >> RED_SHIFT) >= RED_THR_Q);   // 量化值 q >= 6 才判红
                pb_redc   <= 13'd0;
                d_red_l   <= d_red;                      // ★ 清零前先锁存给显示
                d_dark_l  <= d_dark;
                d_red     <= 13'd0; d_dark <= 13'd0;
                pb_scan <= 1'b1; pb_i <= 8'd0;
                pb_gotx <= 1'b0; pb_goty <= 1'b0; pb_inpx <= 1'b0; pb_inpy <= 1'b0;
                pb_npc  <= 7'd0; pb_npr  <= 7'd0;
            end else if (pb_scan) begin
                // ★ 谷->峰 = 左/上边界（只在第一次上升沿取）；峰->谷 = 右/下边界（每次都更新）
                if (pb_scan_col) begin
                    if (!pb_inpx && pb_hit_peak) begin
                        pb_inpx <= 1'b1;
                        pb_npc  <= pb_npc + 7'd1;              // ★ 列峰 +1
                        if (!pb_gotx) begin pb_gotx <= 1'b1; pb_x1n <= PBX1S + {4'd0, pb_i}; end
                    end else if (pb_inpx && pb_hit_val) begin
                        pb_inpx <= 1'b0;
                        pb_x2n  <= PBX1S + {4'd0, pb_i} - 12'd1;
                    end
                    if ((pb_i == 8'd75) && pb_inpx && !pb_hit_val)        // 峰延续到内缩区右边界
                        pb_x2n <= PBX1S + 12'd55;   // 内缩区右端 = 121
                end else begin
                    if (!pb_inpy && pb_hit_peak) begin
                        pb_inpy <= 1'b1;
                        pb_npr  <= pb_npr + 7'd1;              // ★ 行峰 +1
                        if (!pb_goty) begin pb_goty <= 1'b1; pb_y1n <= PBY1S + {4'd0, pb_ridx}; end
                    end else if (pb_inpy && pb_hit_val) begin
                        pb_inpy <= 1'b0;
                        pb_y2n  <= PBY1S + {4'd0, pb_ridx} - 12'd1;
                    end
                    if ((pb_i == 8'd187) && pb_inpy && !pb_hit_val)      // 峰延续到内缩区下边界
                        pb_y2n <= PBY1S + 12'd91;   // 内缩区下端 = 341
                end
                pb_i <= pb_i + 8'd1;
                if (pb_i == 8'd187) begin
                    pb_scan <= 1'b0;
                    pb_ok   <= pb_gotx && pb_goty;
                    pb_npc_d <= pb_npc; pb_npr_d <= pb_npr;   // ★ 峰数锁存给 OSD
                    // ★ 锁定那一刻把结果存入对应框（未锁定的框保持 ff）
                    // (结果锁存移入锁定分支)
                    // ★ 把本帧包围盒存到当前框（画框用）
                    if (clr_boxes_w) begin
                        // ★ 状态机 S_RESET：清掉所有绿框（保持 3 帧）
                        for (rk = 0; rk < 14; rk = rk + 1) begin
                            bok_b[rk]<=1'b0; bx1_b[rk]<=12'd0; bx2_b[rk]<=12'd0;
                            by1_b[rk]<=12'd0; by2_b[rk]<=12'd0;
                        end
                    end else begin
                        bx1_b[box_sel]<=pb_x1; bx2_b[box_sel]<=pb_x2;
                        by1_b[box_sel]<=pb_y1; by2_b[box_sel]<=pb_y2;
                        // 频闪版：每帧按投影结果重算（无尺寸限制）
                        bok_b[box_sel]<=pb_gotx && pb_goty;
                    end
                    
                    // ★ D00 锁定后才进入 D01
                    // box_sel 的切换统一由锁定 FSM 块驱动（见 lock 块）
                    pb_ok_d <= pb_gotx && pb_goty;
                    // ★ 迟滞：变化 > PB_HYST 才更新；第一次找到时直接采用
                    if (pb_gotx && pb_goty) begin
                        if (pb_ok_rise) begin
                        // 首次出现：直接采用并把历史填满当前值
                        pb_avg_fill <= 2'd0;
                        pb_x1 <= pb_x1n; pb_x2 <= pb_x2n; pb_y1 <= s_y1[13:2]; pb_y2 <= s_y2[13:2];
                        pb_x1h1<=pb_x1n; pb_x1h2<=pb_x1n; pb_x1h3<=pb_x1n;
                        pb_x2h1<=pb_x2n; pb_x2h2<=pb_x2n; pb_x2h3<=pb_x2n;
                        pb_y1h1<=pb_y1n; pb_y1h2<=pb_y1n; pb_y1h3<=pb_y1n;
                        pb_y2h1<=pb_y2n; pb_y2h2<=pb_y2n; pb_y2h3<=pb_y2n;
                    end else if (pb_gotx && pb_goty) begin
                        if (pb_avg_ok) begin
                            // 4 帧平均（精确 >>2）
                            pb_x1 <= s_x1[13:2]; pb_x2 <= s_x2[13:2];   // ★ 顺序扫描（每框连续多帧）-> 恢复 4 帧平滑
                            pb_y1 <= pb_y1n; pb_y2 <= pb_y2n;
                        end else begin
                            // 历史未满：直接用原始候选
                            pb_x1 <= pb_x1n; pb_x2 <= pb_x2n;
                            pb_y1 <= pb_y1n; pb_y2 <= pb_y2n;
                            if (pb_avg_fill != 2'd3) pb_avg_fill <= pb_avg_fill + 2'd1;
                        end
                        // 历史滑动
                        pb_x1h3<=pb_x1h2; pb_x1h2<=pb_x1h1; pb_x1h1<=pb_x1n;
                        pb_x2h3<=pb_x2h2; pb_x2h2<=pb_x2h1; pb_x2h1<=pb_x2n;
                        pb_y1h3<=pb_y1h2; pb_y1h2<=pb_y1h1; pb_y1h1<=pb_y1n;
                        pb_y2h3<=pb_y2h2; pb_y2h2<=pb_y2h1; pb_y2h1<=pb_y2n;
                    end
                    end
                    if (pb_gotx && pb_goty) begin
                    end               // 列和行都找到峰才算有效
                    for (pi = 0; pi < 76; pi = pi + 1)  pb_col[pi] <= 6'd0;
                    for (pi = 0; pi < 76; pi = pi + 1)  pc7[pi] <= 7'd0;
                    for (pj = 0; pj < 112; pj = pj + 1) pb_row[pj] <= 6'd0;
                end
                // ---- ★ 旁挂：投影熵分桶累加（只读 pb_col/pb_row，不动它们）----
                if (pb_scan_col) begin
                    if (pb_i <= PBW) begin
                        esum_c <= esum_c + {7'd0, pb_col[pb_i[5:0]]};
                        if (pb_col[pb_i[5:0]] != 7'd0) cov_c <= cov_c + 7'd1;
                        if ({1'b0, pb_col[pb_i[5:0]]} > cmax) cmax <= {1'b0, pb_col[pb_i[5:0]]};
                        if ({1'b0,pc7[pb_i[6:0]]} > cmax7) cmax7 <= {1'b0,pc7[pb_i[6:0]]};
                        if ({1'b0,pc7[pb_i[6:0]]} < cmin7) cmin7 <= {1'b0,pc7[pb_i[6:0]]};
                        if ({1'b0, pb_col[pb_i[5:0]]} < cmin) cmin <= {1'b0, pb_col[pb_i[5:0]]};   // ★ 该列有墨
                        if      (pb_col[pb_i[5:0]] >= 7'd32) en6 <= en6 + 7'd1;
                        else if (pb_col[pb_i[5:0]] >= 7'd16) en5 <= en5 + 7'd1;
                        else if (pb_col[pb_i[5:0]] >= 7'd8 ) en4 <= en4 + 7'd1;
                        else if (pb_col[pb_i[5:0]] >= 7'd4 ) en3 <= en3 + 7'd1;
                        else if (pb_col[pb_i[5:0]] >= 7'd2 ) en2 <= en2 + 7'd1;
                    end
                end else if (pb_ridx <= PBH) begin
                        esum_r <= esum_r + {7'd0, pb_row[pb_ridx[6:0]]};
if (pb_row[pb_ridx[6:0]] != 7'd0) cov_r <= cov_r + 7'd1;
                        if ({1'b0, pb_row[pb_ridx[6:0]]} > rmax) rmax <= {1'b0, pb_row[pb_ridx[6:0]]};
                        if ({1'b0, pb_row[pb_ridx[6:0]]} < rmin) rmin <= {1'b0, pb_row[pb_ridx[6:0]]};
                        if      (pb_row[pb_ridx[6:0]] >= 7'd32) er6 <= er6 + 7'd1;
                        else if (pb_row[pb_ridx[6:0]] >= 7'd16) er5 <= er5 + 7'd1;
                        else if (pb_row[pb_ridx[6:0]] >= 7'd8 ) er4 <= er4 + 7'd1;
                        else if (pb_row[pb_ridx[6:0]] >= 7'd4 ) er3 <= er3 + 7'd1;
                        else if (pb_row[pb_ridx[6:0]] >= 7'd2 ) er2 <= er2 + 7'd1;
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            S_window_active_1d <= 1'b0;
            S_window_active_2d <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            S_window_active_1d <= S_window_active;
            S_window_active_2d <= S_window_active_1d;
            O_hdmi_de        <= S_video_de_2d;
            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 4) begin
                // Full-screen MIPI receive diagnostic.  The status is
                // generated and synchronized in design_top_wrapper.
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'd0: O_hdmi_data <= 24'h000080; // blue: sensor config incomplete
                        4'd1: O_hdmi_data <= 24'h000000; // black: no MIPI HS traffic
                        4'd2: O_hdmi_data <= 24'h00ffff; // cyan: HS present, no complete frame
                        4'd3: O_hdmi_data <= 24'hff0000; // red: D-PHY lane error
                        4'd4: O_hdmi_data <= 24'hff00ff; // magenta: RAW10 line length error
                        4'd5: O_hdmi_data <= 24'hffff00; // yellow: wrong lines per frame (expects 1080)
                        4'd6: O_hdmi_data <= 24'h00ff00; // green: MIPI frame structure passed
                        4'd7: O_hdmi_data <= 24'hff8000; // orange: sensor I2C transaction error
                        default: O_hdmi_data <= 24'hffffff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                // 优先级：SW1(普通视频+logo) > logo > SW3 二值化 > 测试框 > 运动框 > 牌位红线 > 绿框 > OSD > 视频
                if (I_fdimg_en) begin
                    // ★ SW1：普通视频输出 + 叠加安路 logo（最高优先级）
                    if (logo_hit) O_hdmi_data <= logo_rgb;
                    else          O_hdmi_data <= (S_window_active_2d ? S_video_data_2d : 24'd0);
                end
                else if (I_logo_hit)  O_hdmi_data <= I_logo_color;
                // ★ SW3：把当前画面按亮度二值化（暗图案 = 白，亮背景 = 黑）
                //   亮度近似 = (R + 2G + B) / 4；阈值 BIN_THR 可调
                else if (I_bin_en && S_video_de_2d)
                    O_hdmi_data <= bin_dark ? 24'hffffff : 24'h000000;
                else if (tbox_hit && S_video_de_2d) O_hdmi_data <= 24'hffffff;  // ★ 测试框白色
                else if (I_fd_osd_en && mbox_hit && S_video_de_2d) O_hdmi_data <= MBOX_COL;   // ★ 帧差方框：只在 SW3 帧差模式下显示
            else if (sbox_hit && S_video_de_2d) O_hdmi_data <= SBOX_COL;  // ★ 切分后选中的那张（黄，识别已移除=不画）
                else if (!I_fd_osd_en && tbox13_hit && S_video_de_2d) O_hdmi_data <= DIV_COL;  // ★ 13 张牌红框
        else if (!I_fd_osd_en && smbox_hit  && S_video_de_2d) O_hdmi_data <= DIV_COL;  // ★ 框13（小框）红框
                else if (!I_fd_osd_en && div_hit && S_video_de_2d)  O_hdmi_data <= DIV_COL;   // ★ 牌位固定红线分割
                // ★ 复位期间立即消隐（不等扫描结束那一拍），保证"一复位框就没"
                        else if (!I_fd_osd_en && box_any_hit && !clr_boxes_w && S_video_de_2d) O_hdmi_data <= 24'h00ff00; // ★ 14 个框的绿框
                else if (osd_hit && S_video_de_2d)  O_hdmi_data <= OSD_COLOR;
                else                  O_hdmi_data <= S_window_active_2d ? S_video_data_2d : 24'd0;
            end
        end
    end

endmodule

// ============================================================================
//  以下为内联的 16x16 点阵字库（原 osd_char_lib.v，避免改 TD 的 .al 文件列表）
// ============================================================================
module osd_char_lib (
    input wire [7:0] I_char,
    input wire [3:0] I_row,
    output reg [15:0] O_row_bits
);

    localparam CHAR_AN    = 8'h80;
    localparam CHAR_LU    = 8'h81;
    localparam CHAR_A     = 8'h41;
    localparam CHAR_B     = 8'h42;
    localparam CHAR_C     = 8'h43;
    localparam CHAR_D     = 8'h44;
    localparam CHAR_E     = 8'h45;
    localparam CHAR_F     = 8'h46;
    localparam CHAR_G     = 8'h47;
    localparam CHAR_H     = 8'h48;
    localparam CHAR_I     = 8'h49;
    localparam CHAR_J     = 8'h4a;
    localparam CHAR_K     = 8'h4b;
    localparam CHAR_L     = 8'h4c;
    localparam CHAR_M     = 8'h4d;
    localparam CHAR_N     = 8'h4e;
    localparam CHAR_O     = 8'h4f;
    localparam CHAR_P     = 8'h50;
    localparam CHAR_Q     = 8'h51;
    localparam CHAR_R     = 8'h52;
    localparam CHAR_S     = 8'h53;
    localparam CHAR_T     = 8'h54;
    localparam CHAR_U     = 8'h55;
    localparam CHAR_V     = 8'h56;
    localparam CHAR_W     = 8'h57;
    localparam CHAR_X     = 8'h58;
    localparam CHAR_Y     = 8'h59;
    localparam CHAR_Z     = 8'h5a;
    localparam CHAR_0     = 8'h30;
    localparam CHAR_1     = 8'h31;
    localparam CHAR_2     = 8'h32;
    localparam CHAR_3     = 8'h33;
    localparam CHAR_4     = 8'h34;
    localparam CHAR_5     = 8'h35;
    localparam CHAR_6     = 8'h36;
    localparam CHAR_7     = 8'h37;
    localparam CHAR_8     = 8'h38;
    localparam CHAR_9     = 8'h39;
    localparam CHAR_COLON = 8'h3a;

    always @(*) begin
        case(I_char)
            CHAR_AN: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0200;
                    4'd1: O_row_bits = 16'h7ff0;
                    4'd2: O_row_bits = 16'h4410;
                    4'd3: O_row_bits = 16'h0400;
                    4'd4: O_row_bits = 16'h7ff0;
                    4'd5: O_row_bits = 16'h0840;
                    4'd6: O_row_bits = 16'h1040;
                    4'd7: O_row_bits = 16'h1c80;
                    4'd8: O_row_bits = 16'h0300;
                    4'd9: O_row_bits = 16'h0cc0;
                    4'd10: O_row_bits = 16'h7020;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_LU: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h79e0;
                    4'd1: O_row_bits = 16'h4a20;
                    4'd2: O_row_bits = 16'h4d40;
                    4'd3: O_row_bits = 16'h7880;
                    4'd4: O_row_bits = 16'h1140;
                    4'd5: O_row_bits = 16'h1630;
                    4'd6: O_row_bits = 16'h5be0;
                    4'd7: O_row_bits = 16'h5220;
                    4'd8: O_row_bits = 16'h5220;
                    4'd9: O_row_bits = 16'h5be0;
                    4'd10: O_row_bits = 16'h6220;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_A: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1c00;
                    4'd3: O_row_bits = 16'h1400;
                    4'd4: O_row_bits = 16'h1400;
                    4'd5: O_row_bits = 16'h1400;
                    4'd6: O_row_bits = 16'h3600;
                    4'd7: O_row_bits = 16'h3e00;
                    4'd8: O_row_bits = 16'h2200;
                    4'd9: O_row_bits = 16'h2200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_B: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3e00;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3300;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_C: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h03f0;
                    4'd3: O_row_bits = 16'h0c0c;
                    4'd4: O_row_bits = 16'h1800;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h1800;
                    4'd11: O_row_bits = 16'h0c0c;
                    4'd12: O_row_bits = 16'h03f0;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_D: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_E: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3e00;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_F: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3e00;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_G: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1c00;
                    4'd3: O_row_bits = 16'h3200;
                    4'd4: O_row_bits = 16'h6000;
                    4'd5: O_row_bits = 16'h6000;
                    4'd6: O_row_bits = 16'h6e00;
                    4'd7: O_row_bits = 16'h6600;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h1e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_H: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3300;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3f00;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3300;
                    4'd9: O_row_bits = 16'h3300;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_I: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h0c00;
                    4'd4: O_row_bits = 16'h0c00;
                    4'd5: O_row_bits = 16'h0c00;
                    4'd6: O_row_bits = 16'h0c00;
                    4'd7: O_row_bits = 16'h0c00;
                    4'd8: O_row_bits = 16'h0c00;
                    4'd9: O_row_bits = 16'h1e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_J: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h0600;
                    4'd4: O_row_bits = 16'h0600;
                    4'd5: O_row_bits = 16'h0600;
                    4'd6: O_row_bits = 16'h0600;
                    4'd7: O_row_bits = 16'h0600;
                    4'd8: O_row_bits = 16'h2600;
                    4'd9: O_row_bits = 16'h1c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_K: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3300;
                    4'd3: O_row_bits = 16'h3600;
                    4'd4: O_row_bits = 16'h3400;
                    4'd5: O_row_bits = 16'h3c00;
                    4'd6: O_row_bits = 16'h3c00;
                    4'd7: O_row_bits = 16'h3400;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3300;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_L: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3000;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_M: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3600;
                    4'd3: O_row_bits = 16'h3600;
                    4'd4: O_row_bits = 16'h3600;
                    4'd5: O_row_bits = 16'h3600;
                    4'd6: O_row_bits = 16'h2a00;
                    4'd7: O_row_bits = 16'h2a00;
                    4'd8: O_row_bits = 16'h2200;
                    4'd9: O_row_bits = 16'h2200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_N: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h380c;
                    4'd4: O_row_bits = 16'h3c0c;
                    4'd5: O_row_bits = 16'h360c;
                    4'd6: O_row_bits = 16'h330c;
                    4'd7: O_row_bits = 16'h318c;
                    4'd8: O_row_bits = 16'h30cc;
                    4'd9: O_row_bits = 16'h306c;
                    4'd10: O_row_bits = 16'h303c;
                    4'd11: O_row_bits = 16'h301c;
                    4'd12: O_row_bits = 16'h300c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_O: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h3200;
                    4'd4: O_row_bits = 16'h6300;
                    4'd5: O_row_bits = 16'h6300;
                    4'd6: O_row_bits = 16'h6300;
                    4'd7: O_row_bits = 16'h6300;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_P: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3e00;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Q: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h3200;
                    4'd4: O_row_bits = 16'h6300;
                    4'd5: O_row_bits = 16'h6300;
                    4'd6: O_row_bits = 16'h6300;
                    4'd7: O_row_bits = 16'h6300;
                    4'd8: O_row_bits = 16'h7600;
                    4'd9: O_row_bits = 16'h3c00;
                    4'd10: O_row_bits = 16'h0c00;
                    4'd11: O_row_bits = 16'h0700;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_R: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3c00;
                    4'd7: O_row_bits = 16'h3600;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_S: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h1e00;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3c00;
                    4'd6: O_row_bits = 16'h0f00;
                    4'd7: O_row_bits = 16'h0300;
                    4'd8: O_row_bits = 16'h0300;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_T: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3ffc;
                    4'd3: O_row_bits = 16'h03c0;
                    4'd4: O_row_bits = 16'h03c0;
                    4'd5: O_row_bits = 16'h03c0;
                    4'd6: O_row_bits = 16'h03c0;
                    4'd7: O_row_bits = 16'h03c0;
                    4'd8: O_row_bits = 16'h03c0;
                    4'd9: O_row_bits = 16'h03c0;
                    4'd10: O_row_bits = 16'h03c0;
                    4'd11: O_row_bits = 16'h03c0;
                    4'd12: O_row_bits = 16'h03c0;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_U: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3300;
                    4'd3: O_row_bits = 16'h3300;
                    4'd4: O_row_bits = 16'h3300;
                    4'd5: O_row_bits = 16'h3300;
                    4'd6: O_row_bits = 16'h3300;
                    4'd7: O_row_bits = 16'h3300;
                    4'd8: O_row_bits = 16'h3300;
                    4'd9: O_row_bits = 16'h1e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_V: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h2200;
                    4'd3: O_row_bits = 16'h2200;
                    4'd4: O_row_bits = 16'h3600;
                    4'd5: O_row_bits = 16'h3600;
                    4'd6: O_row_bits = 16'h1400;
                    4'd7: O_row_bits = 16'h1400;
                    4'd8: O_row_bits = 16'h1c00;
                    4'd9: O_row_bits = 16'h1c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_W: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h2200;
                    4'd3: O_row_bits = 16'h2200;
                    4'd4: O_row_bits = 16'h2a00;
                    4'd5: O_row_bits = 16'h2a00;
                    4'd6: O_row_bits = 16'h3600;
                    4'd7: O_row_bits = 16'h3600;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3600;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_X: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h1c00;
                    4'd4: O_row_bits = 16'h1c00;
                    4'd5: O_row_bits = 16'h0800;
                    4'd6: O_row_bits = 16'h1c00;
                    4'd7: O_row_bits = 16'h1400;
                    4'd8: O_row_bits = 16'h3600;
                    4'd9: O_row_bits = 16'h3200;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Y: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3180;
                    4'd3: O_row_bits = 16'h1300;
                    4'd4: O_row_bits = 16'h1b00;
                    4'd5: O_row_bits = 16'h1e00;
                    4'd6: O_row_bits = 16'h0c00;
                    4'd7: O_row_bits = 16'h0c00;
                    4'd8: O_row_bits = 16'h0c00;
                    4'd9: O_row_bits = 16'h0c00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Z: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h3e00;
                    4'd3: O_row_bits = 16'h0400;
                    4'd4: O_row_bits = 16'h0c00;
                    4'd5: O_row_bits = 16'h0800;
                    4'd6: O_row_bits = 16'h0800;
                    4'd7: O_row_bits = 16'h1800;
                    4'd8: O_row_bits = 16'h1000;
                    4'd9: O_row_bits = 16'h3e00;
                    4'd10: O_row_bits = 16'h0000;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_0: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_1: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h000c;
                    4'd2: O_row_bits = 16'h000c;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_2: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_3: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_4: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_5: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_6: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_7: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_8: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_9: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_COLON: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h0000;
                    4'd3: O_row_bits = 16'h0000;
                    4'd4: O_row_bits = 16'h0000;
                    4'd5: O_row_bits = 16'h0000;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0000;
                    4'd9: O_row_bits = 16'h0000;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            default: O_row_bits = 16'h0000;
        endcase
    end

endmodule

