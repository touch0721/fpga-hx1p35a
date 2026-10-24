`timescale 1ns / 1ps

/*
 * SC500CS 原厂 HD 模式初始化寄存器表：1280x720 @ 60fps。
 * 24 MHz MCLK，2-Lane MIPI RAW10，480 Mbps/Lane，Sensor 内部 VBIN + HSUB。
 *
 * 数据来源：
 *   1) 安路官方 lab_ex2_edge 工程内的 SC500CS 原厂 720p60 全表
 *      （doc/hw/fpgahw-b8c9cf-lab_ex2_edge_SC520.zip -> uics500_cfg/uics500reg_720p60.v）
 *   2) SC500CS 数据手册 V1.8（2021.9.26）表 1-2/1-3/1-9、表 2-1~2-10
 *
 * 分辨率与帧率相关寄存器与本工程原 SC520CS 表保持完全一致，未做任何改动：
 *   0x3208/0x3209 = 0x0500 -> 宽 1280
 *   0x320a/0x320b = 0x02d0 -> 高 720
 *   HTS 0x320c/0x320d = 0x0640 = 1600（480 Mbps/Lane 对应值；原 SC520CS 表未写此寄存器）
 *
 * ★ VTS 由原厂表的 1000 改成 1200（50 fps），这是本轮唯一的帧率改动，原因是工频闪烁：
 *   1 行 = HTS/PCLK = 1600/96MHz = 16.667 us。
 *   VTS=1000 -> 帧周期 16.667ms，与我国 50Hz 市电照明的光周期（20ms）不成整数倍，
 *               每 6 帧重复一次 -> 画面以 60/6 = 10Hz 整体忽明忽暗（实测确认）。
 *   VTS=1200 -> 帧周期 20.000ms = 正好一个 20ms 光周期（也是两个 10ms 光周期），
 *               每帧采到完全相同的灯光相位 -> 闪烁消除（对 100Hz 和 50Hz 两种灯都有效）。
 *   附带好处：帧变长后允许更长的曝光（上限 2*VTS-10 = 2390 半行 = 19.917ms），
 *             比原来的 1990 半行更亮。
 *   注意 PCLK 与 HTS 都没变，所以 MIPI 仍是 480 Mbps/Lane，SDC/PHY 一行都不用改。
 *
 * 序列结构与末项顺序（供 uicfgcs520 状态机识别）：
 *   索引 0                    ：软复位 0x0103=0x01，之后等待 5 ms
 *   索引 STREAM_ON_INDEX      ：stream on 0x0100=0x01，之后等待 10 ms
 *   索引 STREAM_ON_INDEX + 1  ：0x302d=0x00（原厂要求 stream on 后延时 10 ms 再写）
 */
module uics520reg_720p60 (
    input  wire [8:0]  REG_INDEX,
    output reg  [23:0] REG_DATA,
    output wire [8:0]  REG_SIZE
);
    // stream on 所在索引，uicfgcs520 依据它在写入前插入 10 ms 延时
    localparam [8:0] STREAM_ON_INDEX = 9'd115;

    assign REG_SIZE = 9'd117;

    always @(*) begin
        case (REG_INDEX)
              0: REG_DATA = {16'h0103, 8'h01}; // 软复位（数据手册表 1-3）
              1: REG_DATA = {16'h0100, 8'h00}; // 0=软睡眠 / 1=stream on（表 1-2）
              2: REG_DATA = {16'h36e9, 8'h80}; // PLL
              3: REG_DATA = {16'h36f9, 8'h80}; // PLL
              4: REG_DATA = {16'h36ea, 8'h3c};
              5: REG_DATA = {16'h36ec, 8'h1b};
              6: REG_DATA = {16'h36fd, 8'h14};
              7: REG_DATA = {16'h36e9, 8'h04}; // PLL
              8: REG_DATA = {16'h36f9, 8'h04}; // PLL
              9: REG_DATA = {16'h3016, 8'h10}; // GPIO OEN（表 2-1 Bit[4]；原厂表值）
             10: REG_DATA = {16'h3017, 8'h0e}; // 原厂表值（手册未公开）
             11: REG_DATA = {16'h301f, 8'h04};
             12: REG_DATA = {16'h302d, 8'h20}; // 原厂表值；stream on 后延时 10ms 再写 0x00
             13: REG_DATA = {16'h3106, 8'h01}; // 原厂表值（手册未公开）
             14: REG_DATA = {16'h3200, 8'h00};
             15: REG_DATA = {16'h3201, 8'h14};
             16: REG_DATA = {16'h3202, 8'h01};
             17: REG_DATA = {16'h3203, 8'h00};
             18: REG_DATA = {16'h3204, 8'h0a};
             19: REG_DATA = {16'h3205, 8'h1b};
             20: REG_DATA = {16'h3206, 8'h06};
             21: REG_DATA = {16'h3207, 8'ha7};
             22: REG_DATA = {16'h3208, 8'h05}; // 输出窗口宽度高字节 = 0x0500 -> 1280（表 2-8）
             23: REG_DATA = {16'h3209, 8'h00};
             24: REG_DATA = {16'h320a, 8'h02}; // 输出窗口高度高字节 = 0x02d0 -> 720（表 2-8）
             25: REG_DATA = {16'h320b, 8'hd0};
             26: REG_DATA = {16'h320c, 8'h06}; // HTS = 0x0640 = 1600（480 Mbps/Lane 对应的行总长）
             27: REG_DATA = {16'h320d, 8'h40};
             28: REG_DATA = {16'h320e, 8'h04}; // VTS = 0x04b0 = 1200 -> 50 fps（消工频闪，见文件头）
             29: REG_DATA = {16'h320f, 8'hb0}; // 原厂值 0x03e8=1000 -> 60 fps 会以 10Hz 闪烁
             30: REG_DATA = {16'h3210, 8'h00}; // 列起始（表 2-8）
             31: REG_DATA = {16'h3211, 8'h02};
             32: REG_DATA = {16'h3212, 8'h00}; // 行起始（表 2-8）
             33: REG_DATA = {16'h3213, 8'h02};
             34: REG_DATA = {16'h3215, 8'h31};
             35: REG_DATA = {16'h3220, 8'h01}; // VBIN + HSUB
             36: REG_DATA = {16'h3249, 8'h0f};
             37: REG_DATA = {16'h3253, 8'h06};
             38: REG_DATA = {16'h3271, 8'h13};
             39: REG_DATA = {16'h3273, 8'h13};
             40: REG_DATA = {16'h3301, 8'h0a};
             41: REG_DATA = {16'h3309, 8'h60};
             42: REG_DATA = {16'h330a, 8'h00};
             43: REG_DATA = {16'h330b, 8'he8};
             44: REG_DATA = {16'h331e, 8'h41};
             45: REG_DATA = {16'h331f, 8'h51};
             46: REG_DATA = {16'h3320, 8'h04};
             47: REG_DATA = {16'h3333, 8'h10};
             48: REG_DATA = {16'h335d, 8'h60};
             49: REG_DATA = {16'h3364, 8'h56};
             50: REG_DATA = {16'h336b, 8'h08};
             51: REG_DATA = {16'h3390, 8'h08};
             52: REG_DATA = {16'h3391, 8'h18};
             53: REG_DATA = {16'h3392, 8'h38};
             54: REG_DATA = {16'h3393, 8'h0a};
             55: REG_DATA = {16'h3394, 8'h24};
             56: REG_DATA = {16'h3395, 8'h40};
             57: REG_DATA = {16'h33ad, 8'h29};
             58: REG_DATA = {16'h341c, 8'h04};
             59: REG_DATA = {16'h341d, 8'h04};
             60: REG_DATA = {16'h341e, 8'h03};
             61: REG_DATA = {16'h3425, 8'h00};
             62: REG_DATA = {16'h3426, 8'h00};
             63: REG_DATA = {16'h3622, 8'hc7};
             64: REG_DATA = {16'h3632, 8'h44};
             65: REG_DATA = {16'h3636, 8'h6e};
             66: REG_DATA = {16'h3637, 8'h08};
             67: REG_DATA = {16'h3638, 8'h0a};
             68: REG_DATA = {16'h3651, 8'h9d};
             69: REG_DATA = {16'h3670, 8'h4b};
             70: REG_DATA = {16'h3674, 8'hc0};
             71: REG_DATA = {16'h3675, 8'h58};
             72: REG_DATA = {16'h3676, 8'h5a};
             73: REG_DATA = {16'h367c, 8'h18};
             74: REG_DATA = {16'h367d, 8'h38};
             75: REG_DATA = {16'h3690, 8'h43};
             76: REG_DATA = {16'h3691, 8'h53};
             77: REG_DATA = {16'h3692, 8'h53};
             78: REG_DATA = {16'h3699, 8'h08};
             79: REG_DATA = {16'h369a, 8'h10};
             80: REG_DATA = {16'h369b, 8'h1f};
             81: REG_DATA = {16'h369c, 8'h18};
             82: REG_DATA = {16'h369d, 8'h38};
             83: REG_DATA = {16'h36a2, 8'h08};
             84: REG_DATA = {16'h36a3, 8'h18};
             85: REG_DATA = {16'h36a6, 8'h08};
             86: REG_DATA = {16'h36a7, 8'h18};
             87: REG_DATA = {16'h36ab, 8'h40};
             88: REG_DATA = {16'h36ac, 8'h40};
             89: REG_DATA = {16'h36ad, 8'h40};
             90: REG_DATA = {16'h3901, 8'h00};
             91: REG_DATA = {16'h3904, 8'h0c};
             92: REG_DATA = {16'h3906, 8'h3a};
             93: REG_DATA = {16'h391d, 8'h14};
             94: REG_DATA = {16'h3e00, 8'h00}; // 曝光初值 = 0x0956 = 2390 = 2*VTS-10（表 2-2）
             95: REG_DATA = {16'h3e01, 8'h95}; // VTS=1200 时的最大值 = 1195 行 = 19.917ms
             96: REG_DATA = {16'h3e02, 8'h60}; // （ae_set 之后还会再写一次同样的值）
             97: REG_DATA = {16'h4000, 8'h00};
             98: REG_DATA = {16'h4001, 8'h04};
             99: REG_DATA = {16'h4002, 8'haa};
            100: REG_DATA = {16'h4003, 8'haa};
            101: REG_DATA = {16'h4004, 8'haa};
            102: REG_DATA = {16'h4005, 8'h00};
            103: REG_DATA = {16'h4006, 8'h07};
            104: REG_DATA = {16'h4007, 8'ha5};
            105: REG_DATA = {16'h4008, 8'h00};
            106: REG_DATA = {16'h4009, 8'hc8};
            107: REG_DATA = {16'h440e, 8'h02};
            108: REG_DATA = {16'h4509, 8'h28};
            109: REG_DATA = {16'h4837, 8'h42}; // MIPI 时序，对应 480 Mbps/Lane
            110: REG_DATA = {16'h5000, 8'h4e}; // 原厂表值
            111: REG_DATA = {16'h5901, 8'h04}; // 原厂表值
            112: REG_DATA = {16'h3e03, 8'h0b}; // AGC 使能：Bit[3:0]=4'hb，使 0x3e08/0x3e09/0x3e06/0x3e07 生效（表 2-3 / 2.2.3）
            113: REG_DATA = {16'h3e08, 8'h0f}; // ANA GAIN 初值 0x0f -> 4.000x（表 2-4）
            114: REG_DATA = {16'h3e09, 8'h20}; // ANA FINE GAIN 初值 0x20 -> 1.000x（表 2-4）
            115: REG_DATA = {16'h0100, 8'h01}; // 0=软睡眠 / 1=stream on（表 1-2）
            116: REG_DATA = {16'h302d, 8'h00}; // 原厂表值；stream on 后延时 10ms 再写 0x00
            default: REG_DATA = {16'h0000, 8'h00};
        endcase
    end
endmodule

// ===== BEGIN uics520reg_1080p60 (由 sim/gen_1080p60_table.py 生成，请勿手改) =====

// ============================================================================
//  SC500CS 原厂 1080p60 初始化表（1920x1080 @ 60fps、2-Lane RAW10、960 Mbps/Lane）
// ----------------------------------------------------------------------------
//  来源：厂商 SDK 04_MLK-CAM001-CS500/.../fpga_sdk/mipi_cs500/src/cfg_cs500.c
//        的 reg_cs500[] 数组，由 sim/gen_1080p60_table.py 自动解析生成，逐条一致。
//
//  关键值：
//    0x3208/0x3209 = 0x0780 -> 宽 1920
//    0x320a/0x320b = 0x0438 -> 高 1080
//    0x320c/0x320d = 0x0b24 -> HTS 2852
//    0x320e/0x320f = 0x0467 -> VTS 1127 -> 60 fps（PCLK 192 MHz）
//    0x4837        = 0x21   -> MIPI 960 Mbps/Lane（720p60 表是 0x42=480Mbps）
//    0x3e01/0x3e02         -> 曝光初值 = 2*VTS-10 = 2244（厂商原值 2246，超上限 2，已收敛到上限）
//
//  序列结构（供 uicfgcs520 状态机识别）：
//    索引 0            ：软复位 0x0103=0x01，之后等待 5 ms
//    索引 115           ：stream on 0x0100=0x01（**最后一条**，之后无写入）
//    0x302d=0x20 在前部（索引 15），0x302d=0x00 在索引 112
// ============================================================================
module uics520reg_1080p60 (
    input  wire [8:0]  REG_INDEX,
    output reg  [23:0] REG_DATA,
    output wire [8:0]  REG_SIZE
);
    // stream on 所在索引，uicfgcs520 依据它决定写完最后一条即配置完成
    localparam [8:0] STREAM_ON_INDEX = 9'd115;

    assign REG_SIZE = 9'd116;

    always @(*) begin
        case (REG_INDEX)
              0: REG_DATA = {16'h0103, 8'h01}; // 软复位
              1: REG_DATA = {16'h0100, 8'h00}; // 停流
              2: REG_DATA = {16'h36e9, 8'h80};
              3: REG_DATA = {16'h36f9, 8'h80};
              4: REG_DATA = {16'h36ea, 8'h3c};
              5: REG_DATA = {16'h36eb, 8'h0c};
              6: REG_DATA = {16'h36ec, 8'h0b};
              7: REG_DATA = {16'h36ed, 8'h26};
              8: REG_DATA = {16'h36fa, 8'h3b};
              9: REG_DATA = {16'h36fb, 8'ha6};
             10: REG_DATA = {16'h36fc, 8'h10};
             11: REG_DATA = {16'h36fd, 8'h24};
             12: REG_DATA = {16'h36e9, 8'h04};
             13: REG_DATA = {16'h36f9, 8'h04};
             14: REG_DATA = {16'h301f, 8'h50};
             15: REG_DATA = {16'h302d, 8'h20}; // 原厂序列固定值
             16: REG_DATA = {16'h3106, 8'h01};
             17: REG_DATA = {16'h3200, 8'h01};
             18: REG_DATA = {16'h3201, 8'h54};
             19: REG_DATA = {16'h3202, 8'h01};
             20: REG_DATA = {16'h3203, 8'hb4};
             21: REG_DATA = {16'h3204, 8'h08};
             22: REG_DATA = {16'h3205, 8'hdb};
             23: REG_DATA = {16'h3206, 8'h05};
             24: REG_DATA = {16'h3207, 8'hf3};
             25: REG_DATA = {16'h3208, 8'h07}; // 宽 1920
             26: REG_DATA = {16'h3209, 8'h80};
             27: REG_DATA = {16'h320a, 8'h04}; // 高 1080
             28: REG_DATA = {16'h320b, 8'h38};
             29: REG_DATA = {16'h320c, 8'h0b}; // HTS = 0x0b24 = 2852
             30: REG_DATA = {16'h320d, 8'h24};
             31: REG_DATA = {16'h320e, 8'h04}; // VTS = 0x0467 = 1127
             32: REG_DATA = {16'h320f, 8'h67};
             33: REG_DATA = {16'h3210, 8'h00};
             34: REG_DATA = {16'h3211, 8'h04};
             35: REG_DATA = {16'h3212, 8'h00};
             36: REG_DATA = {16'h3213, 8'h04};
             37: REG_DATA = {16'h3249, 8'h0f};
             38: REG_DATA = {16'h3253, 8'h06};
             39: REG_DATA = {16'h3271, 8'h13};
             40: REG_DATA = {16'h3273, 8'h13};
             41: REG_DATA = {16'h3301, 8'h0a};
             42: REG_DATA = {16'h3306, 8'h40};
             43: REG_DATA = {16'h3309, 8'h60};
             44: REG_DATA = {16'h330a, 8'h00};
             45: REG_DATA = {16'h330b, 8'hd8};
             46: REG_DATA = {16'h331e, 8'h41};
             47: REG_DATA = {16'h331f, 8'h51};
             48: REG_DATA = {16'h3320, 8'h04};
             49: REG_DATA = {16'h3333, 8'h10};
             50: REG_DATA = {16'h335d, 8'h60};
             51: REG_DATA = {16'h3364, 8'h56};
             52: REG_DATA = {16'h336b, 8'h08};
             53: REG_DATA = {16'h3390, 8'h08};
             54: REG_DATA = {16'h3391, 8'h18};
             55: REG_DATA = {16'h3392, 8'h38};
             56: REG_DATA = {16'h3393, 8'h0a};
             57: REG_DATA = {16'h3394, 8'h24};
             58: REG_DATA = {16'h3395, 8'h30};
             59: REG_DATA = {16'h33ad, 8'h29};
             60: REG_DATA = {16'h341c, 8'h04};
             61: REG_DATA = {16'h341d, 8'h04};
             62: REG_DATA = {16'h341e, 8'h03};
             63: REG_DATA = {16'h3425, 8'h00};
             64: REG_DATA = {16'h3426, 8'h00};
             65: REG_DATA = {16'h3622, 8'hc7};
             66: REG_DATA = {16'h3632, 8'h44};
             67: REG_DATA = {16'h3636, 8'h6e};
             68: REG_DATA = {16'h3637, 8'h08};
             69: REG_DATA = {16'h3638, 8'h0a};
             70: REG_DATA = {16'h3651, 8'h9d};
             71: REG_DATA = {16'h3670, 8'h4b};
             72: REG_DATA = {16'h3674, 8'hc0};
             73: REG_DATA = {16'h3675, 8'h58};
             74: REG_DATA = {16'h3676, 8'h5a};
             75: REG_DATA = {16'h367c, 8'h18};
             76: REG_DATA = {16'h367d, 8'h38};
             77: REG_DATA = {16'h3690, 8'h43};
             78: REG_DATA = {16'h3691, 8'h53};
             79: REG_DATA = {16'h3692, 8'h53};
             80: REG_DATA = {16'h3699, 8'h08};
             81: REG_DATA = {16'h369a, 8'h10};
             82: REG_DATA = {16'h369b, 8'h1f};
             83: REG_DATA = {16'h369c, 8'h18};
             84: REG_DATA = {16'h369d, 8'h38};
             85: REG_DATA = {16'h36a2, 8'h08};
             86: REG_DATA = {16'h36a3, 8'h18};
             87: REG_DATA = {16'h36a6, 8'h08};
             88: REG_DATA = {16'h36a7, 8'h18};
             89: REG_DATA = {16'h36ab, 8'h40};
             90: REG_DATA = {16'h36ac, 8'h40};
             91: REG_DATA = {16'h36ad, 8'h40};
             92: REG_DATA = {16'h3901, 8'h00};
             93: REG_DATA = {16'h3904, 8'h0c};
             94: REG_DATA = {16'h3906, 8'h3a};
             95: REG_DATA = {16'h391d, 8'h14};
             96: REG_DATA = {16'h3e01, 8'h8c}; // 曝光 = 2244 = 2*VTS-10
             97: REG_DATA = {16'h3e02, 8'h40};
             98: REG_DATA = {16'h4000, 8'h00};
             99: REG_DATA = {16'h4001, 8'h04};
            100: REG_DATA = {16'h4002, 8'hb0};
            101: REG_DATA = {16'h4003, 8'h00};
            102: REG_DATA = {16'h4004, 8'h00};
            103: REG_DATA = {16'h4005, 8'h00};
            104: REG_DATA = {16'h4006, 8'h07};
            105: REG_DATA = {16'h4007, 8'hae};
            106: REG_DATA = {16'h4008, 8'h00};
            107: REG_DATA = {16'h4009, 8'hc8};
            108: REG_DATA = {16'h440e, 8'h02};
            109: REG_DATA = {16'h4509, 8'h28};
            110: REG_DATA = {16'h4837, 8'h21}; // MIPI 960 Mbps/Lane
            111: REG_DATA = {16'h5000, 8'h0e};
            112: REG_DATA = {16'h302d, 8'h00}; // 原厂序列固定值
            113: REG_DATA = {16'h3e08, 8'h07};
            114: REG_DATA = {16'h3e09, 8'h3f};
            115: REG_DATA = {16'h0100, 8'h01}; // stream on（最后一条）
            default: REG_DATA = 24'h000000;
        endcase
    end
endmodule
// ===== END uics520reg_1080p60
