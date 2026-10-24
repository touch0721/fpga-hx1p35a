`timescale 1ns / 1ps

/*
 * SC500CS 曝光/增益在线配置表（每次 AE 请求写入一次）。
 *
 * 寄存器语义依据 SC500CS 数据手册 V1.8：
 *   表 2-2 曝光 = {0x3e20[3:0], 0x3e00[7:0], 0x3e01[7:0], 0x3e02[7:4]}，单位 1/2 行，
 *         最小 1，最大 2*VTS-10；本工程 VTS=1000，上限 1990，20 bit 字段足够，
 *         故 0x3e20[3:0] 无需写入（保持复位默认 0）。
 *   表 2-3 模拟增益 = 0x3e08(ANA GAIN) + 0x3e09(ANA FINE GAIN)
 *         数字增益 = 0x3e06(DIG GAIN)  + 0x3e07(DIG FINE GAIN)
 *   表 2-4 0x3e08 粗调合法值 0x03/0x07/0x0f/0x1f；0x3e09 细调合法值 0x20~0x3f
 *   表 2-5 0x3e06=0x00、0x3e07=0x80 即数字增益 1.000x
 * 0x3e03(AGC 使能) 已在初始化表中写入 0x0b，此处不再重复。
 *
 * 与安路官方 SC500CS 原厂 AE 表（uics500_cfg/uics500regAE.v，5 项）保持一致：
 * 只写曝光 + 模拟增益，不再使用未文档化的 0x3812。
 * 手册表 2-6 文档化的 group hold 地址是 0x3800/0x3817，原厂 AE 表并未使用
 * 打包机制（曝光/增益写完第 N+2 帧生效，见手册 2.2.2），故此表保持直写。
 * 额外补写数字增益 0x3e06/0x3e07 的 1.000x 项，确保数字增益确定为单位增益。
 */
module uics520regAE (
    input  wire [8:0]  REG_INDEX,
    output reg  [23:0] REG_DATA,
    output wire [8:0]  REG_SIZE,
    input  wire [15:0] I_AE,
    input  wire [15:0] I_AG
);
    assign REG_SIZE = 9'd7;

    always @(*) begin
        case (REG_INDEX)
            9'd0: REG_DATA = {16'h3e00, 4'h0, I_AE[15:12]}; // 曝光 [19:12]
            9'd1: REG_DATA = {16'h3e01, I_AE[11:4]};        // 曝光 [11:4]
            9'd2: REG_DATA = {16'h3e02, I_AE[3:0], 4'h0};   // 曝光 [3:0] -> 0x3e02[7:4]
            9'd3: REG_DATA = {16'h3e08, I_AG[15:8]};        // ANA GAIN
            9'd4: REG_DATA = {16'h3e09, I_AG[7:0]};         // ANA FINE GAIN
            9'd5: REG_DATA = {16'h3e06, 8'h00};             // DIG GAIN = 1.000x
            9'd6: REG_DATA = {16'h3e07, 8'h80};             // DIG FINE GAIN = 1.000x
            default: REG_DATA = {16'h0000, 8'h00};
        endcase
    end
endmodule
