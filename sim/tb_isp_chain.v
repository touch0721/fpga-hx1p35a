`timescale 1ns / 1ps
/*
 * tb_isp_chain.v — 验证 BLC 与 gamma 已正确接入 isp_top 链路
 *
 * 数据通路（本 TB 关注的部分）：
 *   I_tdata(RAW10 x4)
 *     -> BLC（真实模块，减 15 黑电平）
 *     -> demosaic（桩：RAW10>>2 转灰度 RGB888，放入 128bit 低 96 位）
 *     -> data128_96（真实）-> awb（桩：1 级直通）-> axis_gamma_rgb96（真实）
 *     -> data_96bit_to_128bit（真实）-> O_tdata
 *
 * 断言：
 *   1) demosaic 桩的输入 == BLC 的输出        （BLC 在 demosaic 之前）
 *   2) BLC 输出 == 输入 - 15，且低到 0 不借位  （黑电平算法正确）
 *   3) gamma 的输入 == data128_96 的输出       （gamma 接在打包之后）
 *   4) 96->128 的输入 == gamma 的输出          （gamma 在打包之前）
 *   5) gamma 输出 != gamma 输入（确实做了映射）
 *
 * 运行：
 *   iverilog -g2012 -o sim\tb_isp_chain.vvp ^
 *     -y user_source\hdl_source\isp -y user_source\hdl_source\isp\data128_96 ^
 *     -y user_source\hdl_source\isp\data96_128 sim\tb_isp_chain.v
 *   vvp sim\tb_isp_chain.vvp
 */

// ---------------- demosaic 桩 ----------------
module demosaic #(
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH  = 1920,
    parameter data_complete_delay = 50,
    parameter BAYER_MODE = "BGGR"
) (
    input               I_clk,
    input               I_rst_n,
    input [39:0]        axi4s_video_tdata,
    input [9:0]         axi4s_video_tdest,
    input               axi4s_video_tlast,
    input               axi4s_video_tvalid,
    input               axi4s_video_tuser,
    output              axi4s_video_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [127:0]  O_tdata,
    output reg          O_tvalid,
    input               O_tready
);
    assign axi4s_video_tready = O_tready;
    wire [7:0] p0 = axi4s_video_tdata[9:2];
    wire [7:0] p1 = axi4s_video_tdata[19:12];
    wire [7:0] p2 = axi4s_video_tdata[29:22];
    wire [7:0] p3 = axi4s_video_tdata[39:32];
    always @(posedge I_clk) begin
        O_tdata  <= {32'd0, p3, p3, p3, p2, p2, p2, p1, p1, p1, p0, p0, p0};
        O_tlast  <= axi4s_video_tlast;
        O_tuser  <= axi4s_video_tuser;
        O_tvalid <= axi4s_video_tvalid;
    end
endmodule

// ---------------- awb 桩（真实 awb 依赖 divider IP）----------------
module awb #(
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH  = 1920
) (
    input               I_clk,
    input               I_rst_n,
    input               I_tlast,
    input               I_tuser,
    input [95:0]        I_tdata,
    input               I_tvalid,
    output              I_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg          O_tvalid,
    input               O_tready
);
    assign I_tready = O_tready;
    always @(posedge I_clk) begin
        O_tlast  <= I_tlast;
        O_tuser  <= I_tuser;
        O_tdata  <= I_tdata;
        O_tvalid <= I_tvalid;
    end
endmodule

// ---------------- 测试平台 ----------------
module tb_isp_chain;

    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50 MHz

    reg         rst_n   = 1'b0;
    reg  [39:0] i_tdata = 40'd0;
    reg         i_tvalid = 1'b0;
    reg         i_tlast  = 1'b0;
    reg         i_tuser  = 1'b0;

    wire [127:0] o_tdata;
    wire         o_tlast, o_tuser, o_tvalid, o_ready;

    assign o_ready = 1'b1;

    isp_top u_isp_top (
        .axi4s_video_aclk (clk),
        .I_rst_n          (rst_n),
        .I_tlast          (i_tlast),
        .I_tuser          (i_tuser),
        .I_tdata          (i_tdata),
        .I_tvalid         (i_tvalid),
        .I_tdest          (10'd0),
        .I_sobel_en       (1'b1),
        .I_sobel_y_thresh (8'd128),
        .I_sobel_g_thresh (4'd1),
        .I_dbg_sel        (2'b00),
        .I_bnd_en         (1'b0),
        .I_bnd_ratio      (3'd0),
        .I_mid_ratio      (2'd0),
        .O_tready         (o_ready),
        .O_tdata          (o_tdata),
        .O_tlast          (o_tlast),
        .O_tuser          (o_tuser),
        .O_tvalid         (o_tvalid),
        .I_tready         ()
    );

    integer fails = 0;
    task chk;
        input cond;
        input [255:0] msg;
        begin
            if (cond) $display("  PASS  %0s", msg);
            else begin $display("  FAIL  %0s", msg); fails = fails + 1; end
        end
    endtask

    // 4 个测试像素（RAW10）
    localparam [9:0] P0 = 10'h200;   // 512 -> BLC 497 -> >>2 = 124
    localparam [9:0] P1 = 10'h004;   // 4   -> BLC 0（下钳位）-> >>2 = 0
    localparam [9:0] P2 = 10'h0FF;   // 255 -> BLC 240 -> >>2 = 60
    localparam [9:0] P3 = 10'h3FF;   // 1023-> BLC 1008-> >>2 = 252

    wire [39:0] raw_in  = {P3, P2, P1, P0};
    wire [39:0] exp_blc = {P3 - 10'd15, P2 - 10'd15,
                           (P1 > 10'd15) ? (P1 - 10'd15) : 10'd0,
                           (P0 > 10'd15) ? (P0 - 10'd15) : 10'd0};

    integer i;

    // 在 gamma 输出有效的那个周期捕获输入/输出对，用于数值核对
    reg [95:0] g_in_cap  = 96'd0;
    reg [95:0] g_out_cap = 96'd0;
    reg        g_cap_ok  = 1'b0;
    always @(posedge clk) begin
        if (u_isp_top.u_gamma.O_tvalid) begin
            g_in_cap  <= u_isp_top.u_gamma.I_tdata;
            g_out_cap <= u_isp_top.u_gamma.O_tdata;
            g_cap_ok  <= 1'b1;
        end
    end

    initial begin
        $display("=== tb_isp_chain start ===");
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        i_tuser  = 1'b1;  i_tlast = 1'b0;  i_tdata = raw_in;  i_tvalid = 1'b1;
        @(posedge clk);
        i_tuser  = 1'b0;
        for (i = 0; i < 8; i = i + 1) begin
            i_tdata = raw_in;
            @(posedge clk);
        end
        i_tvalid = 1'b0;
        @(posedge clk);
        repeat (4) @(posedge clk);

        $display("  raw_in            = %010h", raw_in);
        $display("  u_blc.O_tdata     = %010h", u_isp_top.u_blc.O_tdata);
        $display("  demosaic 输入      = %010h", u_isp_top.u_demosaic.axi4s_video_tdata);
        $display("  data128_96.O_tdata= %024h", u_isp_top.u_data128_96.O_tdata);
        $display("  u_gamma.I_tdata   = %024h", u_isp_top.u_gamma.I_tdata);
        $display("  u_gamma.O_tdata   = %024h", u_isp_top.u_gamma.O_tdata);
        $display("  96to128.I_96b_data= %024h", u_isp_top.u_data_96bit_to_128bit.I_96b_data);

        chk(u_isp_top.u_blc.O_tdata == exp_blc,
            "BLC output == input - 15 with clamp at 0");
        chk(u_isp_top.u_demosaic.axi4s_video_tdata == u_isp_top.u_blc.O_tdata,
            "BLC drives demosaic (BLC is before demosaic)");
        chk(u_isp_top.u_gamma.I_tdata == u_isp_top.u_data128_96.O_tdata,
            "gamma input == data128_96 output");
        // Sobel 接入后，gamma 后面多了 96->128 -> sobel -> 128->96 三级，
        // 所以 96to128 的输入不再等于 gamma 输出（详见 tb_sobel_pipeline.v）
        chk(u_isp_top.u_sob_pack.I_tdata == u_isp_top.u_gamma.O_tdata,
            "gamma -> sob_pack (Sobel 接在 gamma 之后)");
        chk(u_isp_top.u_data_96bit_to_128bit.I_96b_data == u_isp_top.u_sob_unpack.O_tdata,
            "96to128 input == sobel unpack output");
        chk(u_isp_top.u_gamma.O_tdata != u_isp_top.u_gamma.I_tdata,
            "gamma actually remaps the pixels");

        $display("  --- gamma 有效周期捕获 ---");
        $display("  g_in_cap          = %024h", g_in_cap);
        $display("  g_out_cap         = %024h", g_out_cap);
        chk(g_cap_ok, "gamma produced valid output");
        chk(g_out_cap != g_in_cap, "captured gamma out != in");

        $display("=== FAILS %0d ===", fails);
        if (fails == 0) $display("ALL CHECKS PASSED");
        $finish;
    end

    initial begin
        #100us;
        $display("TIMEOUT fails=%0d", fails);
        $finish;
    end

endmodule
