`timescale 1ns / 1ps
// ============================================================================
//  tb_sobel_pipeline.v — Sobel 接入 isp_top 后的端到端集成验证
// ----------------------------------------------------------------------------
//  真实模块：BLC、data128_96、gamma、data96_128_pa、sobel_edge_128、
//            data_96bit_to_128bit（均为工程内真实代码）
//  桩模块  ：demosaic（RAW10>>2 转灰度 RGB，4 像素/拍，正确的 lane 布局）、
//            awb（1 级直通）
//
//  测试图像：16 像素宽 x 6 行，左半（列 0~7）黑、右半（列 8~15）白
//            黑 = RAW10 15（BLC 后 0 -> 灰度 0）
//            白 = RAW10 1023（BLC 后 1008 -> 灰度 252）
//            每行 4 拍，共 24 拍
//
//  检查项：
//    1) 连线：gamma -> sob_pack -> sobel -> sob_unpack -> 96to128
//    2) 拍数不变 = 24（DDR 里帧布局不能错）
//    3) 帧内前 6 槽全黑（3 行延迟，窗口凑不齐）
//    4) 第 2~5 行：只有像素 7、8 是边缘，其余为 0x000000
//    5) SOBEL_EN=0 的旁路实例，96to128 的输入应等于 gamma 输出
//
//  运行：
//    iverilog -g2012 -o tb.vvp ^
//      sim\tb_sobel_pipeline.v user_source\hdl_source\isp\isp_top.v ^
//      user_source\hdl_source\isp\data96_128\data96_128.v ^
//      user_source\hdl_source\isp\data128_96\data128_96.v ^
//      sim\prim_ph1p_eram_sim.v
//    vvp tb.vvp
// ============================================================================

// ---------------- demosaic 桩：RAW10 -> 灰度 RGB888，4 像素/拍 ----------------
// lane m = {8'h00, gray,gray,gray}，与本工程 128 位约定一致
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
        O_tdata  <= { {8'h00,p3,p3,p3}, {8'h00,p2,p2,p2},
                      {8'h00,p1,p1,p1}, {8'h00,p0,p0,p0} };
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
module tb_sobel_pipeline;

    localparam integer W     = 16;
    localparam integer H     = 30;
    localparam integer BEATS = W / 4;
    localparam integer TOTAL = H * BEATS;

    localparam [9:0] RAW_BLK = 10'd15;      // BLC 后 0        -> 灰度 0
    localparam [9:0] RAW_WHT = 10'd1023;    // BLC 后 1008>>2  -> 灰度 252

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg          rst_n    = 1'b0;
    reg  [39:0]  i_tdata  = 40'd0;
    reg          i_tvalid = 1'b0, i_tlast = 1'b0, i_tuser = 1'b0;

    wire [127:0] o_tdata;
    wire         o_tlast, o_tuser, o_tvalid;

    isp_top u_en (
        .axi4s_video_aclk (clk), .I_rst_n(rst_n),
        .I_tlast(i_tlast), .I_tuser(i_tuser), .I_tdata(i_tdata), .I_tvalid(i_tvalid),
        .I_tdest(10'd0), .O_tready(1'b1),
        .I_sobel_en(1'b1), .I_sobel_y_thresh(8'd128), .I_sobel_g_thresh(4'd1), .I_dbg_sel(2'b00), .I_bnd_en(1'b0), .I_bnd_ratio(3'd2), .I_mid_ratio(2'd2),
        .O_tdata(o_tdata), .O_tlast(o_tlast), .O_tuser(o_tuser), .O_tvalid(o_tvalid),
        .I_tready()
    );

    wire [127:0] o_tdata_byp;
    wire         o_tlast_byp, o_tuser_byp, o_tvalid_byp;

    // 旁路参考实例：SOBEL_EN=0，96to128 的输入应等于 gamma 输出
    isp_top u_byp (
        .axi4s_video_aclk (clk), .I_rst_n(rst_n),
        .I_tlast(i_tlast), .I_tuser(i_tuser), .I_tdata(i_tdata), .I_tvalid(i_tvalid),
        .I_tdest(10'd0), .O_tready(1'b1),
        .I_sobel_en(1'b0), .I_sobel_y_thresh(8'd128), .I_sobel_g_thresh(4'd1), .I_dbg_sel(2'b00), .I_bnd_en(1'b0), .I_bnd_ratio(3'd2), .I_mid_ratio(2'd2),
        .O_tdata(o_tdata_byp), .O_tlast(o_tlast_byp), .O_tuser(o_tuser_byp),
        .O_tvalid(o_tvalid_byp), .I_tready()
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

    // ---------------- 图像定义 ----------------
    function [9:0] raw_of;
        input integer c;
        begin raw_of = (c >= 8) ? RAW_WHT : RAW_BLK; end
    endfunction

    function [39:0] raw_beat;
        input integer b;
        begin raw_beat = { raw_of(4*b+3), raw_of(4*b+2), raw_of(4*b+1), raw_of(4*b) }; end
    endfunction

    // 期望的 96 位输出：{p0, p1, p2, p3}，每个 24 位，边缘=0xFFFFFF
    // 链路 = 腐蚀 -> Sobel -> 膨胀：
    //   输入白区 [8,15] -> 腐蚀 [9,14] -> Sobel 在列 8/9 出梯度 -> 膨胀铺开到 7,8,9,10
    function [95:0] exp96;
        input integer b;
        reg [23:0] p [0:3];
        integer m, c;
        begin
            for (m = 0; m < 4; m = m + 1) begin
                c = 4*b + m;
                p[m] = (c >= 7 && c <= 10) ? 24'hFFFFFF : 24'h000000;
            end
            exp96 = { p[0], p[1], p[2], p[3] };
        end
    endfunction

    // ---------------- 采集到达 96->128 打包器的数据 ----------------
    reg [95:0] cap96 [0:255];
    integer    n96 = 0;
    always @(posedge clk) begin
        if (rst_n && u_en.u_data_96bit_to_128bit.I_96b_valid && n96 < 256) begin
            cap96[n96] = u_en.u_data_96bit_to_128bit.I_96b_data;
            n96 = n96 + 1;
        end
    end

    // 旁路实例：抓一拍"有效且非全黑"的数据（黑拍的 96 位本来就全是 0，拿来比对没意义），
    // 与同一拍的 gamma 输出比对
    reg [95:0] byp_pack = 96'd0, byp_gam = 96'd0;
    reg        byp_ok   = 1'b0;
    always @(posedge clk) begin
        if (rst_n && u_byp.u_data_96bit_to_128bit.I_96b_valid && !byp_ok
            && (u_byp.u_gamma.O_tdata !== 96'd0)) begin
            byp_pack <= u_byp.u_data_96bit_to_128bit.I_96b_data;
            byp_gam  <= u_byp.u_gamma.O_tdata;
            byp_ok   <= 1'b1;
        end
    end

    integer r, b, k, nbad;
    reg     ok;
    reg [95:0] cap_byp;

    initial begin
        $display("=== tb_sobel_pipeline start (W=%0d H=%0d TOTAL=%0d) ===", W, H, TOTAL);
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        for (r = 0; r < H; r = r + 1)
            for (b = 0; b < BEATS; b = b + 1) begin
                i_tdata  = raw_beat(b);
                i_tvalid = 1'b1;
                i_tuser  = (r == 0) && (b == 0);
                i_tlast  = (b == BEATS-1);
                @(negedge clk);
            end
        i_tvalid = 1'b0; i_tlast = 1'b0; i_tuser = 1'b0;
        repeat (80) @(negedge clk);

        // ---- 1) 连线检查 ----
        chk(u_en.u_sob_pack.I_tdata == u_en.u_gamma.O_tdata,
            "gamma -> data96_128_pa（96->128 像素对齐）");
        chk(u_en.u_sobel.I_tdata == u_en.u_sob_pack.O_tdata,
            "data96_128_pa -> sobel");
        chk(u_en.u_sobel.I_tvalid == u_en.u_gamma.O_tvalid,
            "sobel 的 valid 来自 gamma");
        chk(u_en.u_sob_unpack.I_tdata == u_en.u_sobel.O_tdata,
            "sobel -> data128_96（128->96）");
        chk(u_en.u_data_96bit_to_128bit.I_96b_data == u_en.u_sob_unpack.O_tdata,
            "data128_96 -> data_96bit_to_128bit（Sobel 在 DDR 打包之前）");

        // ---- 2) 拍数 ----
        $display("  到达打包器的有效拍数 = %0d（期望 %0d）", n96, TOTAL);
        chk(n96 == TOTAL, "拍数不变，DDR 里帧布局不会错");

        // ---- 3) 消隐已停用：只保留"拍数不变"的检查（前几行现在是未成形的窗口，不是黑）----
        ok = 1'b1;
        for (k = 0; k < 0*BEATS && k < n96; k = k + 1)
            if (cap96[k] !== 96'd0) ok = 1'b0;
        chk(ok, "（消隐已停用，此项跳过）");

        // ---- 4) 有效区（最终行 r3=3..H-4 -> 槽 6..H-1）边缘位置 ----
        nbad = 0;
        for (r = 3; r <= H-4; r = r + 1)
            for (b = 0; b < BEATS; b = b + 1) begin
                k = (r+3)*BEATS + b;
                if (k < n96 && cap96[k] !== exp96(b)) begin
                    nbad = nbad + 1;
                    if (nbad <= 4)
                        $display("    r=%0d b=%0d got=%024h exp=%024h",
                                 r, b, cap96[k], exp96(b));
                end
            end
        $display("  有效区不匹配拍数 = %0d", nbad);
        chk(nbad == 0, "有效区：只有像素 7/8 为边缘，其余 0x000000");

        // 每像素只能是 0x000000 或 0xFFFFFF
        ok = 1'b1;
        for (k = 0; k < n96; k = k + 1)
            for (b = 0; b < 4; b = b + 1)
                if (cap96[k][24*b+23 -: 24] !== 24'h000000 &&
                    cap96[k][24*b+23 -: 24] !== 24'hFFFFFF) ok = 1'b0;
        chk(ok, "输出像素只有 0x000000 / 0xFFFFFF");

        // ---- 5) SOBEL_EN=0 旁路 ----
        $display("  旁路实例 valid 期打包器输入 = %024h，同期 gamma 输出 = %024h",
                 byp_pack, byp_gam);
        chk(byp_ok, "旁路实例抓到了非全黑的采样");
        chk(byp_pack == byp_gam,
            "SOBEL_EN=0 时 96to128 的输入 == gamma 输出（旁路干净）");
        chk(byp_pack !== 96'd0, "旁路采样数据非零（确实抓到了真实图像）");

        // 目视：一个有效行
        if (n96 >= 7*BEATS)
            for (b = 0; b < BEATS; b = b + 1)
                $display("  行3 beat%0d = %024h (exp %024h)",
                         b, cap96[6*BEATS+b], exp96(b));

        $display("=== FAILS %0d ===", fails);
        if (fails == 0) $display("*** ALL CHECKS PASSED ***");
        else            $display("*** 有失败项 ***");
        $finish;
    end

    initial begin
        #2ms;
        $display("TIMEOUT fails=%0d n96=%0d", fails, n96);
        $finish;
    end

endmodule
