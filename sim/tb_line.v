`timescale 1ns / 1ps
// ============================================================================
//  tb_line.v — 验证"彩色竖线是否画在左右边界上"
//  直接探测 sobel_edge_128 内部的 lh0..lh3（彩线命中）/ xl_c / xr_c（左右边界）
//  图案：64 列 × 20 行，列 8..55 全白（每行 48 个白像素）
//  期望：xl=8, xr=55；H = 填充行数
//        W = 0.703*H；彩线在 xl+W, xl+2W, ... 且离边界 >=8
// ============================================================================
module tb_line;
    localparam integer W  = 64;
    localparam integer H  = 20;
    localparam integer BE = W/4;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [3:0] bin;
    reg        valid, last, user;
    reg  [8:0] beat;
    reg  [11:0] rw;
    reg  [7:0]  thr;
    reg         k4;

    wire [11:0] xl, xr, yt, yb, oh;
    wire [4:0]  lcnt;
    wire [191:0] lxs;

    sobel_edge_128 #(
        .IMG_WIDTH(128), .IMG_HEIGHT(64)
    ) dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_tdata(128'd0), .I_tvalid(1'b0), .I_tlast(1'b0), .I_tuser(1'b0),
        .I_y_thresh(8'd128), .I_g_thresh(4'd2), .I_dbg_sel(2'b00),
        .I_bnd_en(1'b1), .I_bnd_ratio(thr), .I_mid_ratio(2'd0),
        .I_k4_en(k4),
        .O_tdata(), .O_tvalid(), .O_tlast(), .O_tuser(),
        .O_roi_x1(xl), .O_roi_x2(xr), .O_roi_y1(yt), .O_roi_y2(yb), .O_roi_valid(),
        .I_roi_idx(4'd0), .I_roi_yidx(4'd0),
        .O_roi_cnt(), .O_rcnt(), .O_roi_h(oh),
        .O_line_cnt(lcnt), .O_line_x(lxs), .O_fill_h()
    );

    function px_on;
        input [11:0] r, c;
        begin px_on = (c >= 12'd8) && (c <= 12'd55); end
    endfunction

    integer r, b, k, f, gi;
    reg [3:0] w4;
    reg [11:0] lxk;

    initial begin
        bin=0; valid=0; last=0; user=0; beat=0; rw=0; thr=8'd0; k4=1'b1;
        #20 rst_n = 1; #20;

        for (k = 0; k < 4; k = k + 1) begin
            for (r = 0; r < H; r = r + 1) begin
                for (b = 0; b < BE; b = b + 1) begin
                    w4[0]=px_on(r[11:0], b*4+0); w4[1]=px_on(r[11:0], b*4+1);
                    w4[2]=px_on(r[11:0], b*4+2); w4[3]=px_on(r[11:0], b*4+3);
                    bin <= w4; valid <= 1'b1; last <= (b == BE-1);
                    user <= (r==0) && (b==0);
                    beat <= b[8:0]; rw <= r[11:0];
                    @(posedge clk);
                end
            end
            valid <= 1'b0; last <= 1'b0; user <= 1'b0;
            repeat (1500) @(posedge clk);
        end
        repeat (200) @(posedge clk);

        $display("");
        $display("=== 结果 ===");
        $display("  左右边界: xl=%0d  xr=%0d", xl, xr);
        $display("  KEY4 高度(填充行数最大值) = %0d", dut.proj_fill_h);
        $display("  W = %0d（3/4 时=%0d，0.703 时=%0d）", dut.band_w, (dut.proj_fill_h*3)/4, (dut.proj_fill_h*703)/1000);
        $display("  彩线条数 = %0d", lcnt);
        $display("  彩线坐标:");
        for (gi = 0; gi < 16; gi = gi + 1) begin
            lxk = lxs[gi*12 +: 12];
            if (gi < lcnt)
                $display("     第%0d条 : x=%0d   （距 xl %0d，距 xr %0d）",
                         gi, lxk, lxk-xl, xr-lxk);
        end
        $display("");
        f = 0;
        for (gi = 0; gi < 16; gi = gi + 1) begin
            lxk = lxs[gi*12 +: 12];
            if (gi < lcnt) begin
                if (lxk == xl) begin $display("  FAIL  第%0d条 落在左边界 xl 上！", gi); f=f+1; end
                if (lxk == xr) begin $display("  FAIL  第%0d条 落在右边界 xr 上！", gi); f=f+1; end
                if ((lxk - xl) < 12'd8) begin $display("  FAIL  第%0d条 距左边界 <8", gi); f=f+1; end
                if ((xr - lxk) < 12'd8) begin $display("  FAIL  第%0d条 距右边界 <8", gi); f=f+1; end
            end
        end
        if (f == 0) $display("*** LINE OK：彩线都不在左右边界上，且离边界 >=8 ***");
        else        $display("*** LINE FAILS %0d ***", f);
        $finish;
    end
endmodule
