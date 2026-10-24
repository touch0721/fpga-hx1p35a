`timescale 1ns / 1ps
// ============================================================================
//  tb_row.v — 专查"上下边界"：模拟 200 行画面（ROW_LO=8, ROW_HI=192）
//  图案：行 60..140、列 2..5 与 40..43 各一块（每行 8 个前景像素）
//  期望：O_yt = 60, O_yb = 140（波峰 = 8，第一次出现行 60，最后一次行 140）
//  同时打印逐行的 row_sum 与判定，定位为什么找不到
// ============================================================================
module tb_row;
    localparam integer W  = 80;      // 20 拍/行（4 像素/拍）
    localparam integer H  = 200;
    localparam integer BE = W/4;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [3:0] bin;
    reg        valid, last, user;

    wire [11:0] xl, xr, yt, yb;
    wire [3:0]  cnt, rcnt;
    wire [11:0] oh;

    sobel_edge_128 #(
        .IMG_WIDTH(320), .IMG_HEIGHT(200)
    ) dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_tdata(128'd0), .I_tvalid(1'b0), .I_tlast(1'b0), .I_tuser(1'b0),
        .I_y_thresh(8'd128), .I_g_thresh(4'd2), .I_dbg_sel(2'b00),
        .I_bnd_en(1'b1), .I_bnd_ratio(3'd0), .I_mid_ratio(2'd0),
        .O_tdata(), .O_tvalid(), .O_tlast(), .O_tuser(),
        .O_roi_x1(), .O_roi_x2(), .O_roi_y1(), .O_roi_y2(), .O_roi_valid(),
        .I_roi_idx(4'd0), .I_roi_yidx(4'd0),
        .O_roi_cnt(cnt), .O_rcnt(rcnt), .O_roi_h(oh)
    );

    // 直接把二值流打给内部的 bin_proj_bound（绕过整个 ISP 链，专测行/列边界）
    //  —— 用一个独立的 bin_proj_bound 实例，参数与 1080p 一致
    wire [11:0] p_xl, p_xr, p_yt, p_yb;
    wire [3:0]  p_cnt, p_rcnt;
    reg  [8:0]  beat;
    reg  [11:0] rw;

    bin_proj_bound #(
        .BEATS(W/4), .IMG_HEIGHT(H), .RW((H+2)/3),
        .ROW_LO(8), .ROW_HI(192)
    ) u_proj (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_bin(bin), .I_valid(valid), .I_last(last), .I_user(user),
        .I_mid_ratio(2'd0), .I_beat(beat), .I_row(rw),
        .I_roi_idx(4'd0), .I_roi_yidx(4'd0),
        .O_roi_cnt(p_cnt), .O_rcnt(p_rcnt),
        .O_xl(p_xl), .O_xr(p_xr), .O_yt(p_yt), .O_yb(p_yb),
        .O_row(), .O_rowpeak(),
        .O_x1_all(), .O_x2_all()
    );

    function px_on;
        input [11:0] r, c;
        begin
            px_on = ((r >= 60) && (r <= 140)) &&
                    (((c >= 2) && (c <= 5)) || ((c >= 40) && (c <= 43)));
        end
    endfunction

    integer r, b, k, f;
    reg [3:0] w4;

    initial begin
        bin = 0; valid = 0; last = 0; user = 0; beat = 0; rw = 0;
        #20 rst_n = 1; #20;

        for (k = 0; k < 2; k = k + 1) begin
            for (r = 0; r < H; r = r + 1) begin
                for (b = 0; b < BE; b = b + 1) begin
                    w4[0] = px_on(r[11:0], b*4+0);
                    w4[1] = px_on(r[11:0], b*4+1);
                    w4[2] = px_on(r[11:0], b*4+2);
                    w4[3] = px_on(r[11:0], b*4+3);
                    bin   <= w4;
                    valid <= 1'b1;
                    last  <= (b == BE-1);
                    user  <= (r == 0) && (b == 0);
                    beat  <= b[8:0];
                    rw    <= r[11:0];
                    @(posedge clk);
                end
            end
            valid <= 1'b0; last <= 1'b0; user <= 1'b0;
            repeat (3000) @(posedge clk);
        end
        repeat (200) @(posedge clk);

        $display("");
        $display("=== bin_proj_bound 输出 ===");
        $display("  左右：cnt=%0d  x=%0d..%0d", p_cnt, p_xl, p_xr);
        $display("  上下：rcnt=%0d y=%0d..%0d   (期望 60..140)", p_rcnt, p_yt, p_yb);
        $display("  sobel 例化的 ROI：cnt=%0d rcnt=%0d  y1=%0d y2=%0d h=%0d", cnt, rcnt, yt, yb, oh);
        $display("");
        f = 0;
        if (p_yt != 12'd60)  begin $display("  FAIL  上边界 = %0d（期望 60）", p_yt); f = f + 1; end
        else $display("  PASS  上边界 = 60");
        if (p_yb != 12'd140) begin $display("  FAIL  下边界 = %0d（期望 140）", p_yb); f = f + 1; end
        else $display("  PASS  下边界 = 140");
        if (p_rcnt == 4'd1)  $display("  PASS  rcnt = 1");
        else begin $display("  FAIL  rcnt = %0d（期望 1）", p_rcnt); f = f + 1; end
        if (f == 0) $display("*** ROW ALL CHECKS PASSED ***");
        else        $display("*** ROW FAILS %0d ***", f);
        $finish;
    end
endmodule
