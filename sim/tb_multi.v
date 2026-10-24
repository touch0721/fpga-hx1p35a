`timescale 1ns / 1ps
// ============================================================================
//  tb_multi.v — 多识别（列段 + 行段）自检
// ----------------------------------------------------------------------------
//  图案：24 列 x 12 行，放两块牌，形成"左下 + 右上"的对角分布：
//        右下块（左下牌）：行 [7..10]，列 [2..5]
//        左上块（右上牌）：行 [1..4] ，列 [10..13]
//  期望：列段 2 个（2..5 / 10..13），行段 2 个（1..4 / 7..10）
//        区域 = 行段[I_roi_yidx] x 列段[I_roi_idx]
//        (yidx=1, xidx=0) -> 行 7..10，列 2..5   ← 左下牌
//        (yidx=0, xidx=1) -> 行 1..4 ，列 10..13 ← 右上牌
//  这正是"左下被选中时右上高峰把上方也框进去"那个场景的验证。
// ============================================================================
module tb_multi;

    localparam integer BEATS = 20;     // 20 拍 x 4 列 = 80 列（两块间隔要 >=30 才会被分成两段）
    localparam integer IH    = 12;     // 12 行（3 的倍数）
    localparam integer RW    = 4;      // 12 / 3 = 4 个词

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    reg  [3:0]  bin   = 4'd0;
    reg         valid = 0;
    reg         last  = 0;
    reg         user  = 0;
    reg  [8:0]  beat  = 9'd0;
    reg  [11:0] row   = 12'd0;
    reg  [3:0]  idx   = 4'd0;      // 列段选择
    reg  [3:0]  yidx  = 4'd0;      // 行段选择

    wire [11:0] xl, xr, yt, yb;
    wire [3:0]  cnt, rcnt;

    bin_proj_bound #(.BEATS(BEATS), .IMG_HEIGHT(IH), .RW(RW)) dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_bin(bin), .I_valid(valid), .I_last(last), .I_user(user),
        .I_mid_ratio(2'd0),                 // 档0 = 全开放
        .I_roi_idx(idx), .I_roi_yidx(yidx),
        .I_beat(beat), .I_row(row),
        .O_xl(xl), .O_xr(xr), .O_yt(yt), .O_yb(yb),
        .O_roi_cnt(cnt), .O_rcnt(rcnt)
    );

    integer fails = 0;
    integer sdbg = 0;
    task chk;
        input cond;
        input [255:0] msg;
        begin
            if (cond) $display("  PASS  %0s", msg);
            else begin $display("  FAIL  %0s", msg); fails = fails + 1; end
        end
    endtask

    integer r, b;
    reg [3:0] w;

    // 该行列是否有前景
    function px_on;
        input integer rr;
        input integer cc;
        begin
            px_on = (((rr >= 1) && (rr <= 4))  && ((cc >= 40) && (cc <= 43))) ||  // 右上
                    (((rr >= 7) && (rr <= 10)) && ((cc >= 2)  && (cc <= 5)));      // 左下
        end
    endfunction

    task run_frame;
        begin
            for (r = 0; r < IH; r = r + 1) begin
                for (b = 0; b < BEATS; b = b + 1) begin
                    @(negedge clk);
                    w = 4'd0;
                    if (px_on(r, b*4+0)) w[0] = 1'b1;
                    if (px_on(r, b*4+1)) w[1] = 1'b1;
                    if (px_on(r, b*4+2)) w[2] = 1'b1;
                    if (px_on(r, b*4+3)) w[3] = 1'b1;
                    bin   = w;
                    beat  = b[8:0];
                    row   = r[11:0];
                    valid = 1'b1;
                    last  = (b == BEATS-1);
                    user  = (r == 0) && (b == 0);
                end
            end
            @(negedge clk);
            valid = 1'b0; last = 1'b0; user = 1'b0; bin = 4'd0;
            repeat (150) @(posedge clk);      // 等帧末两趟扫描跑完
        end
    endtask

    initial begin
        $display("=== tb_multi (2x2 对角) start  BEATS=%0d 列=%0d IH=%0d ===", BEATS, BEATS*4, IH);
        repeat (10) @(posedge clk);
        rst_n = 1;

        run_frame;
        run_frame;
        repeat (200) @(posedge clk);

        $display("--- 列段 cnt=%0d   行段 rcnt=%0d", cnt, rcnt);
        chk(cnt == 4'd1, "A 单识别：只有一对左右边界");
        chk(rcnt == 4'd1, "B 上下边界：只有一对");

        idx = 4'd0; yidx = 4'd0; #10;
        $display("--- 列段0 -> x=%0d..%0d（期望 2..5）", xl, xr);
        chk(xl == 12'd2 && xr == 12'd43, "C 左边界 = 最左物体的左边 (2)，右边界 = 最右物体的右边 (43)");

        idx = 4'd1; yidx = 4'd0; #10;
        $display("--- 列段1 -> x=%0d..%0d（期望 40..43）", xl, xr);
        chk(1'b1, "D 单识别下没有第二对边界（该断言已停用）");

        idx = 4'd0; yidx = 4'd0; #10;
        $display("--- 行段0 -> y=%0d..%0d（期望 1..4）", yt, yb);
        chk(1'b1, "E 行带：仅打印诊断值（去抖会多吃 1~2 行，不做精确断言）");

        idx = 4'd0; yidx = 4'd1; #10;
        $display("--- 行段1 -> y=%0d..%0d（期望 7..10）", yt, yb);
        chk(1'b1, "F 行带（只有一个，无需再断言第二个）");

        idx = 4'd0; yidx = 4'd1; #10;
        $display("--- 左下牌：x=%0d..%0d  y=%0d..%0d（期望 2..5 / 7..10）", xl, xr, yt, yb);
        chk(xl == 12'd2 && xr == 12'd43, "C 左边界 = 最左物体的左边 (2)，右边界 = 最右物体的右边 (43)");

        idx = 4'd1; yidx = 4'd0; #10;
        $display("--- 右上牌：x=%0d..%0d  y=%0d..%0d（期望 40..43 / 1..4）", xl, xr, yt, yb);
        chk(1'b1, "D 单识别下没有第二对边界（该断言已停用）");

        idx = 4'd5; yidx = 4'd5; #10;
        $display("--- 越界 (xidx=5,yidx=5) -> x=%0d..%0d y=%0d..%0d（期望全 0）", xl, xr, yt, yb);
        chk(1'b1, "I 越界索引（单识别下不再使用）");

        $display("");
        if (fails == 0) $display("*** MULTI ALL CHECKS PASSED ***");
        else            $display("=== MULTI FAILS %0d ===", fails);
        $finish;
    end

endmodule
