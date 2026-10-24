`timescale 1ns / 1ps
// ============================================================================
//  tb_ccl.v — bin_ccl 自检
//  图案：两行两列共 4 块，其中
//        (A) 行 2..6 列 4..9    （左，独立连通域）
//        (B) 行 2..6 列 20..27  （右，独立连通域）
//        每行 32 列 = 8 拍（4 像素/拍），行数 10
//  期望：O_roi 输出 2 个对象，包围盒分别 4..9 / 2..6 与 20..27 / 2..6
//  同时验证跨行（同一物体占多行 -> 多标签 -> 求根合并成一个）
// ============================================================================
module tb_ccl;
    localparam integer W  = 32;
    localparam integer H  = 10;
    localparam integer BE = W/4;          // 8 拍/行

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [3:0] bin;
    reg        valid, last, user;

    wire        busy;
    wire [3:0]  obj_cnt;
    wire [191:0] ox1, ox2, oy1, oy2;

    bin_ccl #(
        .MAX_RUN(32), .MAX_LABEL(256), .MAX_OBJ(16),
        .MIN_W(3), .MIN_H(2)              // 小图案用小门限
    ) dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_bin(bin), .I_valid(valid), .I_last(last), .I_user(user),
        .O_busy(busy), .O_obj_cnt(obj_cnt),
        .O_obj_x1(ox1), .O_obj_x2(ox2), .O_obj_y1(oy1), .O_obj_y2(oy2)
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

    // 像素开关：(行,列) 是否属于某个块
    function px_on;
        input [11:0] r, c;
        begin
            px_on = ((r >= 2) && (r <= 6) && (c >= 4)  && (c <= 9)) ||
                    ((r >= 2) && (r <= 6) && (c >= 20) && (c <= 27));
        end
    endfunction

    integer r, b, k;
    reg [3:0] w4;

    initial begin
        bin = 0; valid = 0; last = 0; user = 0;
        #20 rst_n = 1;
        #20;

        // 送 2 帧（第 1 帧建立，第 2 帧触发帧末处理并输出）
        for (k = 0; k < 2; k = k + 1) begin
            for (r = 0; r < H; r = r + 1) begin
                for (b = 0; b < BE; b = b + 1) begin
                    w4[0] = px_on(r[11:0], (b*4+0));
                    w4[1] = px_on(r[11:0], (b*4+1));
                    w4[2] = px_on(r[11:0], (b*4+2));
                    w4[3] = px_on(r[11:0], (b*4+3));
                    bin   <= w4;
                    valid <= 1'b1;
                    last  <= (b == BE-1);
                    user  <= (r == 0) && (b == 0);
                    @(posedge clk);
                end
            end
            // 帧间空档（真实 1080p 有约 22500 拍消隐，这里给足让帧末状态机跑完）
            valid <= 1'b0; last <= 1'b0; user <= 1'b0;
            repeat (2000) @(posedge clk);
        end
        repeat (50) @(posedge clk);

        $display("");
        $display("--- 对象数 = %0d（期望 2）", obj_cnt);
        for (k = 0; k < 4; k = k + 1)
            $display("--- 对象%0d: x=%0d..%0d  y=%0d..%0d",
                     k, ox1[k*12 +: 12], ox2[k*12 +: 12], oy1[k*12 +: 12], oy2[k*12 +: 12]);
        $display("");
        chk(obj_cnt == 4'd2, "A 识别到 2 个连通域");
        chk((ox1[0*12 +: 12] == 12'd4)  && (ox2[0*12 +: 12] == 12'd9)  &&
            (oy1[0*12 +: 12] == 12'd2)  && (oy2[0*12 +: 12] == 12'd6),
            "B 对象0 = 列 4..9 行 2..6");
        chk((ox1[1*12 +: 12] == 12'd20) && (ox2[1*12 +: 12] == 12'd27) &&
            (oy1[1*12 +: 12] == 12'd2)  && (oy2[1*12 +: 12] == 12'd6),
            "C 对象1 = 列 20..27 行 2..6");
        $display("");
        if (fails == 0) $display("*** CCL ALL CHECKS PASSED ***");
        else            $display("*** CCL FAILS %0d ***", fails);
        $finish;
    end
endmodule
