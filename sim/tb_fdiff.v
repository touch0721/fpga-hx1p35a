`timescale 1ns / 1ps
// ============================================================================
//  tb_fdiff.v — 邻域投票 + 开运算 自检
//  尺寸：IMG_W=128(32拍/行) IMG_H=64 -> BC=16, BR=8
//
//  测试对象（块坐标 -> 像素 = 块×8）：
//    A : 块列 2..5、块行 1..4  (4×4 块 = 32×32 像素)   -> 开运算后应保留
//    B : 块列 10..13、块行 3..6 (4×4 块)               -> 开运算后应保留
//    噪点: 块列 8、块行 0  (1×1 块)                     -> ★ 开运算应删掉
//
//  期望：
//    第2帧(只有A)           -> 1 个框
//    第3帧(A + B + 孤立噪点) -> ★★ 2 个框（噪点被开运算去掉）
//    第4帧(回背景)           -> 0 个框
// ============================================================================
module tb_fdiff;
    localparam integer IW = 128;
    localparam integer IH = 64;
    localparam integer BCp = IW/8;      // 16
    localparam integer BRp = IH/8;      // 8
    localparam integer BEATS = IW/4;    // 32

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [31:0] ynow;
    reg         vld, user, bg_cap, en;
    wire [7:0]  bv;
    wire [95:0] bx1, bx2, by1, by2;
    wire [15:0] fcnt;
    wire [11:0] mcnt;
    wire [4:0]  ocnt;
    wire        bg_rdy;

    frame_diff_onchip #(
        .IMG_W(IW), .IMG_H(IH), .BC(BCp), .BR(BRp),
        .DIFF_THR(8'd12), .MIN_HITS(2), .OPEN_EN(1)
    ) dut (
        .I_pix_clk(clk), .I_rst_n(rst_n),
        .I_pix_valid(vld), .I_pix_last(1'b0), .I_pix_user(user),
        .I_y_now(ynow), .I_en(en), .I_bg_cap(bg_cap),
        .O_bv(bv),
        .O_bx1(bx1), .O_bx2(bx2), .O_by1(by1), .O_by2(by2),
        .O_frame_cnt(fcnt), .O_mot_cnt(mcnt), .O_obj_cnt(ocnt), .O_bg_rdy(bg_rdy)
    );

    integer ly, bp, k, px, i;
    reg [7:0] yv;
    reg [31:0] w32;
    reg wv, wu;

    // fsel: 0=纯背景  1=只有A  2=A+B+噪点
    task send_frame;
        input integer fsel;
        begin
            for (ly = 0; ly < IH; ly = ly + 1) begin
                for (bp = 0; bp < BEATS; bp = bp + 1) begin
                    w32 = 32'd0;
                    for (k = 0; k < 4; k = k + 1) begin
                        px = bp*4 + k;
                        yv = 8'd100;
                        if (fsel >= 1) begin
                            // A: 块列 2..5 (px 16..47)、块行 1..4 (行 8..39)
                            if ((px >= 16) && (px <= 47) && (ly >= 8) && (ly <= 39)) yv = 8'd200;
                        end
                        if (fsel >= 2) begin
                            // B: 块列 10..13 (px 80..111)、块行 3..6 (行 24..55)
                            if ((px >= 80) && (px <= 111) && (ly >= 24) && (ly <= 55)) yv = 8'd200;
                            // 孤立噪点: 块列 8 (px 64..71)、块行 0 (行 0..7)
                            if ((px >= 64) && (px <= 71) && (ly <= 7)) yv = 8'd200;
                        end
                        w32[k*8 +: 8] = yv;
                    end
                    wv = 1'b1; wu = (ly==0)&&(bp==0);
                    ynow = w32; vld = wv; user = wu;
                    @(posedge clk);
                end
            end
            vld = 0; user = 0;
            repeat (4000) @(posedge clk);
        end
    endtask


    // ★ 检查任意两个有效框不得重叠（重叠应已合并）
    task check_no_overlap;
        input [127:0] tag;
        integer a, b2;
        reg bad;
        begin
            bad = 0;
            for (a = 0; a < 4; a = a + 1)
                for (b2 = a+1; b2 < 4; b2 = b2 + 1)
                    if (bv[a] && bv[b2]) begin
                        if ((bx1[a*12+:12] <= bx2[b2*12+:12]) && (bx2[a*12+:12] >= bx1[b2*12+:12]) &&
                            (by1[a*12+:12] <= by2[b2*12+:12]) && (by2[a*12+:12] >= by1[b2*12+:12])) begin
                            bad = 1;
                            $display("     !! 框%0d 与 框%0d 重叠: [%0d..%0d,%0d..%0d] vs [%0d..%0d,%0d..%0d]",
                                     a, b2, bx1[a*12+:12], bx2[a*12+:12], by1[a*12+:12], by2[a*12+:12],
                                     bx1[b2*12+:12], bx2[b2*12+:12], by1[b2*12+:12], by2[b2*12+:12]);
                        end
                    end
            if (bad) begin
                $display("  FAIL 存在重叠的框（应合并为一个）");
                fails = fails + 1;
            end else begin
                $display("  PASS 有效框之间无重叠 ✓");
            end
        end
    endtask

    integer fails;
    initial begin
        fails = 0;
        ynow = 0; vld = 0; user = 0; en = 1; bg_cap = 1;
        #20 rst_n = 1; #20;

        send_frame(0);
        $display("  第1帧(建背景) 帧=%0d bg_rdy=%b", fcnt, bg_rdy);
        if (bg_rdy !== 1'b1) begin $display("  FAIL 背景未建立"); fails = fails + 1; end
        else $display("  PASS 背景已建立");

        bg_cap = 0; repeat (10) @(posedge clk);

        // 第 2 帧：只有 A（4×4 块，能过开运算）
        send_frame(1);
        $display("  第2帧(只有A) obj=%0d M=%0d", ocnt, mcnt);
        for (i = 0; i < 4; i = i + 1)
            if (bv[i]) $display("     框%0d: x=%0d..%0d y=%0d..%0d",
                                i, bx1[i*12+:12], bx2[i*12+:12], by1[i*12+:12], by2[i*12+:12]);
        if (ocnt !== 5'd1) begin $display("  FAIL 期望 1 个框，实际 %0d", ocnt); fails = fails + 1; end
        else $display("  PASS 1 个框（4x4 块通过开运算）");

        // 第 3 帧：A + B + 孤立噪点
        send_frame(2);
        $display("  第3帧(A+B+噪点) obj=%0d M=%0d", ocnt, mcnt);
        for (i = 0; i < 4; i = i + 1)
            if (bv[i]) $display("     框%0d: x=%0d..%0d y=%0d..%0d",
                                i, bx1[i*12+:12], bx2[i*12+:12], by1[i*12+:12], by2[i*12+:12]);
        if (ocnt !== 5'd2) begin
            $display("  FAIL 期望 2 个框（噪点应被开运算去掉），实际 %0d", ocnt);
            fails = fails + 1;
        end else $display("  PASS 2 个框，孤立噪点已被开运算删除");
        check_no_overlap("A+B");

        // 第 4~7 帧：回背景（★ 输出框有 HOLD_FRM=3 的保持计数，要多发几帧才熄灭）
        send_frame(0);
        $display("  第4帧(回背景,保持期) obj=%0d bv=%b", ocnt, bv);
        send_frame(0); send_frame(0); send_frame(0); send_frame(0);
        send_frame(0); send_frame(0); send_frame(0); send_frame(0);
        send_frame(0); send_frame(0); send_frame(0); send_frame(0);
        $display("  第16帧(回背景,保持期已过) obj=%0d bv=%b", ocnt, bv);
        if (bv !== 8'b00000000) begin $display("  FAIL 回背景后不该有框"); fails = fails + 1; end
        else $display("  PASS 回背景 -> 无框");

        $display("");
        if (fails == 0) $display("*** FDIF ALL CHECKS PASSED ***");
        else            $display("*** FDIF FAILS %0d ***", fails);
        $finish;
    end
endmodule
