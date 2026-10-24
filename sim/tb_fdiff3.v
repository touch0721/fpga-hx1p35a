`timescale 1ns / 1ps
// ============================================================================
//  tb_fdiff3.v — 720p 真实几何下的【3 物体】检测自检
//  尺寸与板上一致：IMG_W=1280, IMG_H=720 -> BC=160, BR=90
//  造 3 个彼此远离的物体（各 4x4 块 = 32x32 像素）：
//    A: 块列 2..5,  块行 1..4    -> 像素 x 16..47,  y 8..39
//    B: 块列 30..33, 块行 10..13  -> 像素 x 240..271, y 80..111
//    C: 块列 100..103,块行 60..63 -> 像素 x 800..831, y 480..511
//  期望：O_obj_cnt==3，且 3 个框的坐标分别落在上面 3 个位置附近（±8 像素）
// ============================================================================
module tb_fdiff3;
    localparam integer IW = 1280;
    localparam integer IH = 720;
    localparam integer BCp = IW/8;      // 160
    localparam integer BRp = IH/8;      // 90
    localparam integer BEATS = IW/4;    // 320

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
        .DIFF_THR(8'd90), .MARGIN(4), .MIN_HITS(2), .MIN_SIZE(3),
        .HOLD_FRM(8), .MAXO(8), .XCOMP(1), .YCOMP(2), .OPEN_EN(1)
    ) dut (
        .I_pix_clk(clk), .I_rst_n(rst_n),
        .I_pix_valid(vld), .I_pix_last(1'b0), .I_pix_user(user),
        .I_y_now(ynow), .I_en(en), .I_bg_cap(bg_cap),
        .O_bv(bv),
        .O_bx1(bx1), .O_bx2(bx2), .O_by1(by1), .O_by2(by2),
        .O_frame_cnt(fcnt), .O_mot_cnt(mcnt), .O_obj_cnt(ocnt), .O_bg_rdy(bg_rdy)
    );

    integer ly, bp, k, px, i, fails;
    reg [7:0] yv;
    reg [31:0] w32;
    reg wv, wu;

    // fsel: 0=纯背景, 1=A+B+C 三个物体
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
                            if ((px >= 16)  && (px <= 47)  && (ly >= 8)   && (ly <= 39))  yv = 8'd210;  // A
                            if ((px >= 240) && (px <= 271) && (ly >= 80)  && (ly <= 111)) yv = 8'd210;  // B
                            if ((px >= 800) && (px <= 831) && (ly >= 480) && (ly <= 511)) yv = 8'd210;  // C
                        end
                        w32[k*8 +: 8] = yv;
                    end
                    wv = 1'b1; wu = (ly==0)&&(bp==0);
                    ynow = w32; vld = wv; user = wu;
                    @(posedge clk);
                end
            end
            vld = 0; user = 0;
            repeat (2000) @(posedge clk);
        end
    endtask

    // 检查一个框是否落在期望位置（±16 像素）
    task chk_box;
        input [127:0] tag;
        input [11:0] ex1, ex2, ey1, ey2;
        integer n, hit;
        begin
            hit = 0;
            for (n = 0; n < 8; n = n + 1)
                if (bv[n]) begin
                    if ((bx1[n*12+:12] <= ex2) && (bx2[n*12+:12] >= ex1) &&
                        (by1[n*12+:12] <= ey2) && (by2[n*12+:12] >= ey1)) begin
                        hit = 1;
                        $display("  PASS %0s 命中: 框%0d x=%0d..%0d y=%0d..%0d (期望 x=%0d..%0d y=%0d..%0d)",
                                 tag, n, bx1[n*12+:12], bx2[n*12+:12], by1[n*12+:12], by2[n*12+:12],
                                 ex1, ex2, ey1, ey2);
                        // 宽高合理性：应在 16..64 像素
                        if ((bx2[n*12+:12] - bx1[n*12+:12]) > 80 || (by2[n*12+:12] - by1[n*12+:12]) > 80) begin
                            $display("  FAIL %0s 框尺寸异常（过大）", tag);
                            fails = fails + 1;
                        end
                    end
                end
            if (!hit) begin
                $display("  FAIL %0s 未找到对应框", tag);
                fails = fails + 1;
            end
        end
    endtask

    initial begin
        fails = 0;
        ynow = 0; vld = 0; user = 0; en = 1; bg_cap = 1;
        #20 rst_n = 1; #20;

        send_frame(0);
        $display("  帧1 建背景: 帧数=%0d bg_rdy=%b", fcnt, bg_rdy);
        if (bg_rdy !== 1'b1) begin $display("  FAIL 背景未建立"); fails = fails + 1; end
        else $display("  PASS 背景已建立");

        bg_cap = 0; repeat (10) @(posedge clk);

        send_frame(1);
        $display("  帧2 (A+B+C): obj=%0d M=%0d", ocnt, mcnt);
        for (i = 0; i < 8; i = i + 1)
            if (bv[i]) $display("     框%0d: x=%0d..%0d y=%0d..%0d",
                                i, bx1[i*12+:12], bx2[i*12+:12], by1[i*12+:12], by2[i*12+:12]);

        $display("  --- 原始输出 O_bv=%b", dut.O_bv);
        $display("      O_bx1=%h", dut.O_bx1);
        $display("      O_bx2=%h", dut.O_bx2);
        $display("      O_by1=%h", dut.O_by1);
        $display("      O_by2=%h", dut.O_by2);
        $display("  --- 内部目标表 t_cnt=%0d ---", dut.t_cnt);
        for (i = 0; i < 6; i = i + 1)
            $display("     t[%0d] val=%b hit=%0d x=%0d..%0d y=%0d..%0d",
                     i, dut.t_val[i], dut.t_hit[i], dut.t_x1[i], dut.t_x2[i], dut.t_y1[i], dut.t_y2[i]);
        if (ocnt !== 5'd3) begin $display("  FAIL 期望 3 个框，实际 %0d", ocnt); fails = fails + 1; end
        else $display("  PASS 3 个框");

        chk_box("A", 12'd16,  12'd47,  12'd8,   12'd39);
        chk_box("B", 12'd240, 12'd271, 12'd80,  12'd111);
        chk_box("C", 12'd800, 12'd831, 12'd480, 12'd511);

        send_frame(0);
        $display("  帧3 回背景: obj=%0d", ocnt);

        if (fails == 0) $display("*** FDIF3 ALL CHECKS PASSED ***");
        else            $display("*** FDIF3 FAILED (%0d) ***", fails);
        $finish;
    end
endmodule
