`timescale 1ns / 1ps
// tb_fscan.v — 最小诊断：只关心 s_vend / s_st / s_lb / s_line
module tb_fscan;
    localparam integer IW = 64, IH = 16;
    localparam integer BCp = IW/8, BRp = IH/8;
    localparam integer BEATS = IW/4;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [31:0] ynow;
    reg         vld, last, user, en;
    wire        box_v;
    wire [11:0] bx1, bx2, by1, by2;

    frame_diff_onchip #(.IMG_W(IW), .IMG_H(IH), .BC(BCp), .BR(BRp), .DIFF_THR(8'd12)) dut (
        .I_pix_clk(clk), .I_rst_n(rst_n),
        .I_pix_valid(vld), .I_pix_last(last), .I_pix_user(user),
        .I_y_now(ynow), .I_en(en),
        .O_box_valid(box_v), .O_box_x1(bx1), .O_box_x2(bx2), .O_box_y1(by1), .O_box_y2(by2)
    );

    integer ly, bp, k, px, cyc;
    reg [7:0] yv;
    reg [31:0] w32;
    reg wv, wu, wl;

    // ★ 监视扫描过程：CMAX 期间把 i 和取到的桶值打出来（每帧都打，不用全局计数限制）
    always @(posedge clk) begin
        if (rst_n && en && (dut.s_st == 3'd1) && (dut.s_i <= 10)) begin
            $display("   CMAX t=%0t i=%0d col[i]=%0d cmax=%0d", $time, dut.s_i, dut.s_col[dut.s_i], dut.s_cmax);
        end
    end

    task send_frame;
        input integer fsel;
        begin
            for (ly = 0; ly < IH; ly = ly + 1) begin
                for (bp = 0; bp < BEATS; bp = bp + 1) begin
                    w32 = 32'd0;
                    for (k = 0; k < 4; k = k + 1) begin
                        px = bp*4 + k;
                        yv = 8'd100;
                        if (fsel == 1)
                            if ((px >= 16) && (px <= 31) && (ly <= 7)) yv = 8'd200;
                        w32[k*8 +: 8] = yv;
                    end
                    wv = 1'b1; wu = (ly==0)&&(bp==0); wl = (ly==IH-1)&&(bp==BEATS-1);
                    // ★ 全部用阻塞赋值：先把数据和 valid 摆好，再等时钟沿（DUT 在该沿采样）
                    ynow = w32; vld = wv; user = wu; last = wl;
                    @(posedge clk);
                end
            end
            vld = 0; last = 0; user = 0;
            // 只等 1200 拍，然后把关键信号打出来
            for (cyc = 0; cyc < 1200; cyc = cyc + 1) begin
                if (cyc < 6 || (cyc % 200 == 0))
                    $display("   cyc=%0d st=%0d lb=%0d line=%0d vend=%b i=%0d cmax=%0d col0..3=%0d %0d %0d %0d",
                             cyc, dut.s_st, dut.s_lb, dut.s_line, dut.s_vend, dut.s_i,
                             dut.s_cmax, dut.s_col[0], dut.s_col[1], dut.s_col[2], dut.s_col[3]);
                @(posedge clk);
            end
        end
    endtask

    initial begin
        ynow = 0; vld = 0; last = 0; user = 0; en = 1;
        #20 rst_n = 1; #20;
        $display("=== LB_MAX 应该是 %0d（= IW/4-1 = %0d）===", dut.LB_MAX, IW/4-1);
        send_frame(0);
        $display("=== 帧1 结束状态: st=%0d cmax=%0d rmax=%0d ===", dut.s_st, dut.s_cmax, dut.s_rmax);
        send_frame(0);
        send_frame(1);
        $display("=== 帧3 结束: box_v=%b x=%0d..%0d 列桶=%0d %0d %0d %0d cmax=%0d ===",
                 box_v, bx1, bx2, dut.s_col[0],dut.s_col[1],dut.s_col[2],dut.s_col[3], dut.s_cmax);
        $finish;
    end
endmodule
