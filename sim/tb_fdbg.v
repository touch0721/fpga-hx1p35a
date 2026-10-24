`timescale 1ns / 1ps
// tb_fdbg.v — 定位：哪些块报了运动、读到的旧值是多少
module tb_fdbg;
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

    integer ly, bp, k, px, nmot, nblk;
    reg [7:0] yv;
    reg [31:0] w32;
    reg wv, wu, wl;

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
                    ynow <= w32; vld <= wv; user <= wu; last <= wl;
                    @(posedge clk);
                end
            end
            vld <= 0; last <= 0; user <= 0;
            repeat (1500) @(posedge clk);
        end
    endtask

    // 每次运动判定都打印
    always @(posedge clk) begin
        if (rst_n && en && dut.s_mot_v) begin
            nmot = nmot + 1;
            $display("   运动#%0d: bc_d4=%0d br_d4=%0d  新值=%0d 旧值=%0d  addr_latch=%0d",
                     nmot, dut.s_bc_d4, dut.s_br_d4, dut.s_da, dut.s_db, dut.s_blk_addr_r);
        end
        if (rst_n && en && dut.s_blk_done) begin
            nblk = nblk + 1;
            if (nblk <= 16)
                $display("   块#%0d: bc=%0d br=%0d avg=%0d latch_addr=%0d",
                         nblk, dut.s_bc, dut.s_br, dut.s_avg_w, dut.s_blk_addr_r);
        end
    end

    initial begin
        nmot = 0; nblk = 0;
        ynow = 0; vld = 0; last = 0; user = 0; en = 1;
        #20 rst_n = 1; #20;

        $display("===== 第1帧 全100 =====");
        send_frame(0);
        $display("  第1帧: 块=%0d 运动=%0d", nblk, nmot);
        $display("  RAM[0..15]=%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
            dut.s_ram[0],dut.s_ram[1],dut.s_ram[2],dut.s_ram[3],dut.s_ram[4],dut.s_ram[5],
            dut.s_ram[6],dut.s_ram[7],dut.s_ram[8],dut.s_ram[9],dut.s_ram[10],dut.s_ram[11],
            dut.s_ram[12],dut.s_ram[13],dut.s_ram[14],dut.s_ram[15]);

        nmot = 0; nblk = 0;
        $display("===== 第2帧 全100（应与上帧一样，0 运动）=====");
        send_frame(0);
        $display("  第2帧: 块=%0d 运动=%0d box_v=%b", nblk, nmot, box_v);

        nmot = 0; nblk = 0;
        $display("===== 第3帧 块列2~3、行0~7 = 200 =====");
        send_frame(1);
        $display("  第3帧: 块=%0d 运动=%0d box_v=%b x=%0d..%0d", nblk, nmot, box_v, bx1, bx2);
        $display("  列桶=%0d %0d %0d %0d %0d %0d %0d %0d  cmax=%0d rmax=%0d st=%0d",
            dut.s_col[0],dut.s_col[1],dut.s_col[2],dut.s_col[3],
            dut.s_col[4],dut.s_col[5],dut.s_col[6],dut.s_col[7],
            dut.s_cmax, dut.s_rmax, dut.s_st);
        $finish;
    end
endmodule
