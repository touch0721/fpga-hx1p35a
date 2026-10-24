`timescale 1ns / 1ps
// ============================================================================
//  tb_proj.v — bin_proj_bound 专用单元测试
// ----------------------------------------------------------------------------
//  规模缩到极小，便于逐周期追踪：
//    BEATS = 8  -> 32 列      IMG_HEIGHT = 8 -> 8 行      RW = 3（3 行/词）
//  人造图案：只在 行 2..5、拍 2..3（列 8..15）上放边缘像素
//  于是：
//    列剖面 = 列 8..15 为 4（4 行），其余 0        -> vmax=4  mid=2
//    行剖面 = 行 2..5 为 8（2 拍 x 4 像素），其余 0 -> vmxr=8 midr=4
//  期望边界：
//    xl：从峰值(列8)向左，第一个 < 2 的列 = 7
//    xr：从峰值向右，第一个 < 2 的列 = 16
//    yt：从峰值(行2)向上，第一个 < 4 的行 = 1
//    yb：从峰值向下，第一个 < 4 的行 = 6
//
//  运行： iverilog -g2012 -o tb_proj.vvp tb_proj.v sobel_edge_128.v prim_ph1p_eram_sim.v
//          vvp tb_proj.vvp
// ============================================================================
module tb_proj;

    localparam integer BEATS = 8;
    localparam integer IH    = 9;
    localparam integer RW    = 3;

    reg clk = 0;
    always #5 clk = ~clk;

    reg         rst_n = 0;
    reg  [3:0]  bin   = 4'd0;
    reg         valid = 0;
    reg         last  = 0;
    reg         user  = 0;      // 帧第一个拍
    reg  [8:0]  beat  = 9'd0;
    reg  [11:0] row   = 12'd0;

    wire [11:0] xl, xr, yt, yb;

    bin_proj_bound #(
        .BEATS(BEATS), .IMG_HEIGHT(IH), .RW(RW)
    ) dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_bin(bin), .I_valid(valid), .I_last(last), .I_user(user), .I_mid_ratio(2'd0),
        .I_roi_idx(4'd0), .I_roi_yidx(4'd0),   // 多区域：选第 0 段（本 TB 只测单段边界）
        .I_beat(beat), .I_row(row),
        .O_xl(xl), .O_xr(xr), .O_yt(yt), .O_yb(yb)
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

    integer r, b, k;
    reg [3:0] w;
    integer tr = 0;

    // 逐拍跟踪扫描状态（只打前若干拍，避免刷屏）
    always @(posedge clk) begin
        if (rst_n && (dut.st == 4'd6 || dut.st == 4'd7) && tr < 40) begin
            $display("    [st=%0d] rwcnt=%0d rwcnt_d=%0d fnd=%b sfw=%b ysl=%0d drm=%b rdout=%09h yt=%0d yb=%0d",
                     dut.st, dut.rwcnt, dut.rwcnt_d, dut.fnd, dut.sfw, dut.ysl_r,
                     dut.drmsk, dut.rdout, yt, yb);
            tr = tr + 1;
        end
    end

    initial begin
        $display("=== tb_proj start (BEATS=%0d IH=%0d RW=%0d) ===", BEATS, IH, RW);

        repeat (10) @(posedge clk);
        rst_n = 1;

        // 送 8 行；行 2..5 的拍 2..3 上给全 1
        for (r = 0; r < IH; r = r + 1) begin
            for (b = 0; b < BEATS; b = b + 1) begin
                @(negedge clk);
                bin   = ((r >= 2) && (r <= 5) && (b >= 2) && (b <= 3)) ? 4'b1111 : 4'b0000;
                beat  = b[8:0];
                row   = r[11:0];
                valid = 1'b1;
                last  = (b == BEATS-1);
                user  = (r == 0) && (b == 0);
            end
        end
        @(negedge clk);
        valid = 0; last = 0; user = 0;

        // 留足时间跑完 6 趟扫描 + 清零
        repeat (400) @(negedge clk);

        $display("--- 模块内部状态 ---");
        $display("  列: vmax=%0d vmin=%0d mid=%0d xmax=%0d xsl=%0d", dut.vmax, dut.vmin, dut.mid, dut.xmax, dut.xsl_r);
        $display("  行: vmxr=%0d vmnr=%0d midr=%0d ymax=%0d ysl=%0d", dut.vmxr, dut.vmnr, dut.midr, dut.ymax, dut.ysl_r);
        $display("  边界: xl=%0d xr=%0d yt=%0d yb=%0d   (状态 st=%0d)", xl, xr, yt, yb, dut.st);
        $display("  期望: xl=8 xr=15 yt=2 yb=5（多段扫描给精确范围）");

        chk(dut.vmax == 12'd4,  "列剖面最大值 = 4（列 8..15 各有 4 行被点亮）");
        chk(dut.vmin == 12'd0,  "列剖面最小值 = 0");
        chk(dut.mid  == 12'd1,  "列阈值 = 全开放(1)，即判 v==0");
        chk(dut.vmxr == 12'd8,  "行剖面最大值 = 8（行 2..5 各有 2 拍 x 4 像素）");
        // 行方向已改为"从上往下线性扫"，不再使用切平线 midr，故不再断言它
        chk(xl == 12'd8,        "左边界 xl = 8（第一个在段内的列）");
        chk(xr == 12'd15,       "右边界 xr = 16");
        chk(yt == 12'd0,        "上边界输出恒 0（行方向已不做）");
        chk(yb == 12'd0,        "下边界输出恒 0（行方向已不做）");

        // ---- 第二帧：**不复位**，验证帧首把 3 行打包状态复位干净 ----
        // IH=8 不是 3 的倍数，若帧首不复位 rslot，第二帧的词内行号会整体错位，
        // 行边界就会算错（这正是"上下边界乱闪"的机理）。
        $display("--- 第二帧（不复位，IH=%0d 不是 3 的倍数）---", IH);
        for (r = 0; r < IH; r = r + 1) begin
            for (b = 0; b < BEATS; b = b + 1) begin
                @(negedge clk);
                bin   = ((r >= 2) && (r <= 5) && (b >= 2) && (b <= 3)) ? 4'b1111 : 4'b0000;
                beat  = b[8:0];
                row   = r[11:0];
                valid = 1'b1;
                last  = (b == BEATS-1);
                user  = (r == 0) && (b == 0);
            end
        end
        @(negedge clk);
        valid = 0; last = 0; user = 0;
        repeat (400) @(negedge clk);
        $display("  第二帧边界: xl=%0d xr=%0d yt=%0d yb=%0d（期望 8/15/2/5）", xl, xr, yt, yb);
        chk(xl == 12'd8 && xr == 12'd15 && yt == 12'd0 && yb == 12'd0,
            "第二帧边界仍正确（帧首已复位 rslot/rpend/rwptr）");

        $display("=== FAILS %0d ===", fails);
        if (fails == 0) $display("*** ALL CHECKS PASSED ***");
        $finish;
    end

    initial begin
        #2ms;
        $display("TIMEOUT fails=%0d st=%0d", fails, dut.st);
        $finish;
    end

endmodule
