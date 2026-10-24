`timescale 1ns / 1ps
/*
 * tb_ae_set.v — 验证板载按键映射（三个量都只递增、到顶清零）
 *
 * 按键低有效（开发板手册：按下为低电平，松开为高电平）：
 *   I_btn[0] -> KEY1 曝光 O_ae            每次 +64 半行，到 2244 后回 0
 *   I_btn[1] -> KEY2 边缘/梯度阈值 O_g_thresh  每次 +1，1..8 走完回到 0
 *   I_btn[2] -> KEY3 亮度二值化阈值 O_y_thresh 每次 +1，0..255 走完回到 0
 *   I_btn[3] -> KEY4 未使用（不应改变任何量）
 *
 * 步进速率：曝光/边缘阈值 = r_a8_tick = 8 x r_ae_tick；亮度阈值 = r_ae_tick。
 * r_ae_tick = 100000 周期 @23.809524MHz ≈ 4.2 ms。
 *
 * 运行：
 *   iverilog -g2012 -o sim\tb_ae_set.vvp sim\tb_ae_set.v user_source\hdl_source\uiisp_beta\ae_set.v
 *   vvp sim\tb_ae_set.vvp
 */
module tb_ae_set;

    // 23.809524 MHz -> 42 ns
    reg clk = 1'b0;
    always #21 clk = ~clk;

    reg        rst      = 1'b0;      // ae_set 为高有效复位
    reg [3:0]  btn      = 4'b1111;   // 全部松开（高）
    reg        cfg_done = 1'b0;
    reg        ae_done  = 1'b0;

    wire        ae_req;
    wire [15:0] ae;
    wire [15:0] ag;
    wire [3:0]  g_thr;
    wire [7:0]  y_thr;

    ae_set dut (
        .I_clk(clk), .I_rst(rst), .I_btn(btn),
        .I_ae_cfg_done(ae_done), .I_cam_cfg_done(cfg_done),
        .O_ae_req(ae_req), .O_ae(ae), .O_ag(ag),
        .O_g_thresh(g_thr), .O_y_thresh(y_thr)
    );

    integer fails = 0;
    integer n = 0;
    task chk;
        input cond;
        input [255:0] msg;
        begin
            if (cond) $display("  PASS  %0s", msg);
            else begin $display("  FAIL  %0s", msg); fails = fails + 1; end
        end
    endtask

    task wait_ticks;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge dut.r_ae_tick);
        end
    endtask

    initial begin
        $display("=== tb_ae_set start ===");
        rst = 1'b1;
        repeat (10) @(posedge clk);
        rst = 1'b0;
        cfg_done = 1'b1;
        ae_done  = 1'b1;
        repeat (10) @(posedge clk);

        // 1) 全部松开：什么都不该变
        btn = 4'b1111;
        wait_ticks(12);
        $display("idle: ae=%0d ag=%0d y=%0d g=%0d",
                 dut.r_ae_set, dut.r_ag_set, dut.r_y_set, dut.r_g_set);
        chk(dut.r_ae_set == 16'd2244, "idle: 曝光保持 2244（VTS=1127 的上限 2*VTS-10）");
        chk(dut.r_ag_set == 16'd144,  "idle: 模拟增益保持 144（15.75x，不由按键控制）");
        chk(dut.r_y_set  == 8'd128,   "idle: 亮度阈值保持 128");
        chk(dut.r_g_set  == 4'd1,     "idle: 边缘阈值保持 1");

        // 2) KEY4 未使用：按住不应有任何变化
        btn = 4'b0111;
        wait_ticks(20);
        chk(dut.r_ae_set == 16'd2244 && dut.r_y_set == 8'd128 && dut.r_g_set == 4'd1,
            "KEY4 未使用：按住 20 个 tick 三个量都不变");
        btn = 4'b1111;
        wait_ticks(2);

        // 3) KEY1 曝光：2244 已在顶 -> 第一步回 0，之后 +64 递增
        //    注意：按键同步比 r_a8_tick 的 1 拍脉冲慢 2 个周期，按下瞬间可能吃掉一个 tick，
        //    所以用"等到值变化"而不是死数 tick 个数。
        btn = 4'b1110;
        n = 0;
        while (dut.r_ae_set !== 16'd0 && n < 40) begin @(posedge dut.r_a8_tick); n = n + 1; end
        $display("KEY1 第1步: ae=%0d（期望 0，因为 2244 已在顶）", dut.r_ae_set);
        chk(dut.r_ae_set == 16'd0, "KEY1：已在顶 -> 第一步清零");
        n = 0;
        while (dut.r_ae_set !== 16'd64 && n < 40) begin @(posedge dut.r_a8_tick); n = n + 1; end
        $display("KEY1 第2步: ae=%0d（期望 64）", dut.r_ae_set);
        chk(dut.r_ae_set == 16'd64, "KEY1：+64 -> 64");
        n = 0;
        while (dut.r_ae_set !== 16'd128 && n < 40) begin @(posedge dut.r_a8_tick); n = n + 1; end
        $display("KEY1 第3步: ae=%0d（期望 128）", dut.r_ae_set);
        chk(dut.r_ae_set == 16'd128, "KEY1：再 +64 -> 128");
        btn = 4'b1111;
        wait_ticks(2);

        // 4) KEY2 边缘阈值：1..8 走完回到 0
        btn = 4'b1101;
        n = 0;
        while (dut.r_g_set !== 4'd0 && n < 40) begin @(posedge dut.r_a8_tick); n = n + 1; end
        $display("KEY2 走到顶: g=%0d（期望 0，8 到顶后清零）", dut.r_g_set);
        chk(dut.r_g_set == 4'd0, "KEY2：1..8 走完后回到 0");
        n = 0;
        while (dut.r_g_set !== 4'd1 && n < 40) begin @(posedge dut.r_a8_tick); n = n + 1; end
        $display("KEY2 下一步: g=%0d（期望 1）", dut.r_g_set);
        chk(dut.r_g_set == 4'd1, "KEY2：清零后继续 -> 1");
        btn = 4'b1111;
        wait_ticks(2);

        // 5) KEY3 亮度阈值：128 -> 255 -> 0
        btn = 4'b1011;
        n = 0;
        while (dut.r_y_set !== 8'd255 && n < 300) begin @(posedge dut.r_ae_tick); n = n + 1; end
        $display("KEY3 递增到顶: y=%0d（走了 %0d 步，期望 255）", dut.r_y_set, n);
        chk(dut.r_y_set == 8'd255, "KEY3：+1 递增到 255");
        // 到顶之后再走一步必须清零（此时按键标志已稳定，不会漏 tick）
        @(posedge dut.r_ae_tick);
        $display("KEY3 再过一步: y=%0d（期望 0，255 到顶后清零）", dut.r_y_set);
        chk(dut.r_y_set == 8'd0, "KEY3：255 到顶后清零");
        wait_ticks(3);
        $display("KEY3 再走 3 步: y=%0d（期望 3）", dut.r_y_set);
        chk(dut.r_y_set == 8'd3, "KEY3：清零后继续 -> 3");
        btn = 4'b1111;
        wait_ticks(2);

        // 6) 曝光确实被改过（后续会经 AE 表写进 SC500CS）
        chk(dut.r_ae_set != 16'd2244, "曝光已被按键改变，会经 AE 表写进 SC500CS");

        $display("=== FAILS %0d ===", fails);
        if (fails == 0) $display("*** ALL CHECKS PASSED ***");
        $finish;
    end

    initial begin
        #3000ms;
        $display("TIMEOUT fails=%0d", fails);
        $finish;
    end

endmodule
