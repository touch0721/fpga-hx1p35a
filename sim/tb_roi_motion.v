`timescale 1ns / 1ps
// ============================================================================
//  tb_roi_motion.v — roi_motion_onchip 自检
// ----------------------------------------------------------------------------
//  小图 256x64（32x8 个 8x8 块），像素内容按"模式"生成：
//    模式 0：全亮（Y=200，空桌）
//    模式 1：主框区域画成暗图案（Y=40，牌）；小框仍全亮        -> 只有 A 报"有"
//    模式 2：主框全亮、小框画暗图案                            -> 只有 B 报"有"
//
//  侧参数：AX1=32 AX2=95（块列 4..11，8 块）、BX1=128 BX2=159（块列 16..19，4 块）、
//          CY1=16 CY2=31（块行 2..3，2 行）-> 主框 8x2=16 块、小框 4x2=8 块
//          THR_A=8、THR_B=4、HOLD=2
//
//  用例：
//    1) 模式 0 + 抓背景（I_bg_cap=1）-> O_bg_rdy=1，A/B 都不报
//    2) 模式 0 + 不抓        -> 计数 0，A/B 都不报（背景空白 => 没有变化）
//    3) 模式 1 + 不抓        -> A 计数 = 16、A_present=1；B 计数 0、B_present=0
//    4) 模式 2 + 不抓        -> B 计数 = 8、B_present=1；A 计数 0、A_present=0
//    5) 模式 0 + 不抓（连续 3 帧）-> A 计数 0，A_present 在 HOLD=2 帧后熄
// ============================================================================
module tb_roi_motion;
    localparam integer IMG_W = 256;
    localparam integer IMG_H = 64;
    localparam integer LB_MAX = (IMG_W/4) - 1;   // 63
    localparam integer LN_MAX = IMG_H - 1;       // 63

    reg clk = 0, rst_n = 0, en = 1;
    always #5 clk = ~clk;                        // 100 MHz

    reg         pix_valid = 0, pix_last = 0, pix_user = 0;
    reg  [31:0] y_now = 0;
    reg         bg_cap = 0;
    reg  [1:0]  mode = 0;

    wire        a_present, b_present, bg_rdy;
    wire [11:0] a_count, b_count;

    roi_motion_onchip #(
        .IMG_W(IMG_W), .IMG_H(IMG_H),
        .DIFF_THR(8'd40), .AVG_SH(7),
        .THR_A(8), .THR_B(4), .HOLD(2),
        .AX1(32), .AX2(95), .BX1(128), .BX2(159), .CY1(16), .CY2(31)
    ) dut (
        .I_pix_clk(clk), .I_rst_n(rst_n),
        .I_pix_valid(pix_valid), .I_pix_last(pix_last), .I_pix_user(pix_user),
        .I_y_now(y_now), .I_en(en), .I_bg_cap(bg_cap),
        .O_a_present(a_present), .O_b_present(b_present),
        .O_a_count(a_count), .O_b_count(b_count), .O_bg_rdy(bg_rdy)
    );

    integer fails = 0, oks = 0;
    task check;
        input cond;
        input [255:0] msg;
        begin
            if (cond) oks = oks + 1;
            else begin fails = fails + 1; $display("  FAIL: %0s", msg); end
        end
    endtask

    // 像素内容
    function [7:0] px;
        input integer x, y;
        begin
            px = 8'd200;                                   // 默认：亮（空桌）
            if (mode == 2'd1) begin                        // 主框有牌
                if ((x >= 32) && (x <= 95) && (y >= 16) && (y <= 31)) px = 8'd40;
            end else if (mode == 2'd2) begin               // 小框有东西
                if ((x >= 128) && (x <= 159) && (y >= 16) && (y <= 31)) px = 8'd40;
            end
        end
    endfunction

    integer l, ln, lb;
    task send_frame;
        begin
            for (ln = 0; ln <= LN_MAX; ln = ln + 1) begin
                for (lb = 0; lb <= LB_MAX; lb = lb + 1) begin
                    @(negedge clk);
                    y_now = {px(4*lb+3, ln), px(4*lb+2, ln), px(4*lb+1, ln), px(4*lb+0, ln)};
                    pix_valid = 1'b1;
                    pix_last  = (lb == LB_MAX);
                    pix_user  = (lb == 0) && (ln == 0);
                    @(posedge clk);
                end
            end
            @(negedge clk);
            pix_valid = 1'b0; pix_last = 1'b0; pix_user = 1'b0;
            for (l = 0; l < 200; l = l + 1) @(posedge clk);   // 消隐：够结算 FSM 跑完
        end
    endtask

    initial begin
        $dumpfile("tb_roi_motion.vcd");
        $dumpvars(0, tb_roi_motion);

        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        $display("=== tb_roi_motion：固定 ROI 背景帧差分 ===");

        // 1) 空桌抓背景
        mode = 2'd0; bg_cap = 1'b1;
        send_frame; #1;
        $display("  [抓背景] bg_rdy=%b a_cnt=%0d b_cnt=%0d a_p=%b b_p=%b",
                 bg_rdy, a_count, b_count, a_present, b_present);
        check(bg_rdy === 1'b1,   "抓过背景后 bg_rdy = 1");
        check(a_present === 1'b0, "抓背景期间不报有牌");

        // 2) 仍然是空桌，冻结背景
        bg_cap = 1'b0;
        send_frame; #1;
        $display("  [空桌]   a_cnt=%0d b_cnt=%0d a_p=%b b_p=%b", a_count, b_count, a_present, b_present);
        check(a_count === 12'd0,  "空桌：主框运动列条 = 0");
        check(b_count === 12'd0,  "空桌：小框运动列条 = 0");
        check(a_present === 1'b0, "空桌：不报有牌");

        // 3) 主框出现牌
        mode = 2'd1;
        send_frame; #1;
        $display("  [主框有牌] a_cnt=%0d b_cnt=%0d a_p=%b b_p=%b", a_count, b_count, a_present, b_present);
        check(a_count === 12'd8,  "主框有牌：8 个运动列条（每列条 = 8 像素宽 x ROI 高）");
        check(a_present === 1'b1, "主框有牌：a_present = 1");
        check(b_count === 12'd0,  "主框有牌：小框不受影响（0 列条）");
        check(b_present === 1'b0, "主框有牌：小框不报");

        // 4) 只有小框有东西
        mode = 2'd2;
        send_frame; #1;
        $display("  [小框有物] a_cnt=%0d b_cnt=%0d a_p=%b b_p=%b", a_count, b_count, a_present, b_present);
        check(b_count === 12'd4,  "小框有物：4 个运动列条");
        check(b_present === 1'b1, "小框有物：b_present = 1");
        check(a_count === 12'd0,  "小框有物：主框回到空白（0 列条）");

        // 5) 全部回到空桌 -> HOLD=2 帧后 A 熄灭
        mode = 2'd0;
        send_frame; #1;
        $display("  [回空桌 1] a_cnt=%0d a_p=%b（HOLD=2，应还保持 1）", a_count, a_present);
        check(a_count === 12'd0,  "回空桌：主框运动列条 = 0");
        check(a_present === 1'b1, "回空桌第 1 帧：还在保持期内");
        send_frame; #1;
        $display("  [回空桌 2] a_p=%b", a_present);
        check(a_present === 1'b0, "回空桌第 2 帧：保持期用尽（HOLD=2；上一帧已计过 1 次不达标）");
        send_frame; #1;
        $display("  [回空桌 3] a_p=%b", a_present);
        check(a_present === 1'b0, "回空桌第 3 帧：保持熄灭");

        $display("========================================");
        if (fails == 0) $display("*** ROI MOTION ALL CHECKS PASSED ***  (通过 %0d 项)", oks);
        else            $display("*** FAILED: %0d 项失败 / 通过 %0d 项 ***", fails, oks);
        $display("========================================");
        $finish;
    end
endmodule
