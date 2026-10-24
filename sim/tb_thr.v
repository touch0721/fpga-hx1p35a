`timescale 1ns / 1ps
// ============================================================================
//  tb_thr.v — 专门验证"填充阈值"逻辑
//  图案（每行 80 列 = 20 拍）：
//     行 40..120 ：列 2..5 与 40..43 -> 每行 8 个白像素（"实心"行）
//     行 80      ：只有列 2..3      -> 只有 2 个白像素（"稀疏"行，用来当反例）
//  验证：
//     fill_thr = I_fill_thr x 8
//     row_has  = (本行白像素数 > fill_thr)
//     I_fill_thr = 1 -> 阈值 8  -> 稀疏行(2) 不标记，实心行(8) 也不标记（8 不大于 8）
//     I_fill_thr = 0 -> 阈值 0  -> 稀疏行(2) 标记
// ============================================================================
module tb_thr;
    localparam integer W  = 80;
    localparam integer H  = 160;
    localparam integer BE = W/4;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [3:0] bin;
    reg        valid, last, user;
    reg  [8:0] beat;
    reg  [11:0] rw;
    reg  [7:0] thr;                 // I_fill_thr（档位）

    wire       rhas;

    bin_proj_bound #(
        .BEATS(W/4), .IMG_HEIGHT(H), .RW((H+2)/3),
        .ROW_LO(0), .ROW_HI(H)
    ) u_proj (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_bin(bin), .I_valid(valid), .I_last(last), .I_user(user),
        .I_mid_ratio(2'd0), .I_beat(beat), .I_row(rw),
        .I_fill_thr(thr),
        .I_roi_idx(4'd0), .I_roi_yidx(4'd0),
        .O_roi_cnt(), .O_rcnt(),
        .O_xl(), .O_xr(), .O_yt(), .O_yb(), .O_row(), .O_rowpeak(),
        .O_x1_all(), .O_x2_all(), .O_rowhas(rhas)
    );

    // 本行白像素数
    function [11:0] row_px;
        input [11:0] r;
        begin
            if (r == 12'd80)                       row_px = 12'd2;    // 稀疏行
            else if ((r >= 40) && (r <= 120))      row_px = 12'd8;    // 实心行
            else                                   row_px = 12'd0;
        end
    endfunction

    // 第 c 列在行 r 是否为白
    function px_on;
        input [11:0] r, c;
        begin
            if (r == 12'd80)
                px_on = ((c >= 2) && (c <= 3));                        // 稀疏行只有 2 个
            else if ((r >= 40) && (r <= 120))
                px_on = (((c >= 2) && (c <= 5)) || ((c >= 40) && (c <= 43)));
            else
                px_on = 1'b0;
        end
    endfunction

    integer r, b, k, n_mark, n_mark80;
    reg [3:0] w4;
    reg       rhas_at_last;              // 行末那一拍的 O_rowhas

    initial begin
        bin=0; valid=0; last=0; user=0; beat=0; rw=0; thr=8'd1;
        #20 rst_n = 1; #20;

        // 送 3 帧，统计第 3 帧（前 2 帧让状态机稳定）
        for (k = 0; k < 3; k = k + 1) begin
            n_mark = 0; n_mark80 = 0;
            for (r = 0; r < H; r = r + 1) begin
                for (b = 0; b < BE; b = b + 1) begin
                    w4[0]=px_on(r[11:0], b*4+0); w4[1]=px_on(r[11:0], b*4+1);
                    w4[2]=px_on(r[11:0], b*4+2); w4[3]=px_on(r[11:0], b*4+3);
                    bin <= w4; valid <= 1'b1;
                    last <= (b == BE-1);
                    user <= (r==0) && (b==0);
                    beat <= b[8:0]; rw <= r[11:0];
                    @(posedge clk);
                    // 行末那一拍的值就是 O_rowhas（已寄存）
                    if (b == BE-1) begin
                        #1;
                        if (rhas) begin
                            n_mark = n_mark + 1;
                            if (r == 80) n_mark80 = n_mark80 + 1;
                        end
                    end
                end
            end
            valid <= 1'b0; last <= 1'b0; user <= 1'b0;
            repeat (3000) @(posedge clk);
            if (k == 2) begin
                $display("");
                $display("=== 阈值档位 thr=%0d -> 实际阈值 %0d ===", thr, thr*8);
                $display("  被标记(O_rowhas=1)的行数 = %0d", n_mark);
                $display("  其中稀疏行(80)被标记 = %0d 次（期望 0）", n_mark80);
                if (n_mark80 == 0) $display("  PASS  稀疏行(2 个白像素)被阈值挡掉了");
                else               $display("  FAIL  稀疏行不应被标记");
                if (n_mark == 0)   $display("  PASS  实心行(8)也没超阈值 8（8 不大于 8）-> 全部不标记");
                else               $display("  实心行被标记 %0d 行", n_mark);
            end
        end
        $finish;
    end
endmodule
