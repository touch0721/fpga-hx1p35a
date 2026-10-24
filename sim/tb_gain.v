`timescale 1ns / 1ps
// tb_gain.v — SC520GainTbl 穷举输出检查（对照手册表 2-4）
module tb_gain;
    reg  [15:0] ag;
    wire [15:0] o;

    SC520GainTbl u (.I_AG(ag), .O(o));

    integer i;
    initial begin
        for (i = 14; i < 150; i = i + 1) begin
            ag = i[15:0];
            #1;
            if (i < 20 || i > 140)
                $display("I_AG=%0d O=%04h", i, o);
        end
        ag = 16'd144; #1; $display("I_AG=144 O=%04h", o);
        ag = 16'd176; #1; $display("I_AG=176 O=%04h", o);
        $finish;
    end
endmodule
