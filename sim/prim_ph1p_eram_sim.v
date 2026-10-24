// ============================================================================
//  prim_ph1p_eram_sim.v — 安路 PH1P_LOGIC_ERAM 的仿真模型（仅用于 iverilog）
// ----------------------------------------------------------------------------
//  安路 TD 综合时用真实原语，iverilog 不认这个模块，所以仿真时把它一起编译。
//  行为：简单双端口 —— A 口写、B 口读；读写均同步；B 口输出 1 拍延迟
//        （对应 REGMODE_B = "NOREG" 时 BRAM 内部输出寄存器）。
//  该文件不需要加入 TD 工程，只在仿真命令行里带上。
// ============================================================================
module PH1P_LOGIC_ERAM #(
    parameter DATA_WIDTH_A = 4,
    parameter ADDR_WIDTH_A = 9,
    parameter DATA_DEPTH_A = 512,
    parameter DATA_WIDTH_B = 4,
    parameter ADDR_WIDTH_B = 9,
    parameter DATA_DEPTH_B = 512,
    parameter MODE         = "DP",
    parameter REGMODE_A    = "NOREG",
    parameter REGMODE_B    = "NOREG",
    parameter WRITEMODE_A  = "NORMAL",
    parameter WRITEMODE_B  = "READBEFOREWRITE",
    parameter RESETMODE_A  = "ASYNC",
    parameter RESETMODE_B  = "ASYNC",
    parameter IMPLEMENT    = "20K(FAST)",
    parameter ECC_ENCODE   = "DISABLE",
    parameter ECC_DECODE   = "DISABLE",
    parameter CLKMODE      = "ASYNC",
    parameter SSROVERCE    = "DISABLE",
    parameter OREGSET_A    = "RESET",
    parameter OREGSET_B    = "RESET",
    parameter ASYNC_RESET_RELEASE_A = "ASYNC",
    parameter ASYNC_RESET_RELEASE_B = "ASYNC",
    parameter INIT_FILE    = "NONE",
    parameter FILL_ALL     = "NONE"
) (
    output [DATA_WIDTH_A-1:0] doa,
    output [DATA_WIDTH_B-1:0] dob,
    input  [DATA_WIDTH_A-1:0] dia,
    input  [DATA_WIDTH_B-1:0] dib,
    input  [ADDR_WIDTH_A-1:0] addra,
    input  [ADDR_WIDTH_B-1:0] addrb,
    input  wea,
    input  web,
    input  clka,
    input  clkb,
    input  cea,
    input  ceb,
    input  ocea,
    input  oceb,
    input  bea,
    input  beb,
    input  rsta,
    input  rstb,
    output ecc_sbiterr,
    output ecc_dbiterr,
    input  ecc_sbiterrinj,
    input  ecc_dbiterrinj
);
    integer i;
    reg [DATA_WIDTH_A-1:0] mem [0:DATA_DEPTH_A-1];
    reg [DATA_WIDTH_A-1:0] doa_r;
    reg [DATA_WIDTH_B-1:0] dob_r;

    initial begin
        for (i = 0; i < DATA_DEPTH_A; i = i + 1)
            mem[i] = {DATA_WIDTH_A{1'b0}};
    end

    // A 口：写
    always @(posedge clka) begin
        if (cea && wea && !bea)
            mem[addra] <= dia;
    end

    always @(posedge clka) begin
        if (rsta) doa_r <= {DATA_WIDTH_A{1'b0}};
        else if (cea && !wea) doa_r <= mem[addra];
    end
    assign doa = doa_r;

    // B 口：读（1 拍延迟）
    always @(posedge clkb) begin
        if (rstb) dob_r <= {DATA_WIDTH_B{1'b0}};
        else if (ceb) dob_r <= mem[addrb];
    end
    assign dob = dob_r;

    assign ecc_sbiterr = 1'b0;
    assign ecc_dbiterr = 1'b0;
endmodule
