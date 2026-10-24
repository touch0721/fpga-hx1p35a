`timescale 1ns / 1ps
/*
 * tb_uicfgcs520.v — SC500CS I2C 配置模块时序验证
 *
 * 验证内容（断言由 sim/check_i2c_seq.py 校验本 TB 的输出日志完成）：
 *   1) Sensor ID 校验读到 0xEE27 才进入配置流程
 *   2) 初始化表 116 条按索引 0..115 顺序写出，无跳条、无重复（1080p60 表）
 *   3) 索引 0（0x0103 软复位）之后等待 ~5 ms
 *   4) 索引 115（0x0100 stream on，表中最后一条）之后等待 ~10 ms；
 *      1080p60 表 stream on 之后无写入（uicfgcs520.POST_STREAM_WRITE = 0）
 *   5) 分辨率/帧率寄存器写值 = 1920x1080@60（HTS 2852 / VTS 1127）
 *   6) I_ae_req 后 AE 表写出曝光 0x3e00/01/02 + 增益 0x3e08/09 + 数字增益 0x3e06/07
 *   另：同文件 uics520reg_720p60.v 里还有一个 uics520reg_720p60（117 条，
 *       1280x720），仅供回退、本 TB 不例化它；上面 2) 5) 指的是在用的 1080p60 表。
 *
 * 从机模型：开漏 SDA + 上拉，标准 I2C 时序 ACK；对 0x3107/0x3108 随机读返回
 *           0xEE / 0x27（SC500CS 数据手册 V1.8 表 1-6）。
 *
 * 运行（★ uics520reg_1080p60 这个模块名与它所在的文件名 uics520reg_720p60.v
 *      不一致，而 -y 库查找靠“文件名 = 模块名”匹配，所以必须显式列出该文件，
 *      否则报 Unknown module type: uics520reg_1080p60）：
 *   iverilog -g2012 -o sim\tb_uicfgcs520.vvp ^
 *            -y user_source\hdl_source\uics520_cfg ^
 *            -y user_source\hdl_source\uics500_cfg ^
 *            sim\tb_uicfgcs520.v ^
 *            user_source\hdl_source\uics520_cfg\uics520reg_720p60.v
 *   vvp sim\tb_uicfgcs520.vvp > sim\tb_run.log
 *   python sim\check_i2c_seq.py          （校验日志，期望通过 21 项）
 *
 * 注意：I_ag / I_ae 必须在复位后显式驱动一次（本 TB 先给 17 再给 144），
 *       与 ae_set 的实际行为一致；若用声明初值常量，iverilog 在时刻 0 的
 *       事件调度可能不重新触发 SC520GainTbl 的 always @(*) 组合逻辑。
 */
module tb_uicfgcs520;

    // 23.809524 MHz -> 42 ns
    reg clk = 1'b0;
    always #21 clk = ~clk;

    reg        rst_n;
    reg        ae_req;
    reg [15:0] ae_val;
    reg [15:0] ag_val;

    wire scl;
    wire sda;
    pullup (sda);

    wire        cfg_done, ae_cfg_done, sensor_id_ok, iic_error;
    wire [15:0] sensor_id;

    uicfgcs520 #(
        .CLK_DIV(8)             // 仿真用快时钟，缩短 I2C 传输时间
    ) dut (
        .I_clk         (clk),
        .I_rst_n       (rst_n),
        .I_ae_req      (ae_req),
        .I_ae          (ae_val),
        .I_ag          (ag_val),
        .O_cam_scl     (scl),
        .IO_cam_sda    (sda),
        .O_cfg_done    (cfg_done),
        .O_ae_cfg_done (ae_cfg_done),
        .O_sensor_id   (sensor_id),
        .O_sensor_id_ok(sensor_id_ok),
        .O_iic_error   (iic_error)
    );

    // ------------------------------------------------------------------
    // I2C 从机模型（开漏）
    // ------------------------------------------------------------------
    localparam S_ADDR = 3'd1, S_REGH = 3'd2, S_REGL = 3'd3, S_WRD = 3'd4, S_RDD = 3'd5;

    reg        sda_en  = 1'b0;
    reg        sda_val = 1'b0;
    assign sda = sda_en ? sda_val : 1'bz;

    reg        scl_q = 1'b1, sda_q = 1'b1;
    reg        in_xfer   = 1'b0;
    reg [3:0]  bitn      = 4'd0;
    reg [2:0]  pstate    = 3'd0;
    reg [7:0]  shreg     = 8'd0;
    reg [15:0] cur_reg   = 16'd0;
    reg        rd_active = 1'b0;
    reg [7:0]  rd_byte   = 8'd0;

    integer    n_wr = 0;
    integer    n_rd = 0;
    integer    cfg_writes = 0;
    time       t_prev_wr = 0;

    always @(posedge clk) begin
        scl_q <= scl;
        sda_q <= sda;

        // ---- SCL 为高时 SDA 跳变 = START / STOP ----
        if (scl_q && scl) begin
            if (sda_q && !sda) begin            // START / repeated START
                bitn      <= 4'd0;
                pstate    <= S_ADDR;
                in_xfer   <= 1'b1;
                rd_active <= 1'b0;
                sda_en    <= 1'b0;
                shreg     <= 8'd0;
            end else if (!sda_q && sda) begin   // STOP
                in_xfer   <= 1'b0;
                rd_active <= 1'b0;
                sda_en    <= 1'b0;
                pstate    <= 3'd0;
            end
        end

        // ---- SCL 上升沿：采样数据位 / ACK 位 ----
        if (scl && !scl_q && in_xfer) begin
            if (bitn < 4'd8) begin
                shreg <= {shreg[6:0], sda};
                bitn  <= bitn + 4'd1;
            end else begin
                bitn <= 4'd0;                   // 第 9 个时钟：一个字节结束
                case (pstate)
                    S_ADDR: begin
                        if (shreg[0]) begin     // 读命令：准备返回寄存器内容
                            rd_byte   <= (cur_reg == 16'h3107) ? 8'hee :
                                         (cur_reg == 16'h3108) ? 8'h27 : 8'h00;
                            rd_active <= 1'b1;
                            pstate    <= S_RDD;
                        end else begin
                            pstate <= S_REGH;
                        end
                    end
                    S_REGH: begin
                        cur_reg[15:8] <= shreg;
                        pstate        <= S_REGL;
                    end
                    S_REGL: begin
                        cur_reg[7:0] <= shreg;
                        pstate       <= S_WRD;
                    end
                    S_WRD: begin
                        n_wr <= n_wr + 1;
                        $display("WR idx=%0d addr=%04h data=%02h t=%0t dt=%0t",
                                 dut.reg_index, cur_reg, shreg, $time, $time - t_prev_wr);
                        t_prev_wr <= $time;
                        pstate    <= S_WRD;
                    end
                    S_RDD: begin
                        n_rd <= n_rd + 1;
                        $display("RD idx=%0d addr=%04h data=%02h t=%0t",
                                 dut.reg_index, cur_reg, shreg, $time);
                        // 读数据字节已送出：立即释放 SDA，等待主机 NACK + STOP，
                        // 否则会与主机 STOP1 拉低 SDA 产生总线冲突。
                        rd_active <= 1'b0;
                        sda_en    <= 1'b0;
                        pstate    <= S_RDD;
                    end
                    default: pstate <= S_ADDR;
                endcase
            end
        end

        // ---- SCL 下降沿：驱动 ACK 或读数据位 ----
        if (!scl && scl_q && in_xfer) begin
            if (bitn == 4'd8) begin
                if (rd_active) begin
                    sda_en <= 1'b0;             // 读数据的 ACK 位由主机驱动
                end else if (pstate <= S_WRD) begin
                    sda_en  <= 1'b1;            // 从机 ACK（地址/寄存器/写数据字节）
                    sda_val <= 1'b0;
                end else begin
                    sda_en <= 1'b0;             // 读数据阶段结束，完全释放总线
                end
            end else if (rd_active) begin
                sda_en  <= 1'b1;
                sda_val <= rd_byte[3'd7 - bitn[2:0]];
            end else begin
                sda_en <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------
    // 激励
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_uicfgcs520 start ===");
        ae_req = 1'b0;
        ae_val = 16'd2390;              // VTS=1200（720p60）下的最大曝光 = 2*1200-10
        ag_val = 16'd17;                // 与 ae_set 复位值一致
        rst_n  = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);
        ag_val = 16'd144;               // 配置期间把增益调到目标值 15.750x

        wait (cfg_done === 1'b1);
        $display("CFG_DONE t=%0t sensor_id=%04h id_ok=%b iic_error=%b TOTAL_WRITES=%0d",
                 $time, sensor_id, sensor_id_ok, iic_error, n_wr);
        cfg_writes = n_wr;

        @(posedge clk);
        ae_req = 1'b1;
        @(posedge clk);
        ae_req = 1'b0;

        // 配置完成时 O_ae_cfg_done 已为 1，这里等 AE 表确实写出后再收尾
        wait (n_wr == cfg_writes + 7);
        repeat (50) @(posedge clk);
        $display("AE_DONE t=%0t TOTAL_WRITES=%0d READS=%0d", $time, n_wr, n_rd);
        $display("SUMMARY cfg_writes=%0d ae_writes=%0d", cfg_writes, n_wr - cfg_writes);
        $display("=== tb_uicfgcs520 end ===");
        $finish;
    end

    initial begin
        #400ms;
        $display("TIMEOUT cfg_done=%b ae_cfg_done=%b n_wr=%0d n_rd=%0d",
                 cfg_done, ae_cfg_done, n_wr, n_rd);
        $finish;
    end

endmodule
