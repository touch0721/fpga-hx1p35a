`timescale 1ns / 1ps
// ============================================================================
//  tb_sc500cs_pack.v — sc500cs_cam_pack.v 自检 testbench
// ----------------------------------------------------------------------------
//  完全自包含：只依赖 sc500cs_cam_pack.v，不需要本工程的任何其它文件。
//  内含一个开漏 I2C 从机模型，对 0x3107/0x3108 返回 0xEE / 0x27。
//
//  检查项：
//    A) I2C 配置
//       A1  读 0x3107=0xEE、0x3108=0x27 -> ID=0xEE27、id_ok=1
//       A2  初始化表 117 条按索引 0..116 顺序写出，无跳条
//       A3  索引 0 = 0x0103=0x01（软复位）
//       A4  索引 115 = 0x0100=0x01（stream on）
//       A5  索引 116 = 0x302d=0x00
//       A6  软复位后等待 ≈5 ms
//       A7  stream on 后等待 ≈10 ms
//       A8  配置完成后自动写一次 AE 表（7 条，AE_ONLINE=0 无需按键）
//       A9  AE 表内容：曝光 1990、模拟增益 0x1f/0x3f、数字增益 1.000x
//       A10 O_iic_error = 0
//       A11 分辨率/帧率寄存器：1280 / 720 / VTS 1000
//    B) 黑电平校正 sc500cs_blc_raw40
//       B1  输出 = 输入 - 15
//       B2  小于偏置时钳位到 0（不借位）
//    C) Gamma sc500cs_gamma_rgb96
//       C1  0x00->0x00、0x40->0x61、0x80->0x9d、0xc0->0xd1、0xff->0xff
//
//  运行：
//    iverilog -g2012 -o tb.vvp tb_sc500cs_pack.v sc500cs_cam_pack.v
//    vvp tb.vvp
// ============================================================================
module tb_sc500cs_pack;

    // 23.809524 MHz -> 42 ns（与 CLK_HZ 参数一致）
    reg clk = 1'b0;
    always #21 clk = ~clk;

    integer fails = 0;
    task chk;
        input cond;
        input [255:0] msg;
        begin
            if (cond) $display("  PASS  %0s", msg);
            else begin $display("  FAIL  %0s", msg); fails = fails + 1; end
        end
    endtask

    // ========================================================================
    // A) I2C 配置
    // ========================================================================
    reg         rst_n = 1'b0;
    wire        scl, sda;
    pullup (sda);

    wire        cfg_done, ae_cfg_done, id_ok, iic_error;
    wire [15:0] sensor_id;

    sc500cs_cfg #(
        .CLK_DIV   (16'd8),          // 仿真加速：只影响 SCL 快慢
        .CLK_HZ    (23809524),       // 真实主频，保证 5ms/10ms 准确
        .AE_DEFAULT(16'd1990),
        .AG_DEFAULT(16'd144),        // -> 0x1f / 0x3f
        .AE_ONLINE (1'b0)            // 无按键：自动写一次 AE 表
    ) u_cfg (
        .I_clk        (clk),
        .I_rst_n      (rst_n),
        .I_ae_req     (1'b0),        // AE_ONLINE=0 时被忽略
        .I_ae         (16'd0),
        .I_ag         (16'd0),
        .O_cam_scl    (scl),
        .IO_cam_sda   (sda),
        .O_cfg_done   (cfg_done),
        .O_ae_cfg_done(ae_cfg_done),
        .O_sensor_id  (sensor_id),
        .O_sensor_id_ok(id_ok),
        .O_iic_error  (iic_error)
    );

    // ---------------- 开漏 I2C 从机模型 ----------------
    localparam S_ADDR = 3'd1, S_REGH = 3'd2, S_REGL = 3'd3, S_WRD = 3'd4, S_RDD = 3'd5;

    reg        sda_en = 1'b0, sda_val = 1'b0;
    assign sda = sda_en ? sda_val : 1'bz;

    reg        scl_q = 1'b1, sda_q = 1'b1;
    reg        in_xfer = 1'b0;
    reg [3:0]  bitn = 4'd0;
    reg [2:0]  pstate = 3'd0;
    reg [7:0]  shreg = 8'd0;
    reg [15:0] cur_reg = 16'd0;
    reg        rd_active = 1'b0;
    reg [7:0]  rd_byte = 8'd0;

    integer n_wr = 0, n_rd = 0, idx = 0;
    time    t_prev = 0, dt_reset = 0, dt_stream = 0;

    // 记录前 117 条的地址/数据，供逐条核对
    reg [15:0] wr_addr [0:127];
    reg [7:0]  wr_data [0:127];

    always @(posedge clk) begin
        scl_q <= scl;
        sda_q <= sda;

        if (scl_q && scl) begin
            if (sda_q && !sda) begin
                bitn <= 4'd0; pstate <= S_ADDR; in_xfer <= 1'b1;
                rd_active <= 1'b0; sda_en <= 1'b0; shreg <= 8'd0;
            end else if (!sda_q && sda) begin
                in_xfer <= 1'b0; rd_active <= 1'b0; sda_en <= 1'b0; pstate <= 3'd0;
            end
        end

        if (scl && !scl_q && in_xfer) begin
            if (bitn < 4'd8) begin
                shreg <= {shreg[6:0], sda};
                bitn  <= bitn + 4'd1;
            end else begin
                bitn <= 4'd0;
                case (pstate)
                    S_ADDR: begin
                        if (shreg[0]) begin
                            rd_byte   <= (cur_reg == 16'h3107) ? 8'hee :
                                         (cur_reg == 16'h3108) ? 8'h27 : 8'h00;
                            rd_active <= 1'b1;
                            pstate    <= S_RDD;
                        end else pstate <= S_REGH;
                    end
                    S_REGH: begin cur_reg[15:8] <= shreg; pstate <= S_REGL; end
                    S_REGL: begin cur_reg[7:0]  <= shreg; pstate <= S_WRD;  end
                    S_WRD: begin
                        if (n_wr < 128) begin
                            wr_addr[n_wr] <= cur_reg;
                            wr_data[n_wr] <= shreg;
                        end
                        if (idx == 1)                     dt_reset  <= $time - t_prev;
                        if (idx == 116)                   dt_stream <= $time - t_prev;
                        $display("  WR[%0d] 0x%04x = 0x%02x   dt=%0t", idx, cur_reg, shreg,
                                 $time - t_prev);
                        t_prev <= $time;
                        n_wr   <= n_wr + 1;
                        idx    <= idx + 1;
                        pstate <= S_WRD;
                    end
                    S_RDD: begin
                        n_rd <= n_rd + 1;
                        $display("  RD 0x%04x = 0x%02x", cur_reg, shreg);
                        rd_active <= 1'b0; sda_en <= 1'b0; pstate <= S_RDD;
                    end
                    default: pstate <= S_ADDR;
                endcase
            end
        end

        if (!scl && scl_q && in_xfer) begin
            if (bitn == 4'd8) begin
                if (rd_active) sda_en <= 1'b0;
                else if (pstate <= S_WRD) begin sda_en <= 1'b1; sda_val <= 1'b0; end
                else sda_en <= 1'b0;
            end else if (rd_active) begin
                sda_en <= 1'b1; sda_val <= rd_byte[3'd7 - bitn[2:0]];
            end else sda_en <= 1'b0;
        end
    end

    // 期望的 AE 表内容（AE_DEFAULT=1990, AG_DEFAULT=144）
    localparam [15:0] EXP_AE00 = 16'h3e00, EXP_AE01 = 16'h3e01, EXP_AE02 = 16'h3e02;
    localparam [15:0] EXP_AG08 = 16'h3e08, EXP_AG09 = 16'h3e09;
    localparam [15:0] EXP_DG06 = 16'h3e06, EXP_DG07 = 16'h3e07;

    integer i;
    initial begin
        $display("=== tb_sc500cs_pack start ===");
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;

        wait (cfg_done === 1'b1);
        $display("  CFG_DONE t=%0t", $time);
        // O_ae_cfg_done 在 cfg_done 时已为 1，这里等自动 AE 那一笔真正写完
        wait (n_wr >= 117 + 7);
        repeat (20) @(posedge clk);

        // ---------------- A) 检查 ----------------
        chk(sensor_id == 16'hee27, "A1 ID = 0xEE27");
        chk(id_ok === 1'b1,        "A1 sensor_id_ok = 1");
        chk(n_rd == 2,             "A1 恰好 2 次随机读");
        chk(n_wr == 117 + 7,       "A2 写入总数 = 117 + 7(AE)");
        chk(wr_addr[0] == 16'h0103 && wr_data[0] == 8'h01, "A3 索引0 = 0x0103=0x01");
        chk(wr_addr[115] == 16'h0100 && wr_data[115] == 8'h01, "A4 索引115 = 0x0100=0x01");
        chk(wr_addr[116] == 16'h302d && wr_data[116] == 8'h00, "A5 索引116 = 0x302d=0x00");
        chk(dt_reset  > 4900000  && dt_reset  < 5100000,  "A6 软复位后等待 ≈5 ms");
        chk(dt_stream > 9900000  && dt_stream < 10100000, "A7 stream on 后等待 ≈10 ms");
        chk(wr_addr[117] == EXP_AE00, "A8 自动 AE 第1条 = 0x3e00");
        // 曝光 = {0x3e00[3:0], 0x3e01, 0x3e02[7:4]}，期望 1990 = 0x7C6
        chk(({4'h0, wr_data[117][3:0]} << 12 | wr_data[118] << 4 | wr_data[119][7:4]) == 16'd1990,
            "A9 曝光 = 1990 = 2*VTS-10");
        chk(wr_addr[120] == EXP_AG08 && wr_data[120] == 8'h1f, "A9 0x3e08 = 0x1f (8x 粗调)");
        chk(wr_addr[121] == EXP_AG09 && wr_data[121] == 8'h3f, "A9 0x3e09 = 0x3f (15.750x)");
        chk(wr_addr[122] == EXP_DG06 && wr_data[122] == 8'h00, "A9 0x3e06 = 0x00 (数字 1x)");
        chk(wr_addr[123] == EXP_DG07 && wr_data[123] == 8'h80, "A9 0x3e07 = 0x80 (数字 1x)");
        chk(iic_error === 1'b0, "A10 O_iic_error = 0");
        begin : res_chk
            reg [15:0] w, h, vts;
            w = 16'd0; h = 16'd0; vts = 16'd0;
            for (i = 0; i < 117; i = i + 1) begin
                if (wr_addr[i] == 16'h3208) w[15:8] = wr_data[i];
                if (wr_addr[i] == 16'h3209) w[7:0]  = wr_data[i];
                if (wr_addr[i] == 16'h320a) h[15:8] = wr_data[i];
                if (wr_addr[i] == 16'h320b) h[7:0]  = wr_data[i];
                if (wr_addr[i] == 16'h320e) vts[15:8] = wr_data[i];
                if (wr_addr[i] == 16'h320f) vts[7:0]  = wr_data[i];
            end
            $display("  分辨率 = %0d x %0d, VTS = %0d", w, h, vts);
            chk(w == 1280 && h == 720 && vts == 1000, "A11 1280x720, VTS=1000 (60fps)");
        end

        // ---------------- B) 黑电平校正 ----------------
        run_blc;
        // ---------------- C) Gamma ----------------
        run_gamma;

        $display("=== FAILS %0d ===", fails);
        if (fails == 0) $display("*** ALL CHECKS PASSED ***");
        else            $display("*** 有失败项，移植包不可用 ***");
        $finish;
    end

    // ========================================================================
    // B) sc500cs_blc_raw40
    // ========================================================================
    reg  [39:0] blc_in = 40'd0;
    reg         blc_v  = 1'b0;
    wire [39:0] blc_out;
    wire        blc_vo;

    sc500cs_blc_raw40 #(
        .Black_level_offset_r0 (10'd15),
        .Black_level_offset_r1 (10'd15),
        .Black_level_offset_r2 (10'd15),
        .Black_level_offset_r3 (10'd15)
    ) u_blc (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_tlast(1'b0), .I_tuser(1'b0),
        .I_tdata(blc_in), .I_tvalid(blc_v),
        .O_tlast(), .O_tuser(), .O_tdata(blc_out), .O_tvalid(blc_vo)
    );

    task run_blc;
        begin
            // 4 个像素：512 / 4 / 255 / 1023  -> 497 / 0(钳位) / 240 / 1008
            blc_in = {10'd1023, 10'd255, 10'd4, 10'd512};
            blc_v  = 1'b1;
            @(posedge clk);          // 这一拍 dout / r_tvalid 被写入
            @(negedge clk);          // 等到 NBA 生效后再读
            $display("  BLC in=%010h out=%010h vo=%b", blc_in, blc_out, blc_vo);
            chk(blc_out[9:0]   == 10'd497,  "B1 lane0 512-15 = 497");
            chk(blc_out[19:10] == 10'd0,    "B2 lane1 4 < 15 -> 钳位 0");
            chk(blc_out[29:20] == 10'd240,  "B1 lane2 255-15 = 240");
            chk(blc_out[39:30] == 10'd1008, "B1 lane3 1023-15 = 1008");
            chk(blc_vo === 1'b1,            "B1 O_tvalid 在 I_tvalid 后一拍拉高");
            blc_v  = 1'b0;
            @(posedge clk);
        end
    endtask

    // ========================================================================
    // C) sc500cs_gamma_rgb96
    // ========================================================================
    reg  [95:0] gam_in = 96'd0;
    reg         gam_v  = 1'b0;
    wire [95:0] gam_out;
    wire        gam_vo;

    sc500cs_gamma_rgb96 u_gamma (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_tlast(1'b0), .I_tuser(1'b0),
        .I_tdata(gam_in), .I_tvalid(gam_v),
        .O_tlast(), .O_tuser(), .O_tdata(gam_out), .O_tvalid(gam_vo)
    );

    task gamma_one;
        input  [7:0] vin;
        output [7:0] vout;
        begin
            gam_in = {vin, vin, vin, vin, vin, vin, vin, vin, vin, vin, vin, vin};
            gam_v  = 1'b1;
            @(posedge clk);          // gamma 为纯流水，这一拍输出被写入
            @(negedge clk);          // 等到 NBA 生效后再读（此时 I_tvalid 仍为 1）
            vout   = gam_out[7:0];
            gam_v  = 1'b0;
            @(posedge clk);
        end
    endtask

    reg [7:0] gv;
    task run_gamma;
        begin
            gamma_one(8'h00, gv); chk(gv == 8'h00, "C1 gamma(0x00) = 0x00 (保黑)");
            gamma_one(8'h40, gv); chk(gv == 8'h61, "C1 gamma(0x40) = 0x61");
            gamma_one(8'h80, gv); chk(gv == 8'h9d, "C1 gamma(0x80) = 0x9d");
            gamma_one(8'hc0, gv); chk(gv == 8'hd1, "C1 gamma(0xc0) = 0xd1");
            gamma_one(8'hff, gv); chk(gv == 8'hff, "C1 gamma(0xff) = 0xff (保白)");
            gamma_one(8'h80, gv);
            chk(gam_out[95:88] == gv && gam_out[15:8] == gv && gam_out[7:0] == gv,
                "C1 12 个字节使用同一条曲线");
        end
    endtask

    initial begin
        #200ms;
        $display("TIMEOUT fails=%0d n_wr=%0d cfg_done=%b", fails, n_wr, cfg_done);
        $finish;
    end

endmodule
