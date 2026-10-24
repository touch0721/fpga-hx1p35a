`timescale 1ns / 1ps
// ============================================================================
//  tb_tile_recog.v — tile_recog_onchip 自检（阶段 1 切分 + 阶段 2 细粒度计数）
// ----------------------------------------------------------------------------
//  图像 256x128（64x16 个 8x8 块），静态内容，四种图案（mode）：
//    0  原有三色带：x 0..23 红 / 24..47 暗无彩 / 48..71 蓝 / 其余背景
//    1  "四筒"  ：96..159 的 64x96 子框里 4 个 12x12 圆点（2x2 排布）
//    2  "五条"  ：同一子框里 5 根 8 像素宽、48 像素高的竖条（缝 4 像素）
//    3  "一筒+噪点"：1 个圆点 + 1 个孤立的 4x4 格（应被 DOT_MIN 滤掉）
//
//  阶段 2 的计数单位：4x4 像素 = 1 格（一张立牌 ≈ 17x23 格）
//    圆点 12x12 = 3x3 格 = 9 格；竖条 8x48 = 2x12 格 = 24 格
// ----------------------------------------------------------------------------
//  第 1 组（mode 0）：一个"三张牌并排"的合并框 -> 切成 3 个子框，逐槽查特征
//  第 2 组（mode 0）：一个"单张牌"的框 -> N=1
//  第 3 组（mode 1）：四筒 -> dots=4 bars=2 cells=36 maxc=9
//  第 4 组（mode 2）：五条 -> dots=5 bars=5 cells=120 maxc=24
//  第 5 组（mode 3）：一筒 + 噪点 -> dots=1 cells=10（噪点被 DOT_MIN 滤掉）
// ============================================================================
module tb_tile_recog;
    localparam integer IMG_W = 256;
    localparam integer IMG_H = 128;
    localparam integer LB_MAX = (IMG_W/4) - 1;   // 63
    localparam integer LN_MAX = IMG_H - 1;       // 127
    localparam integer NBOX  = 8;
    localparam integer NSLOT = 16;
    localparam integer TX0   = 96;               // 测试子框左上角（像素）
    localparam integer TY0   = 0;
    localparam integer BLANK = 1600;             // 消隐拍数：要够帧末两段 FSM 跑完

    reg clk = 0, rst_n = 0, en = 1;
    always #5 clk = ~clk;                         // 100 MHz

    reg         pix_valid = 0, pix_last = 0, pix_user = 0;
    reg  [31:0] y_now = 0;
    reg  [7:0]  col4 = 0;

    reg  [NBOX-1:0]    bv = 0;
    reg  [12*NBOX-1:0] bx1 = 0, bx2 = 0, by1 = 0, by2 = 0;
    reg  [3:0]         sel = 0;
    reg  [1:0]         mode = 0;

    wire        o_sel_v, o_sel_blank, o_sel_ar_ok, o_done;
    wire [21:0] o_sel_pixcnt;
    wire [31:0] o_sel_colsig;
    wire [11:0] o_sel_x1, o_sel_x2, o_sel_y1, o_sel_y2;
    wire [3:0]  o_hit_slot;
    wire [4:0]  o_nslot;
    wire [4:0]  o_dots, o_bars;
    wire [8:0]  o_cells, o_maxc;

    tile_recog_onchip #(
        .IMG_W(IMG_W), .IMG_H(IMG_H), .NBOX(NBOX), .NSLOT(NSLOT),
        .MAX_TILE(10), .Y_DARK(8'd150), .BLANK_PCT(2), .MIN_BLK(3),
        .GXOFF(0), .GYOFF(0)
    ) dut (
        .I_pix_clk(clk), .I_rst_n(rst_n), .I_en(en),
        .I_pix_valid(pix_valid), .I_pix_last(pix_last), .I_pix_user(pix_user),
        .I_y_now(y_now), .I_col4(col4),
        .I_bv(bv), .I_bx1(bx1), .I_bx2(bx2), .I_by1(by1), .I_by2(by2),
        .I_sel(sel),
        .O_sel_v(o_sel_v), .O_sel_pixcnt(o_sel_pixcnt), .O_sel_colsig(o_sel_colsig),
        .O_sel_blank(o_sel_blank), .O_sel_ar_ok(o_sel_ar_ok),
        .O_sel_x1(o_sel_x1), .O_sel_x2(o_sel_x2),
        .O_sel_y1(o_sel_y1), .O_sel_y2(o_sel_y2),
        .O_hit_slot(o_hit_slot), .O_nslot(o_nslot), .O_done(o_done),
        .O_sel_dots(o_dots), .O_sel_bars(o_bars),
        .O_sel_cells(o_cells), .O_sel_maxc(o_maxc)
    );

    integer fails = 0, oks = 0;
    integer hit0_cnt = 0;                       // 调试：s_hit[0] 高了多少拍
    integer vend_cnt = 0, frun_cnt = 0, fdone_cnt = 0, spdone_cnt = 0, gdone_cnt = 0;
    always @(posedge clk) begin
        if (dut.s_hit_sel)     hit0_cnt  = hit0_cnt  + 1;
        if (dut.s_vend)        vend_cnt  = vend_cnt  + 1;
        if (dut.s_fst == 3'd1) frun_cnt  = frun_cnt  + 1;
        if (dut.s_fst == 3'd2) fdone_cnt = fdone_cnt + 1;
        if (dut.s_sp  == 3'd5) spdone_cnt= spdone_cnt+ 1;
        if (dut.s_gdone)       gdone_cnt = gdone_cnt + 1;
    end

    task check;
        input cond;
        input [255:0] msg;
        begin
            if (cond) oks = oks + 1;
            else begin fails = fails + 1; $display("  FAIL: %0s", msg); end
        end
    endtask

    // 像素内容：按 mode 生成（前景 = 暗且无彩，或任何有彩色）
    task content;
        input  integer x, y;
        output [7:0] oy, oc;
        integer dx, dy, c, r;
        begin
            oy = 8'd200; oc = 2'd3;                       // 背景：亮 + 无彩 -> 非前景
            if (mode == 2'd0) begin
                if      (x < 24) begin oy = 8'd120; oc = 2'd0; end
                else if (x < 48) begin oy = 8'd60;  oc = 2'd3; end
                else if (x < 72) begin oy = 8'd120; oc = 2'd2; end
            end else if ((x >= TX0) && (x < TX0 + 64) && (y >= TY0) && (y < TY0 + 96)) begin
                dx = x - TX0; dy = y - TY0;
                c  = dx >> 2;                             // 格列 0..15
                r  = dy >> 2;                             // 格行 0..23
                if (mode == 2'd1) begin
                    // 四个 3x3 格的圆点：(列 2..4 / 9..11) x (行 3..5 / 14..16)
                    if ((((c >= 2) && (c <= 4)) || ((c >= 9) && (c <= 11))) &&
                        (((r >= 3) && (r <= 5)) || ((r >= 14) && (r <= 16))))
                        begin oy = 8'd60; oc = 2'd3; end
                end else if (mode == 2'd2) begin
                    // 五根竖条：8 像素宽（dx%12 < 8，且 dx<56 保证正好 5 根）、
                    // 48 像素高（格行 3..14）-> 每根 2x12 = 24 格
                    if (((dx % 12) < 8) && (dx < 56) && (r >= 3) && (r <= 14))
                        begin oy = 8'd60; oc = 2'd3; end
                end else begin
                    // 一个圆点 + 一个孤立格（噪点）
                    if ((c >= 2) && (c <= 4) && (r >= 3) && (r <= 5))
                        begin oy = 8'd60; oc = 2'd3; end
                    if ((c == 12) && (r == 20))
                        begin oy = 8'd60; oc = 2'd3; end
                end
            end
        end
    endtask

    integer l, ln, lb;
    reg [7:0] py, pcd;
    reg [31:0] wy, wco;

    task send_frame;
        begin
            for (ln = 0; ln <= LN_MAX; ln = ln + 1) begin
                for (lb = 0; lb <= LB_MAX; lb = lb + 1) begin
                    wy = 32'd0; wco = 32'd0;
                    for (l = 0; l < 4; l = l + 1) begin
                        content(4*lb + l, ln, py, pcd);
                        wy  = wy  | (py  << (8*l));
                        wco = wco | ({30'd0, pcd} << (2*l));
                    end
                    @(negedge clk);
                    y_now = wy; col4 = wco[7:0];
                    pix_valid = 1'b1;
                    pix_last  = (lb == LB_MAX);
                    pix_user  = (lb == 0) && (ln == 0);
                    @(posedge clk);
                end
            end
            @(negedge clk);
            pix_valid = 1'b0; pix_last = 1'b0; pix_user = 1'b0;
            for (l = 0; l < BLANK; l = l + 1) @(posedge clk);   // 消隐：够两段 FSM 跑完
        end
    endtask

    task set_box;
        input [11:0] x1, x2, y1, y2;
        begin
            bx1[0*12 +: 12] = x1; bx2[0*12 +: 12] = x2;
            by1[0*12 +: 12] = y1; by2[0*12 +: 12] = y2;
        end
    endtask

    initial begin
        $dumpfile("tb_tile_recog.vcd");
        $dumpvars(0, tb_tile_recog);

        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        $display("=== tb_tile_recog：阶段 1 切分 + 阶段 2 细粒度计数 ===");

        // ================= 第 1 组：三张牌并排（mode 0）=================
        mode = 2'd0;
        bv = 8'b0000_0001;
        set_box(12'd0, 12'd71, 12'd0, 12'd31);     // 像素：9 块宽 x 4 块高
        send_frame;                                 // 预热：建立门控
        #1;
        $display("  [预热] 切出 %0d 个子框", o_nslot);

        sel = 4'd0; send_frame; #1;
        $display("  [槽0] v=%b x=%0d..%0d cnt=%0d sig=(%0d,%0d,%0d,%0d) ar=%b bl=%b  nslot=%0d",
                 o_sel_v, o_sel_x1, o_sel_x2, o_sel_pixcnt,
                 o_sel_colsig[31:24], o_sel_colsig[23:16], o_sel_colsig[15:8], o_sel_colsig[7:0],
                 o_sel_ar_ok, o_sel_blank, o_nslot);
        check(o_nslot === 5'd3,                            "三张并排应切成 3 个子框");
        check(o_sel_v === 1'b1,                            "槽0 有效");
        check(o_sel_x1 === 12'd0 && o_sel_x2 === 12'd23,   "槽0 矩形 = 块0..2（像素 0..23）");
        check(o_sel_y1 === 12'd0 && o_sel_y2 === 12'd31,   "槽0 高度 = 块0..3");
        check(o_sel_pixcnt === 22'd768,                    "槽0 像素数 = 768（24x32 全红）");
        check(o_sel_colsig[31:24] === 8'd192,              "槽0 红桶归一化 = 192");
        check(o_sel_ar_ok === 1'b1,                        "槽0 宽高比 0.75 通过");
        check(o_sel_blank === 1'b0,                        "槽0 非空白");
        // ★ 一整块实心 = 1 个连通块、1 组列峰：阶段 2 在"实心牌面"上的基准行为
        $display("  [槽0] 阶段2: dots=%0d bars=%0d cells=%0d maxc=%0d",
                 o_dots, o_bars, o_cells, o_maxc);
        check(o_dots === 5'd1,                             "实心块 = 1 个连通块");
        check(o_bars === 5'd1,                             "实心块 = 1 组列峰");
        check(o_cells === 9'd48,                           "实心块 = 6x8 = 48 格");
        check(o_maxc === 9'd48,                            "最大连通块 = 48 格");

        sel = 4'd1; send_frame; #1;
        $display("  [槽1] x=%0d..%0d cnt=%0d sig=(%0d,%0d,%0d,%0d)",
                 o_sel_x1, o_sel_x2, o_sel_pixcnt,
                 o_sel_colsig[31:24], o_sel_colsig[23:16], o_sel_colsig[15:8], o_sel_colsig[7:0]);
        check(o_sel_v === 1'b1,                            "槽1 有效");
        check(o_sel_x1 === 12'd24 && o_sel_x2 === 12'd47,  "槽1 矩形 = 块3..5");
        check(o_sel_pixcnt === 22'd768,                    "槽1 像素数 = 768");
        check(o_sel_colsig[7:0] === 8'd192,                "槽1 无彩桶归一化 = 192（暗图案）");

        sel = 4'd2; send_frame; #1;
        $display("  [槽2] x=%0d..%0d cnt=%0d sig=(%0d,%0d,%0d,%0d)",
                 o_sel_x1, o_sel_x2, o_sel_pixcnt,
                 o_sel_colsig[31:24], o_sel_colsig[23:16], o_sel_colsig[15:8], o_sel_colsig[7:0]);
        check(o_sel_v === 1'b1,                            "槽2 有效");
        check(o_sel_x1 === 12'd48 && o_sel_x2 === 12'd71,  "槽2 矩形 = 块6..8");
        check(o_sel_colsig[15:8] === 8'd192,               "槽2 蓝桶归一化 = 192");

        sel = 4'd3; send_frame; #1;
        check(o_sel_v === 1'b0,                            "槽3 无效（只切出 3 个）");

        // ================= 第 2 组：单张牌（mode 0）=================
        set_box(12'd0, 12'd23, 12'd0, 12'd31);
        send_frame;                                 // 预热：换门控
        sel = 4'd0; send_frame; #1;
        $display("  [单张] 切出 %0d 个子框  x=%0d..%0d cnt=%0d", o_nslot, o_sel_x1, o_sel_x2, o_sel_pixcnt);
        check(o_nslot === 5'd1,                            "单张牌应只切 1 个");
        check(o_sel_v === 1'b1,                            "单张 槽0 有效");
        check(o_sel_x1 === 12'd0 && o_sel_x2 === 12'd23,   "单张 槽0 矩形 = 块0..2");
        check(o_sel_pixcnt === 22'd768,                    "单张 槽0 像素数仍为 768（累加器已清零）");
        check(o_sel_ar_ok === 1'b1,                        "单张 槽0 宽高比通过");

        // ================= 第 3 组：四筒（mode 1）=================
        //   子框 96..159 x 0..95 = 8 块 x 12 块（宽高比 0.667 通过）-> N=1
        //   4 个 3x3 格圆点 = 36 格；两列圆点 -> 2 组列峰
        mode = 2'd1;
        set_box(12'd96, 12'd159, 12'd0, 12'd95);
        send_frame;                                 // 预热：建立门控
        sel = 4'd0; send_frame; #1;
        $display("  [四筒] nslot=%0d x=%0d..%0d v=%b | dots=%0d bars=%0d cells=%0d maxc=%0d",
                 o_nslot, o_sel_x1, o_sel_x2, o_sel_v, o_dots, o_bars, o_cells, o_maxc);
        check(o_nslot === 5'd1,                            "四筒：只切 1 个子框");
        check(o_sel_v === 1'b1,                            "四筒：槽0 有效");
        check(o_sel_x1 === 12'd96 && o_sel_x2 === 12'd159, "四筒：子框 = 像素 96..159");
        check(o_sel_ar_ok === 1'b1,                        "四筒：宽高比 64/96 通过");
        check(o_dots === 5'd4,                             "四筒：连通块 = 4");
        check(o_bars === 5'd2,                             "四筒：列投影峰 = 2（两列圆点）");
        check(o_cells === 9'd36,                           "四筒：占据格 = 4x9 = 36");
        check(o_maxc === 9'd9,                             "四筒：最大连通块 = 9 格");

        // ================= 第 4 组：五条（mode 2）=================
        //   5 根 8x48 像素竖条 = 5 个 2x12 格的连通块 = 120 格；5 组列峰
        mode = 2'd2;
        send_frame;                                 // 预热：换门控（框尺寸没变，还是走一遍稳）
        sel = 4'd0; send_frame; #1;
        $display("  [五条] nslot=%0d x=%0d..%0d v=%b | dots=%0d bars=%0d cells=%0d maxc=%0d",
                 o_nslot, o_sel_x1, o_sel_x2, o_sel_v, o_dots, o_bars, o_cells, o_maxc);
        check(o_dots === 5'd5,                             "五条：连通块 = 5");
        check(o_bars === 5'd5,                             "五条：列投影峰 = 5");
        check(o_cells === 9'd120,                          "五条：占据格 = 5x24 = 120");
        check(o_maxc === 9'd24,                            "五条：最大连通块 = 24 格");

        // ================= 第 5 组：一筒 + 噪点（mode 3）=================
        //   1 个圆点（9 格）+ 1 个孤立格 -> 连通块只数 1 个（DOT_MIN=2 滤掉噪点）
        mode = 2'd3;
        send_frame;                                 // 预热
        sel = 4'd0; send_frame; #1;
        $display("  [一筒+噪点] dots=%0d bars=%0d cells=%0d maxc=%0d",
                 o_dots, o_bars, o_cells, o_maxc);
        check(o_dots === 5'd1,                             "噪点被 DOT_MIN 滤掉：连通块 = 1");
        check(o_cells === 9'd10,                           "占据格 = 9（圆点）+ 1（噪点）= 10");
        check(o_maxc === 9'd9,                             "最大连通块 = 9 格（单个圆点）");
        check(o_bars === 5'd1,                             "列投影峰 = 1（噪点高度不够）");

        $display("========================================");
        $display("  帧末 FSM 触发次数：vend=%0d frun=%0d fdone=%0d spdone=%0d gdone=%0d",
                 vend_cnt, frun_cnt, fdone_cnt, spdone_cnt, gdone_cnt);
        if (fails == 0) $display("*** TILE RECOG ALL CHECKS PASSED ***  (通过 %0d 项)", oks);
        else            $display("*** FAILED: %0d 项失败 / 通过 %0d 项 ***", fails, oks);
        $display("========================================");
        $finish;
    end
endmodule
