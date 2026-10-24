`timescale 1ns / 1ps
// ============================================================================
//  bin_ccl.v — 流式连通域标记（Streaming Connected Component Labelling）
// ----------------------------------------------------------------------------
//  **逐行扫描，不需要整帧缓存**。输入是二值像素流（4 像素/拍）。
//
//  ① 游程提取：每拍从 4 位里找 0->1（游程开始）、1->0（游程结束）
//  ② 与上一行游程做区间重叠比较（"只进不退"指针，每拍 1 次比较）
//       重叠   -> 同一连通域 -> Union-Find 合并（**小标签优先**，链很短，不做路径压缩）
//       不重叠 -> 分配新标签
//  ③ 游程结束时把该段并入该标签的统计（包围盒）
//  ④ 行末：当前行游程表 -> 上一行游程表
//  ⑤ 帧末消隐期：按标签**递增**顺序遍历，沿 parent 链求根，把叶子包围盒
//       **原地并到根上**。因为根一定 < 叶子、且递增遍历时根已经处理完，
//       所以原地合并是正确的，不需要第二张表。
//  ⑥ 输出：包围盒宽高都达标的根，按标签顺序写进对象表（最多 MAX_OBJ 个）
//
//  资源：parent = 寄存器 256x8；stat = 寄存器 256x48（随机读写各一次/拍）；
//        行游程表 = 寄存器 MAX_RUN x 32；其余约 800~1200 LUT
// ============================================================================
module bin_ccl #(
    parameter integer MAX_RUN   = 32,
    parameter integer MAX_LABEL = 256,
    parameter integer MAX_OBJ   = 16,
    parameter integer MIN_W     = 20,      // 最小宽度（像素），滤噪点
    parameter integer MIN_H     = 20       // 最小高度（像素）
) (
    input  wire         I_clk,
    input  wire         I_rst_n,
    input  wire [3:0]   I_bin,
    input  wire         I_valid,
    input  wire         I_last,
    input  wire         I_user,
    output reg          O_busy,
    output reg  [3:0]   O_obj_cnt,
    output reg  [191:0] O_obj_x1,     // 对象 k 在 [12k +: 12]
    output reg  [191:0] O_obj_x2,
    output reg  [191:0] O_obj_y1,
    output reg  [191:0] O_obj_y2
);
    localparam [47:0] STAT_EMPTY = {12'hFFF, 12'd0, 12'hFFF, 12'd0};

    // ---------------- 拍号 / 行号 ----------------
    reg [8:0]  beat_q;
    reg [11:0] row_q;
    wire [11:0] col_base = { beat_q, 2'b00 };

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin beat_q <= 9'd0; row_q <= 12'd0; end
        else if (I_valid) begin
            beat_q <= (I_last || I_user) ? 9'd0 : (beat_q + 9'd1);
            if (I_user)      row_q <= 12'd0;          // 帧首必须把行号归零
            else if (I_last) row_q <= row_q + 12'd1;
        end
    end

    // ---------------- 表 ----------------
    reg [47:0] stat   [0:MAX_LABEL-1];   // {x1, x2, y1, y2}
    reg [7:0]  parent [0:MAX_LABEL-1];
    reg [8:0]  lbl_next;

    reg [31:0] cur_run [0:MAX_RUN-1];    // {start[12], end[12], label[8]}
    reg [31:0] prv_run [0:MAX_RUN-1];
    reg [5:0]  cur_n, prv_n;
    reg        in_run;
    reg [11:0] run_start;
    reg [7:0]  run_lab;

    reg [5:0]  pptr;
    wire [31:0] pv    = prv_run[pptr];
    wire [11:0] pv_st = pv[31:20];
    wire [11:0] pv_en = pv[19:8];
    wire [7:0]  pv_lb = pv[7:0];
    wire        pptr_adv = (pptr < prv_n) && (pv_en < col_base);

    // ---------------- 帧末触发：I_valid 连续无效 = 进入消隐 ----------------
    localparam integer GAP = 12;
    reg [5:0] idle_cnt;
    reg       done_f;                 // 本帧已处理过
    wire      frame_gap = (idle_cnt >= GAP) && (row_q != 12'd0) && !done_f;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin idle_cnt <= 6'd0; done_f <= 1'b0; end
        else begin
            if (I_valid)        idle_cnt <= 6'd0;
            else if (idle_cnt < 6'd63) idle_cnt <= idle_cnt + 6'd1;
            if (I_valid && I_user) done_f <= 1'b0;      // 新帧开始 -> 允许再次触发
            else if (st != S_SCAN) done_f <= 1'b1;      // 帧末处理中 -> 标记已处理
        end
    end

    // ---------------- 沿提取 ----------------
    reg  prev_b3;
    wire [4:0] ext  = { I_bin, prev_b3 };
    wire [3:0] rise = ~ext[4:1] & ext[3:0];
    wire [3:0] fall =  ext[4:1] & ~ext[3:0];
    wire       any_rise = |rise;
    wire       any_fall = |fall;
    wire [1:0] rise_pos = rise[0] ? 2'd0 : rise[1] ? 2'd1 : rise[2] ? 2'd2 : 2'd3;
    wire [1:0] fall_pos = fall[0] ? 2'd0 : fall[1] ? 2'd1 : fall[2] ? 2'd2 : 2'd3;
    wire [11:0] fall_col = col_base + {10'd0, fall_pos};
    wire [11:0] rise_col = col_base + {10'd0, rise_pos};

    // ---------------- 帧末状态机 ----------------
    localparam [2:0] S_SCAN = 3'd0, S_ROOT = 3'd1, S_MERGE = 3'd2, S_EMIT = 3'd3;
    reg [2:0]  st;
    reg [8:0]  fi;
    reg [8:0]  fj;
    reg        f_first;
    reg        f_isroot;
    reg [7:0]  f_root;
    reg [3:0]  f_emit;
    reg [59:0] sl;                    // 叶子 stat 缓存
    reg [59:0] sr;                    // 根 stat 缓存
    integer i;

    wire [11:0] sl_x1 = sl[59:48], sl_x2 = sl[47:36], sl_y1 = sl[35:24], sl_y2 = sl[23:12];
    wire [11:0] sr_x1 = sr[59:48], sr_x2 = sr[47:36], sr_y1 = sr[35:24], sr_y2 = sr[23:12];

    // 统计表里的一项按 x1 是否为 FFF 判空
    wire [8:0] ridx = f_isroot ? fi : {1'b0, f_root};
    wire [47:0] st_leaf = stat[fi];
    wire [47:0] st_root = stat[ridx[7:0]];

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st <= S_SCAN; fi <= 9'd0; fj <= 9'd0; f_first <= 1'b0; f_isroot <= 1'b0; f_root <= 8'd0;
            f_emit <= 4'd0; sl <= 60'd0; sr <= 60'd0;
            cur_n <= 6'd0; prv_n <= 6'd0; in_run <= 1'b0; run_start <= 12'd0; run_lab <= 8'd0;
            lbl_next <= 9'd1; pptr <= 6'd0; prev_b3 <= 1'b0;
            O_busy <= 1'b0; O_obj_cnt <= 4'd0;
            O_obj_x1 <= 192'd0; O_obj_x2 <= 192'd0; O_obj_y1 <= 192'd0; O_obj_y2 <= 192'd0;
            for (i = 0; i < MAX_LABEL; i = i + 1) begin
                stat[i]   <= STAT_EMPTY;
                parent[i] <= i[7:0];
            end
        end else case (st)
        // ============================================================== 逐行扫描
        S_SCAN: begin
            // 新帧第一个拍：把 parent 复位成自环、stat 清空（为下一帧准备）
            if (I_valid && I_user) begin
                for (i = 0; i < MAX_LABEL; i = i + 1) begin
                    parent[i] <= i[7:0];
                    stat[i]   <= STAT_EMPTY;
                end
            end
            if (I_valid) begin
                prev_b3 <= I_bin[3];
                if (pptr_adv) pptr <= pptr + 6'd1;

                // ---- 游程结束：记表 + 统计 + UNION ----
                if (any_fall && in_run) begin
                    if (cur_n < MAX_RUN) begin
                        cur_run[cur_n] <= { run_start, fall_col, run_lab };
                        cur_n <= cur_n + 6'd1;
                    end
                    // 与上一行重叠 -> union（小标签优先）
                    if ((pptr < prv_n) && (pv_st <= fall_col) && (pv_en >= run_start) &&
                        (pv_lb != run_lab)) begin
                        if (pv_lb < run_lab) parent[run_lab] <= pv_lb;
                        else                 parent[pv_lb]  <= run_lab;
                    end
                    // 统计：把 [run_start, fall_col] 这一段并进 run_lab 的包围盒
                    if (st_leaf_x1_of(run_lab) == 12'hFFF) begin
                        stat[run_lab] <= { run_start, fall_col, row_q, row_q };
                    end else begin
                        stat[run_lab][47:36] <= (run_start < stat[run_lab][47:36]) ? run_start : stat[run_lab][47:36];
                        stat[run_lab][35:24] <= (fall_col  > stat[run_lab][35:24]) ? fall_col  : stat[run_lab][35:24];
                        stat[run_lab][23:12] <= (row_q     < stat[run_lab][23:12]) ? row_q     : stat[run_lab][23:12];
                        stat[run_lab][11:0]  <= row_q;
                    end
                    in_run <= 1'b0;
                end

                // ---- 游程开始：分配新标签 ----
                if (any_rise) begin
                    run_start <= rise_col;
                    run_lab   <= lbl_next[7:0];
                    in_run    <= 1'b1;
                    if (lbl_next < (MAX_LABEL - 1)) lbl_next <= lbl_next + 9'd1;
                end

                // ---- 行末：行表轮转 ----
                if (I_last) begin
                    for (i = 0; i < MAX_RUN; i = i + 1) prv_run[i] <= cur_run[i];
                    prv_n  <= cur_n;
                    cur_n  <= 6'd0;
                    pptr   <= 6'd0;
                    in_run <= 1'b0;
                    prev_b3<= 1'b0;
                end
            end
            // ---- 帧末触发：进入消隐（I_valid 连续无效）----
            if (frame_gap) begin
                st <= S_ROOT; fi <= 9'd0; f_first <= 1'b1; f_emit <= 4'd0;
                O_busy <= 1'b1;
                lbl_next <= 9'd1;          // 标签从 1 重新分配（parent 在输出后复位）
            end
        end
        // ========================================================== 求根（链上游走）
        S_ROOT: begin
            if (f_first) begin
                fj <= fi; f_first <= 1'b0;
            end else if (parent[fj] == fj[7:0]) begin
                f_root   <= fj[7:0];
                f_isroot <= (fj == fi);
                sr       <= {12'd0, stat[fj]};
                sl       <= {12'd0, stat[fi]};
                st       <= S_MERGE;
            end else begin
                fj <= parent[fj];
            end
        end
        // ========================================================== 合并（原地，递增序）
        S_MERGE: begin
            // 叶子非空、且不是根本身 -> 把叶子并进根（原地合并，递增遍历保证正确）
            if (!f_isroot && (sl_x1 != 12'hFFF) && (sr_x1 != 12'hFFF)) begin
                stat[f_root] <= {
                    (sl_x1 < sr_x1) ? sl_x1 : sr_x1,
                    (sl_x2 > sr_x2) ? sl_x2 : sr_x2,
                    (sl_y1 < sr_y1) ? sl_y1 : sr_y1,
                    (sl_y2 > sr_y2) ? sl_y2 : sr_y2 };
            end else if (!f_isroot && (sl_x1 != 12'hFFF) && (sr_x1 == 12'hFFF)) begin
                stat[f_root] <= st_leaf;          // 根还是空的 -> 直接搬过去
            end
            if (fi >= (MAX_LABEL - 1)) begin
                st <= S_EMIT; fi <= 9'd0;
            end else begin
                fi <= fi + 9'd1; f_first <= 1'b1; st <= S_ROOT;
            end
        end
        // ========================================================== 输出（宽高达标）
        S_EMIT: begin
            if ((parent[fi] == fi[7:0]) && (stat[fi][47:36] != 12'hFFF)) begin
                if (((stat[fi][35:24] - stat[fi][47:36] + 12'd1) >= MIN_W[11:0]) &&
                    ((stat[fi][11:0]  - stat[fi][23:12] + 12'd1) >= MIN_H[11:0]) &&
                    (f_emit < MAX_OBJ)) begin
                    O_obj_x1[f_emit*12 +: 12] <= stat[fi][47:36];
                    O_obj_x2[f_emit*12 +: 12] <= stat[fi][35:24];
                    O_obj_y1[f_emit*12 +: 12] <= stat[fi][23:12];
                    O_obj_y2[f_emit*12 +: 12] <= stat[fi][11:0];
                    f_emit <= f_emit + 4'd1;
                end
            end
            if (fi >= (MAX_LABEL - 1)) begin
                O_obj_cnt <= f_emit;
                O_busy    <= 1'b0;
                st        <= S_SCAN;
            end else fi <= fi + 9'd1;
        end
        default: st <= S_SCAN;
        endcase
    end

    // 辅助：某个标签的 x1（判断是否空）
    function [11:0] st_leaf_x1_of;
        input [7:0] L;
        begin st_leaf_x1_of = stat[L][47:36]; end
    endfunction
endmodule
