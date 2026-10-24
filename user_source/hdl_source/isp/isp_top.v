module isp_top (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    // ---- Sobel 运行期控制（由按键/拨码给出）----
    input 			I_sobel_en		,      // 1 = 输出二值/边缘图，0 = 输出原 RGB 彩图
    input [7:0] 	I_sobel_y_thresh,      // 二值化阈值 0..255
    input [3:0] 	I_sobel_g_thresh,      // 边缘判定阈值 |gx|+|gy|，有效 1..8
    // ---- 算法观测点选择（SW2/SW3/SW4）----
    //   00 = 不观测（按 I_sobel_en 走彩图或完整链路）
    //   01 = 二值化（无行缓存）  10 = Sobel 边沿（3 级行缓存）  11 = Y 灰度（无行缓存）
    input [1:0] 	I_dbg_sel		,
    input 			I_bnd_en		,      // 1 = 边缘图叠加红色矩形边界（SW1）
    input [7:0] 	I_bnd_ratio	,      // KEY2 长按：填充阈值档位 0..160（x8）
   input 		I_k4_en			,      // KEY4：计数器使能
   output [4:0] 	O_line_cnt		,      // 宽度竖线条数
   output [191:0]	O_line_x			,      // 宽度竖线坐标列表
   output [11:0] 	O_fill_h		,      // KEY4 计数器值（被填充行数最大值）
   output [31:0] 	O_y_out			,      // 帧差法用：YCbCr 的 Y（4 像素/拍）
   output 			O_y_valid		,      // 与 O_y_out 同拍
   output 			O_y_last		,      // 与 O_y_out 同拍
   output 			O_y_user		,      // 与 O_y_out 同拍
   // ★ 麻将识别用：4 个像素的色系码（2 位/像素），与 O_y_out 同拍、零乘法
   output [7:0] 	O_col4			,      // 0红 1绿 2蓝 3无彩
    input [1:0] 	I_mid_ratio	,      // 投影阈值比例（KEY4 长按）
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready		,
    // ↓ 峰值区域（ROI）对外接口，供后续程序使用（上一帧的结果）
    input  [3:0]  	I_roi_idx		,      // 选第几组**列段**（外部可遍历所有高峰）
    input  [3:0]  	I_roi_yidx		,      // 选第几组**行段**
    output [3:0]  	O_roi_cnt		,      // 识别到几组列段
    output [3:0]  	O_rcnt			,      // 识别到几组行段
    output [11:0] 	O_roi_x1		,      // 左边界列号
    output [11:0] 	O_roi_x2		,      // 右边界列号
    output [11:0] 	O_roi_y1		,      // 峰内第一行
    output [11:0] 	O_roi_y2		,      // 峰内最后一行
    output 			O_roi_valid		,      // 本帧是否存在峰值区域
   output [11:0] 	O_roi_h		,      // 上下边界之间的长度（下-上+1）
   output [11:0] 	O_roi_yt		,      // 上边界（第一行高峰）
   output [11:0] 	O_roi_yb		       // 下边界（最后一行高峰）
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 
  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 
  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

  // ---- blc_raw40 黑电平校正（RAW10，4 像素/拍 40bit）----
  wire [39:0] blc_O_tdata;
  wire        blc_O_tlast;
  wire        blc_O_tuser;
  wire        blc_O_tvalid;

  // ---- Gamma 校正（RGB888，4 像素/拍 96bit，γ≈0.7）----
  wire [95:0] gamma_O_tdata;
  wire        gamma_O_tlast;
  wire        gamma_O_tuser;
  wire        gamma_O_tvalid;

  // ---- Sobel 边缘检测（128 位，4 像素/拍）----
  wire [127:0] sob_in_128;
  wire [127:0] sob_out_128;
  wire [ 95:0] sob_out_96;
  wire         sob_O_tlast;
  wire         sob_O_tuser;
  wire         sob_O_tvalid;
  // 送进 96->128 打包器的最终数据（I_sobel_en=0 时直通 gamma，输出原彩图）
  // 只要接了观测点（SW2/SW3/SW4），也走 Sobel 那一路 —— 观测点是在 Sobel 模块内部选出来的
  wire         S_path_sel  = I_sobel_en | (I_dbg_sel != 2'b00);
  wire [ 95:0] path_tdata  = S_path_sel ? sob_out_96   : gamma_O_tdata;
  wire         path_tuser  = S_path_sel ? sob_O_tuser  : gamma_O_tuser;
  wire         path_tvalid = S_path_sel ? sob_O_tvalid : gamma_O_tvalid;


  assign awb_O_tready = O_tready;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = awb_O_tlast    ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
// ==================================================================
// 黑电平校正：RAW10 域，位于 demosaic 之前。
// 减去传感器黑电平偏置，把暗部压到真正的 0，提升暗部对比度。
// 偏置默认 15（10bit 域，约 15/1023）：暗部仍发灰就加大，
// 暗部出现死黑/细节丢失就减小。
// 模块 blc_raw40 定义在本文件末尾（与 axis_gamma_rgb96 同样的内联方式），
// 这样不需要往 TD 工程的 .al 里添加文件，避免 black box 报错。
// ==================================================================
blc_raw40 #(
    .Black_level_offset_r0 (10'd15),
    .Black_level_offset_r1 (10'd15),
    .Black_level_offset_r2 (10'd15),
    .Black_level_offset_r3 (10'd15)
) u_blc (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n         ),
    .I_tlast (I_tlast         ),
    .I_tuser (I_tuser         ),
    .I_tdata (I_tdata         ),
    .I_tvalid(I_tvalid        ),
    .O_tlast (blc_O_tlast     ),
    .O_tuser (blc_O_tuser     ),
    .O_tdata (blc_O_tdata     ),
    .O_tvalid(blc_O_tvalid    )
);

demosaic #(
    .IMG_HEIGHT          (720),  // 图像高度（1080p60）
    .IMG_WIDTH           (1280),  // 图像宽度（1080p60）
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(blc_O_tdata) 	,  // AXI4-Stream视频数据（经 blc_raw40 黑电平校正）
    .axi4s_video_tdest 	(I_tdest)     	,
    .axi4s_video_tlast 	(blc_O_tlast) 	,  // 行结束信号
    .axi4s_video_tvalid	(blc_O_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(blc_O_tuser) 	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)    	,  // 从模块准备好接受数据（输出给上游）
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

// ==================================================================
// Gamma 校正：RGB888 域，位于 awb 之后、96→128bit 打包之前。
// 放在 awb 之后的原因：awb 是逐帧灰度世界算法，靠 R/G/B 均值算增益；
// 若把 gamma 放在它前面，gamma 会改变均值、被 awb 部分抵消。
// 正确顺序是"先白平衡、后色调曲线"。
// 该模块是纯流水线（无 tready），而本 ISP 通路的 ready 链全部是
// assign I_tready = O_tready 的直通、不参与逻辑，因此插入安全。
// 曲线为 16 段折线，近似 y = 255*(x/255)^0.7（与理想曲线最大偏差约 4.9 LSB）：
//   0x40->0x61(×1.52)、0x80->0x9d(×1.23)、0xc0->0xd1(×1.09)
//   0x20->0x3c(×1.88)、0x10->0x25(×2.31)，越暗提亮越多；
//   0x00->0x00、0xff->0xff，保黑保白。
// ==================================================================
axis_gamma_rgb96 u_gamma (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n         ),
    .I_tlast (awb_O_tlast     ),
    .I_tuser (awb_O_tuser     ),
    .I_tdata (awb_O_tdata     ),
    .I_tvalid(awb_O_tvalid    ),
    .O_tlast (gamma_O_tlast   ),
    .O_tuser (gamma_O_tuser   ),
    .O_tdata (gamma_O_tdata   ),
    .O_tvalid(gamma_O_tvalid  )
);


// ==================================================================
// Sobel 边缘检测：位于 DDR2 写入之前（gamma 之后、96→128bit 打包之前）。
//   gamma(96) → data96_128_pa（纯组合）→ sobel_edge_128（128 位，4 像素/拍）
//             → data128_96（纯组合）→ data_96bit_to_128bit → video_in → DDR2
//
// 两个位宽转换都是纯组合 assign，往返零延迟，所以整条支路只增加 Sobel 自身
// 的 5 拍流水；帧内拍数完全不变（行缓存边界用"复制"而非补 0，
// 不会在行的最左/最右列产生假边缘；帧内前 2 行无法形成 3x3 窗口，输出全黑）。
//
// 行缓存用 PH1P_LOGIC_ERAM 原语，强制落在 BRAM（IMPLEMENT="20K(FAST)"）。
// 模块定义在本文件末尾（与 blc_raw40 / axis_gamma_rgb96 一样内联，
// 这样不需要往 TD 工程的 .al 里添加文件，避免 black box）。
// 独立可移植版本：user_source/hdl_source/isp/sobel_edge_128.v
// ==================================================================
data96_128_pa u_sob_pack (
    .I_tdata (gamma_O_tdata),
    .O_tdata (sob_in_128   )
);

sobel_edge_128 #(
    .IMG_WIDTH (1280),
    .IMG_HEIGHT(720)
) u_sobel (
    .I_clk      (axi4s_video_aclk  ),
    .I_rst_n    (I_rst_n           ),
    .I_tdata    (sob_in_128        ),
    .I_tvalid   (gamma_O_tvalid    ),
    .I_tlast    (gamma_O_tlast     ),
    .I_tuser    (gamma_O_tuser     ),
    .I_y_thresh (I_sobel_y_thresh  ),
    .I_g_thresh (I_sobel_g_thresh  ),
    .I_dbg_sel  (I_dbg_sel         ),
   .I_bnd_en   (I_bnd_en          ),
   .I_bnd_ratio(I_bnd_ratio       ),
        .I_k4_en    (I_k4_en           ),
        .O_line_cnt (O_line_cnt        ),
        .O_line_x   (O_line_x          ),
        .O_fill_h   (O_fill_h          ),
        .O_y_out    (O_y_out           ),
        .O_y_valid  (O_y_valid         ),
        .O_y_last   (O_y_last          ),
        .O_y_user   (O_y_user          ),
        .O_col4     (O_col4            ),   // ★ 麻将识别用色系码（与 Y 同拍）
   .I_mid_ratio(I_mid_ratio       ),
    .O_tdata    (sob_out_128       ),
    .O_tvalid   (sob_O_tvalid      ),
    .O_tlast    (sob_O_tlast       ),
    .O_tuser    (sob_O_tuser       ),
    // 峰值区域（ROI）对外接口（上一帧的结果），供后续程序使用
    .O_roi_x1   (O_roi_x1          ),
    .O_roi_x2   (O_roi_x2          ),
    .O_roi_y1   (O_roi_y1          ),
    .O_roi_y2   (O_roi_y2          ),
    .O_roi_valid(O_roi_valid       ),
    .I_roi_idx  (I_roi_idx         ),
    .I_roi_yidx (I_roi_yidx        ),
    .O_roi_cnt  (O_roi_cnt         ),
    .O_rcnt     (O_rcnt            ),
    .O_roi_h    (O_roi_h           )
);
// 上下边界直接从 ROI 接口引出（O_roi_y1/O_roi_y2 就是第一行/最后一行高峰）
assign O_roi_yt = O_roi_y1;
assign O_roi_yb = O_roi_y2;

data128_96 u_sob_unpack (
    .I_tdata (sob_out_128),
    .O_tdata (sob_out_96 )
);


    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),
	
        .I_96b_frame_start  ( path_tuser        ),
        .I_96b_valid        ( path_tvalid       ),
        .I_96b_data         ( path_tdata        ),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule


// Four-pixel RGB888 gamma stage.  The 16-segment curve approximates
// y = 255 * (x / 255)^0.7.  It lifts shadows and mid-tones while preserving
// black and white and adds one registered pipeline cycle.
module axis_gamma_rgb96 (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    output reg         O_tlast,
    output reg         O_tuser,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid
);

    function [7:0] gamma07;
        input [7:0] value;
        reg [7:0] base;
        reg [7:0] delta;
        reg [11:0] interpolation;
        begin
            case (value[7:4])
                4'h0: begin base = 8'd0;   delta = 8'd37; end
                4'h1: begin base = 8'd37;  delta = 8'd23; end
                4'h2: begin base = 8'd60;  delta = 8'd19; end
                4'h3: begin base = 8'd79;  delta = 8'd18; end
                4'h4: begin base = 8'd97;  delta = 8'd16; end
                4'h5: begin base = 8'd113; delta = 8'd16; end
                4'h6: begin base = 8'd129; delta = 8'd14; end
                4'h7: begin base = 8'd143; delta = 8'd14; end
                4'h8: begin base = 8'd157; delta = 8'd14; end
                4'h9: begin base = 8'd171; delta = 8'd13; end
                4'ha: begin base = 8'd184; delta = 8'd13; end
                4'hb: begin base = 8'd197; delta = 8'd12; end
                4'hc: begin base = 8'd209; delta = 8'd12; end
                4'hd: begin base = 8'd221; delta = 8'd12; end
                4'he: begin base = 8'd233; delta = 8'd11; end
                default: begin base = 8'd244; delta = 8'd11; end
            endcase

            interpolation = delta * value[3:0] + 8;
            if (value == 8'hff)
                gamma07 = 8'hff;
            else
                gamma07 = base + (interpolation >> 4);
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
        end else begin
            O_tlast  <= I_tlast;
            O_tuser  <= I_tuser;
            O_tvalid <= I_tvalid;
            if (I_tvalid) begin
                O_tdata <= {
                    gamma07(I_tdata[95:88]), gamma07(I_tdata[87:80]), gamma07(I_tdata[79:72]),
                    gamma07(I_tdata[71:64]), gamma07(I_tdata[63:56]), gamma07(I_tdata[55:48]),
                    gamma07(I_tdata[47:40]), gamma07(I_tdata[39:32]), gamma07(I_tdata[31:24]),
                    gamma07(I_tdata[23:16]), gamma07(I_tdata[15:8]),  gamma07(I_tdata[7:0])
                };
            end else begin
                O_tdata <= 96'd0;
            end
        end
    end

endmodule


// Centre crop for the ISP's four-pixel-wide RGB888 stream.  Frame start is
// intentionally forwarded at the original frame boundary so video_in can
// reset its asynchronous FIFO before the first selected row arrives.
module axis_crop_rgb96 #(
    parameter integer IN_WIDTH   = 1792,
    parameter integer IN_HEIGHT  = 720,
    parameter integer OUT_WIDTH  = 1600,
    parameter integer OUT_HEIGHT = 900,
    parameter integer X_START    = 96,
    parameter integer Y_START    = 90
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [95:0] O_tdata,
    output wire        O_tvalid
);

    localparam integer X_START_BEAT = X_START / 4;
    localparam integer OUT_BEATS    = OUT_WIDTH / 4;
    localparam integer X_END_BEAT   = X_START_BEAT + OUT_BEATS;

    reg [9:0]  S_hcnt;
    reg [10:0] S_vcnt;

    wire [9:0]  S_current_h = I_tuser ? 10'd0 : S_hcnt;
    wire [10:0] S_current_v = I_tuser ? 11'd0 : S_vcnt;
    wire S_inside = (S_current_h >= X_START_BEAT) &&
                    (S_current_h < X_END_BEAT) &&
                    (S_current_v >= Y_START) &&
                    (S_current_v < (Y_START + OUT_HEIGHT));

    assign O_tdata  = I_tdata;
    assign O_tvalid = I_tvalid && S_inside;
    assign O_tlast  = O_tvalid && (S_current_h == (X_END_BEAT - 1));
    assign O_tuser  = I_tuser;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_hcnt <= 10'd0;
            S_vcnt <= 11'd0;
        end
        else if(I_tuser) begin
            S_hcnt <= (I_tvalid && !I_tlast) ? 10'd1 : 10'd0;
            S_vcnt <= (I_tvalid && I_tlast) ? 11'd1 : 11'd0;
        end
        else if(I_tvalid) begin
            if(I_tlast) begin
                S_hcnt <= 10'd0;
                S_vcnt <= (S_vcnt == IN_HEIGHT - 1) ? 11'd0 : S_vcnt + 1'b1;
            end
            else begin
                S_hcnt <= S_hcnt + 1'b1;
            end
        end
    end

endmodule

// ==================================================================
// 黑电平校正（RAW10，4 像素/拍 40bit）。
//
// 功能与 user_source/hdl_source/isp/BLC/BLC.v 完全一致：逐通道减去黑电平
// 偏置并下钳位到 0，一级流水，tlast/tuser/tvalid 同步延后 1 拍。
//
// 为什么不直接用 BLC.v：
//   BLC.v 不在 TD 工程源文件列表（.al）里，从外部往 .al 补条目会被 TD 覆盖，
//   综合时 BLC 就成了 black box（HDL-7144 / HDL-8007）。
//   本工程原有的 axis_gamma_rgb96 也是同样处理——直接定义在 isp_top.v 内部，
//   而 isp_top.v 本来就在工程列表里，因此无需改动 .al，工程迁移也不会丢。
//   模块名特意取 blc_raw40 而不是 BLC，避免将来把 BLC.v 加进工程时模块重名冲突。
// ==================================================================
module blc_raw40 #(
    parameter [9:0] Black_level_offset_r0 = 10'd15,
    parameter [9:0] Black_level_offset_r1 = 10'd15,
    parameter [9:0] Black_level_offset_r2 = 10'd15,
    parameter [9:0] Black_level_offset_r3 = 10'd15
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [39:0] I_tdata,
    input  wire        I_tvalid,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [39:0] O_tdata,
    output wire        O_tvalid
);
    wire [9:0] off0 = Black_level_offset_r0;
    wire [9:0] off1 = Black_level_offset_r1;
    wire [9:0] off2 = Black_level_offset_r2;
    wire [9:0] off3 = Black_level_offset_r3;

    reg [9:0]  dout [3:0];
    reg        r_tlast, r_tuser, r_tvalid;

    genvar i;
    generate for (i = 0; i < 4; i = i + 1) begin : BLC_LANE
        wire [9:0] din  = I_tdata[i*10 +: 10];
        wire [9:0] offv = (i == 0) ? off0 :
                          (i == 1) ? off1 :
                          (i == 2) ? off2 : off3;
        always @(posedge I_clk) begin
            if (I_tvalid) begin
                // 减黑电平并下钳位到 0，防止出现负数
                dout[i] <= (din > offv) ? (din - offv) : 10'd0;
            end
        end
    end
    endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            {r_tlast, r_tuser, r_tvalid} <= 3'b000;
        else
            {r_tlast, r_tuser, r_tvalid} <= {I_tlast, I_tuser, I_tvalid};
    end

    assign O_tdata  = {dout[3], dout[2], dout[1], dout[0]};
    assign O_tlast  = r_tlast;
    assign O_tuser  = r_tuser;
    assign O_tvalid = r_tvalid;

endmodule

























































































































































































// ===== BEGIN inline sobel_edge_128.v (由 sim/inline_sobel.py 生成，请勿手改) =====
// ============================================================================
//  sobel_edge_128.v — 二值化形态学 + Sobel 边缘检测（128 位宽，4 像素/拍）
// ============================================================================
//  数据流（按用户指定的顺序：先开运算 -> Sobel -> 再闭运算）：
//
//      RGB888 -> YCbCr(Y) -> 二值化 -> 开运算 -> Sobel -> 闭运算 -> 24bit 0/1
//
//  128 位格式（与本工程 demosaic / data128_96 一致）：
//      I_tdata[32*i +: 32] = {8'h00, R[7:0], G[7:0], B[7:0]}    i = 0..3
//  输出同格式：边缘=1 -> 24'hFFFFFF，非边缘=0 -> 24'h000000
//
//  插入位置：DDR2 写入之前。本工程接在 gamma 之后：
//      gamma(96) -> data96_128_pa(组合) -> sobel_edge_128 -> data128_96(组合)
//                -> data_96bit_to_128bit -> video_in -> DDR2
//
//  ---------------------------------------------------------------------------
//  五级流水，每级都是"行缓存 + 3x3 窗口"（模块 bin_rowwin）
//  ---------------------------------------------------------------------------
//      binarize ─► [win1] ─► 腐蚀 ─► [win2] ─► 膨胀 ─► [win3] ─► Sobel
//                ─► [win4] ─► 膨胀 ─► [win5] ─► 腐蚀 ─► 输出
//                └──── 开运算 ────┘                  └──── 闭运算 ────┘
//
//  为什么开/闭要各占两级：
//      一个 bin_rowwin 只能提供"某一幅图像"的 3x3 邻域，即只能做一次 3x3 运算。
//      开运算 = 腐蚀 + 膨胀 = 2 次运算 -> 2 级；闭运算同理 -> 2 级；Sobel 1 级。
//      合计 5 级。单次腐蚀或膨胀才能"借用"已有窗口（那才是零缓存）。
//
//  语义：
//      开运算（Sobel 前）: 去掉二值图里的孤立白点，让边缘更干净
//      闭运算（Sobel 后）: 对边缘图做闭运算 -> 连接断裂的边缘线 / 补边缘缺口
//
//  行缓存用 TD 的 PH1P_LOGIC_ERAM 原语（IMPLEMENT="20K(FAST)"），落在 BRAM
//  而不是分布式 LUTRAM。每个 bin_rowwin 用两块（按行奇偶分开放），共 10 块。
//  器件 PH1P35MDG324 共 108 块 ERAM，实测占用 39 -> 改动后约 47（43.5%）。
//
//  ---------------------------------------------------------------------------
//  边界处理（都做过仿真验证，补 0 会产生假边缘）
//  ---------------------------------------------------------------------------
//      行最左列：左邻复制本拍第一个像素
//      行最右列：右邻复制本拍最后一个像素
//      左右替换都在"计算级"组合完成，不写回移位链
//      帧内前 5 行 / 最后 5 行：窗口凑不齐 -> 输出全黑，但**拍数不变**
//
//  总延迟：5 行 + 约 15 拍。输出第 i 个行槽对应最终结果第 i-5 行。
//
//  综合注意：若 TD 不接受 PH1P_LOGIC_ERAM 直接例化，在文件头加
//      `define SOBEL_LINE_INFER
//  仿真时 iverilog 不认该原语，需一起编译 sim/prim_ph1p_eram_sim.v。
// ============================================================================

// ----------------------------------------------------------------------------
//  [1/5] RGB888 -> YCbCr，只取 Y（4 像素并行）
//        Y = (77*R + 150*G + 29*B) >> 8   （BT.601 整数近似）
// ----------------------------------------------------------------------------
module rgb_to_ycbcr_4 (
    input  wire        I_clk,
    input  wire [127:0] I_tdata,
    output reg  [31:0]  O_y,
    output reg  [31:0]  O_cb,
    output reg  [31:0]  O_cr
);
    function [7:0] calc_y;
        input [23:0] rgb;
        reg   [15:0] s;
        begin
            s = 16'd77 * rgb[23:16] + 16'd150 * rgb[15:8] + 16'd29 * rgb[7:0];
            calc_y = s[15:8];
        end
    endfunction

    function [7:0] calc_cb;
        input [23:0] rgb;
        reg   signed [16:0] s;
        begin
            s = -17'sd43 * $signed({1'b0, rgb[23:16]})
                - 17'sd85 * $signed({1'b0, rgb[15:8]})
                + 17'sd128 * $signed({1'b0, rgb[7:0]});
            calc_cb = s[15:8] + 8'd128;
        end
    endfunction

    function [7:0] calc_cr;
        input [23:0] rgb;
        reg   signed [16:0] s;
        begin
            s =  17'sd128 * $signed({1'b0, rgb[23:16]})
               - 17'sd107 * $signed({1'b0, rgb[15:8]})
               -  17'sd21 * $signed({1'b0, rgb[7:0]});
            calc_cr = s[15:8] + 8'd128;
        end
    endfunction

    always @(posedge I_clk) begin
        O_y  <= { calc_y (I_tdata[127:96]), calc_y (I_tdata[95:64]),
                  calc_y (I_tdata[63:32]),  calc_y (I_tdata[31:0]) };
        O_cb <= { calc_cb(I_tdata[127:96]), calc_cb(I_tdata[95:64]),
                  calc_cb(I_tdata[63:32]),  calc_cb(I_tdata[31:0]) };
        O_cr <= { calc_cr(I_tdata[127:96]), calc_cr(I_tdata[95:64]),
                  calc_cr(I_tdata[63:32]),  calc_cr(I_tdata[31:0]) };
    end
endmodule

// ----------------------------------------------------------------------------
//  [2/5] 二值化：Y >= I_thresh -> 1，否则 0（阈值运行期可调）
// ----------------------------------------------------------------------------
module binarize_4 (
    input  wire        I_clk,
    input  wire [31:0] I_y,
    input  wire [7:0]  I_thresh,
    output reg  [3:0]  O_bin
);
    always @(posedge I_clk)
        O_bin <= { (I_y[31:24] >= I_thresh), (I_y[23:16] >= I_thresh),
                   (I_y[15:8]  >= I_thresh), (I_y[7:0]   >= I_thresh) };
endmodule

// ----------------------------------------------------------------------------
//  [3/5] 行缓存：包一层，强制用 BRAM（PH1P_LOGIC_ERAM, 20K 块）
// ----------------------------------------------------------------------------
module sobel_line_ram (
    input  wire       I_clk,
    input  wire       I_we,
    input  wire [8:0] I_waddr,
    input  wire [3:0] I_wdata,
    input  wire [8:0] I_raddr,
    output wire [3:0] O_rdata
);
`ifdef SOBEL_LINE_INFER
    reg [3:0] mem [0:511];
    reg [3:0] rdata_r;
    always @(posedge I_clk) begin
        if (I_we) mem[I_waddr] <= I_wdata;
        rdata_r <= mem[I_raddr];
    end
    assign O_rdata = rdata_r;
`else
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A      (4),
        .ADDR_WIDTH_A      (9),
        .DATA_DEPTH_A      (512),
        .DATA_WIDTH_B      (4),
        .ADDR_WIDTH_B      (9),
        .DATA_DEPTH_B      (512),
        .MODE              ("DP"),
        .REGMODE_A         ("NOREG"),
        .REGMODE_B         ("NOREG"),
        .WRITEMODE_A       ("NORMAL"),
        .WRITEMODE_B       ("READBEFOREWRITE"),
        .IMPLEMENT         ("20K(FAST)"),
        .ECC_ENCODE        ("DISABLE"),
        .ECC_DECODE        ("DISABLE"),
        .CLKMODE           ("ASYNC"),
        .SSROVERCE         ("DISABLE"),
        .OREGSET_A         ("RESET"),
        .OREGSET_B         ("RESET"),
        .RESETMODE_A       ("ASYNC"),
        .RESETMODE_B       ("ASYNC"),
        .ASYNC_RESET_RELEASE_A ("ASYNC"),
        .ASYNC_RESET_RELEASE_B ("ASYNC"),
        .INIT_FILE         ("NONE"),
        .FILL_ALL          ("NONE")
    ) u_eram (
        .dia   (I_wdata),
        .dib   (4'd0),
        .addra (I_waddr),
        .addrb (I_raddr),
        .cea   (1'b1),
        .ceb   (1'b1),
        .ocea  (1'b0),
        .oceb  (1'b0),
        .clka  (I_clk),
        .clkb  (I_clk),
        .wea   (I_we),
        .web   (1'b0),
        .bea   (1'b0),
        .beb   (1'b0),
        .rsta  (1'b0),
        .rstb  (1'b0),
        .doa   (),
        .dob   (O_rdata),
        .ecc_sbiterr    (),
        .ecc_dbiterr    (),
        .ecc_sbiterrinj (1'b0),
        .ecc_dbiterrinj (1'b0)
    );
`endif
endmodule

// ----------------------------------------------------------------------------
//  [4/5] 二值流的"行延迟 + 3x3 窗口"（可复用单元，本文件例化 5 次）
// ----------------------------------------------------------------------------
//  输入：4 像素/拍的二值字（bit m = 第 m 个像素）+ 控制
//  输出：3 行 x 6 列的窗口 wt/wm/wb，以及与之对齐的控制
//        win6[x]：win6[0] = 该拍第 0 像素的左邻(4b-1 列)
//                  win6[5] = 该拍第 3 像素的右邻(4b+4 列)
//        对第 m 个像素：左邻 = win6[m]，自身 = win6[m+1]，右邻 = win6[m+2]
//  输出行号 O_row = I_row - 1（窗口中心行）；内部已做左右边界"复制"替换
// ----------------------------------------------------------------------------
module bin_rowwin (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [3:0]  I_bin,
    input  wire        I_valid,
    input  wire        I_last,
    input  wire        I_user,
    input  wire [11:0] I_row,
    output wire [5:0]  O_wt,
    output wire [5:0]  O_wm,
    output wire [5:0]  O_wb,
    output wire        O_valid,
    output wire        O_last,
    output wire        O_user,
    output wire [11:0] O_row
);
    // ---------------- S2：写行缓存 + 发预读地址 ----------------
    reg        s2_valid, s2_last, s2_user;
    reg [3:0]  s2_bin;
    reg [8:0]  s2_beat;
    reg [11:0] s2_row;
    reg        s2_par;

    reg [8:0]  beat_q;
    wire [8:0] beat_c = (I_valid && I_user) ? 9'd0 : beat_q;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            beat_q <= 9'd0;
        else if (I_valid)
            beat_q <= I_last ? 9'd0 : beat_c + 9'd1;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s2_valid <= 1'b0; s2_last <= 1'b0; s2_user <= 1'b0;
            s2_bin <= 4'd0; s2_beat <= 9'd0; s2_row <= 12'd0; s2_par <= 1'b0;
        end else begin
            s2_valid <= I_valid;  s2_last <= I_last;  s2_user <= I_user;
            s2_bin   <= I_bin;    s2_beat <= beat_c;  s2_row  <= I_row;
            if (I_valid && I_last) s2_par <= ~s2_par;
        end
    end

    wire [8:0] s2_waddr = s2_beat;
    // 预读下一拍。行末必须回绕到 0：否则请求的是行外地址（beat+1 越过本行末尾），
    // 读到残留数据，而且本行第 0 拍的字**永远不会**被读进移位链 ——
    // 结果是每行头两个像素的窗口拿错字，左边缘凭空多出假边缘。
    // 回绕后，行末读到的 word(0) 会沿着链成为下一行 beat0 的 w0 / beat1 的 prev，正好正确。
    wire [8:0] s2_raddr = s2_last ? 9'd0 : (s2_beat + 9'd1);

    wire [3:0] ram_even_dout, ram_odd_dout;

    sobel_line_ram u_ram_even (
        .I_clk(I_clk), .I_we(s2_valid & ~s2_par),
        .I_waddr(s2_waddr), .I_wdata(s2_bin),
        .I_raddr(s2_raddr), .O_rdata(ram_even_dout)
    );
    sobel_line_ram u_ram_odd (
        .I_clk(I_clk), .I_we(s2_valid &  s2_par),
        .I_waddr(s2_waddr), .I_wdata(s2_bin),
        .I_raddr(s2_raddr), .O_rdata(ram_odd_dout)
    );

    wire [3:0] ram_row_n1 = s2_par ? ram_even_dout : ram_odd_dout;
    wire [3:0] ram_row_n2 = s2_par ? ram_odd_dout  : ram_even_dout;

    // ---------------- S3：每行 3 拍滑窗 ----------------
    reg [3:0] s3_top_n, s3_top_0, s3_top_p;
    reg [3:0] s3_mid_n, s3_mid_0, s3_mid_p;
    reg [3:0] s3_bot_n, s3_bot_0, s3_bot_p;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s3_top_n <= 4'd0; s3_top_0 <= 4'd0; s3_top_p <= 4'd0;
            s3_mid_n <= 4'd0; s3_mid_0 <= 4'd0; s3_mid_p <= 4'd0;
            s3_bot_n <= 4'd0; s3_bot_0 <= 4'd0; s3_bot_p <= 4'd0;
        end else begin
            s3_top_p <= s3_top_0;  s3_top_0 <= s3_top_n;  s3_top_n <= ram_row_n2;
            s3_mid_p <= s3_mid_0;  s3_mid_0 <= s3_mid_n;  s3_mid_n <= ram_row_n1;
            s3_bot_p <= s3_bot_0;  s3_bot_0 <= s3_bot_n;  s3_bot_n <= s2_bin;
        end
    end

    reg        s3_valid, s3_last, s3_user;
    reg [8:0]  s3_beat;
    reg [11:0] s3_row;
    reg        s4_valid, s4_last, s4_user;
    reg [8:0]  s4_beat;
    reg [11:0] s4_row;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s3_valid <= 1'b0; s3_last <= 1'b0; s3_user <= 1'b0;
            s3_beat  <= 9'd0; s3_row  <= 12'd0;
            s4_valid <= 1'b0; s4_last <= 1'b0; s4_user <= 1'b0;
            s4_beat  <= 9'd0; s4_row  <= 12'd0;
        end else begin
            s3_valid <= s2_valid; s3_last <= s2_last; s3_user <= s2_user;
            s3_beat  <= s2_beat;  s3_row  <= s2_row;
            s4_valid <= s3_valid; s4_last <= s3_last; s4_user <= s3_user;
            s4_beat  <= s3_beat;  s4_row  <= s3_row;
        end
    end

    // ---------------- 边界替换（组合，不写回链）----------------
    wire last_beat  = s4_last;
    wire first_beat = (s4_beat == 9'd0);

    wire [3:0] top_p = first_beat ? {s3_top_0[0], 3'b000} : s3_top_p;
    wire [3:0] mid_p = first_beat ? {s3_mid_0[0], 3'b000} : s3_mid_p;
    wire [3:0] bot_p = first_beat ? {s3_bot_0[0], 3'b000} : s3_bot_p;

    wire [3:0] top_n = last_beat ? {3'b000, s3_top_0[3]} : s3_top_n;
    wire [3:0] mid_n = last_beat ? {3'b000, s3_mid_0[3]} : s3_mid_n;
    wire [3:0] bot_n = last_beat ? {3'b000, s3_bot_0[3]} : s3_bot_n;

    wire [11:0] w_top = {top_n, s3_top_0, top_p};
    wire [11:0] w_mid = {mid_n, s3_mid_0, mid_p};
    wire [11:0] w_bot = {bot_n, s3_bot_0, bot_p};

    assign O_wt = w_top[8:3];
    assign O_wm = w_mid[8:3];
    assign O_wb = w_bot[8:3];

    assign O_valid = s4_valid;
    assign O_last  = s4_last;
    assign O_user  = s4_user;
    assign O_row   = (s4_row == 12'd0) ? 12'd0 : s4_row - 12'd1;

endmodule

// ----------------------------------------------------------------------------
//  [5/5] 主模块
// ----------------------------------------------------------------------------
module sobel_edge_128 #(
    parameter integer IMG_WIDTH  = 1280,   // 1080p60；行缓存 512 深 >= 1920/4=480
    parameter integer IMG_HEIGHT = 720,
    // 几何筛选门限（isp_top 用默认值；TB 用小规模图案时会覆盖成小值）
    parameter integer MIN_W      = 100,    // 边界框最小宽度（像素）
    parameter integer MIN_H      = 80,     // 边界框最小高度（像素）
    // 竖直投影的有效行范围（默认 = 画面高度的中间 11/12，上下各砍 1/24）
    parameter integer ROW_LO     = IMG_HEIGHT / 24,
    parameter integer ROW_HI     = IMG_HEIGHT - (IMG_HEIGHT / 24),
    // 0 = 四个形态学步骤全部直通（延迟不变，便于上板 A/B 对比）
    parameter         MORPH3X3_EN = 1'b1
) (
    input  wire         I_clk,
    input  wire         I_rst_n,

    input  wire [127:0] I_tdata,
    input  wire         I_tvalid,
    input  wire         I_tlast,
    input  wire         I_tuser,

    input  wire [7:0]   I_y_thresh,     // 运行期：二值化阈值 0..255
    input  wire [3:0]   I_g_thresh,     // 运行期：边缘阈值 |gx|+|gy|，有效 1..8
    // 调试观测点选择（顶层 SW2/SW3/SW4 给出）：
    //   2'b00 = 正常输出（开->Sobel->闭）  2'b01 = 二值化（无行缓存）
    //   2'b10 = Sobel 边沿（3 级行缓存）   2'b11 = Y 灰度（无行缓存）
    input  wire [1:0]   I_dbg_sel,
    input  wire         I_bnd_en,       // 1 = 叠加红色矩形边界（SW1）
    input  wire [7:0]   I_bnd_ratio,    // KEY4 长按：填充阈值 0..214（像素）
    input  wire [1:0]   I_mid_ratio,    // 投影阈值 = vmax 的比例（KEY4 长按）

    output reg  [127:0] O_tdata,
    output reg          O_tvalid,
    output reg          O_tlast,
    output reg          O_tuser,
    // ↓ 峰值区域（ROI）对外接口，供后续程序使用（都是**上一帧**算出的值）
    output wire [11:0]  O_roi_x1,       // 左边界列号
    output wire [11:0]  O_roi_x2,       // 右边界列号
    output wire [11:0]  O_roi_y1,       // 峰内第一行
    output wire [11:0]  O_roi_y2,       // 峰内最后一行
    output wire         O_roi_valid,    // 本帧是否存在峰值区域
    // ↓ 区域选择输入（选第几组列段/行段），供外部遍历所有高峰
    input  wire [3:0]   I_roi_idx,
    input  wire [3:0]   I_roi_yidx,
    // ↓ 多区域（剖面分段）接口
    output wire [3:0]   O_roi_cnt,      // 本帧识别到几个**列段**
    output wire [3:0]   O_rcnt,         // 0/1：本帧有没有找到行边界
    // ↓ 上下边界的"长度"（= 下边界 - 上边界 + 1）
    output wire [11:0]  O_roi_h,
    // ↓ 宽度坐标列表接口（KEY4 使能计数器后有效）
    input  wire         I_k4_en,        // KEY4：计数器使能
    output wire [4:0]   O_line_cnt,     // [xl, xr] 内按 W 分布的竖线条数（<=16）
    output wire [191:0] O_line_x,       // 坐标列表：第 k 条 = xl + k*W（12 位/条）
    output wire [11:0]  O_fill_h,       // KEY4 计数器值（被填充行数的历史最大值）
    // ↓ 给"帧差法"用：YCbCr 的 Y 分量（4 个像素/拍，8 位 × 4）
    //   ★ 连同 valid/last/user 一起引出（都是 s1 级，与 O_y_out 严格同拍）
    //     不能用 O_tvalid/O_tuser：那是整个 ISP 输出级的，差了整条流水线的延迟
    output wire [31:0]  O_y_out,
    output wire         O_y_valid,
    output wire         O_y_last,
    output wire         O_y_user,
    // ★ 麻将识别（tile_recog_onchip）用：4 个像素各自的"色系码"（每像素 2 位）
    //   0=红 1=绿 2=蓝 3=无彩；与 O_y_out 严格同拍（同一级寄存器）
    //   ★ 为什么不直接引出 Cb/Cr：那会让 rgb_to_ycbcr_4 里的 24 个常量乘法器
    //     从"死逻辑"变成活逻辑 —— 实测 DSP 从 33 涨到 49，超过本器件上限 40，
    //     布局器直接失败（PHY-7027 + PHY-9002）。这里只用比较器，零乘法。
    output wire [7:0]   O_col4
);
    // ========================================================================
    // S0/S1/S2：行计数 -> RGB转Y -> 二值化
    // ========================================================================
    reg  [11:0] row_q;
    wire s0_start = I_tvalid && I_tuser;
    wire s0_end   = I_tvalid && I_tlast;
    wire [11:0] row_c = s0_start ? 12'd0 : row_q;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            row_q <= 12'd0;
        else if (s0_end)
            row_q <= row_c + 12'd1;
    end

    wire [31:0] s1_y, s1_cb, s1_cr;
    rgb_to_ycbcr_4 u_ycbcr (
        .I_clk(I_clk), .I_tdata(I_tdata),
        .O_y(s1_y), .O_cb(s1_cb), .O_cr(s1_cr)
    );

    wire [3:0] s2_bin;
    binarize_4 u_bin (
        .I_clk(I_clk), .I_y(s1_y), .I_thresh(I_y_thresh), .O_bin(s2_bin)
    );

    // ========================================================================
    //  ★ 色系码（麻将识别用，零乘法：只用比较器）
    //    判据与离线模板工具链 sim/make_tile_templates.py 的 color_sig 完全一致：
    //      max-min < COL_SAT -> 无彩；否则按最大分量判 红/绿/蓝
    //    与 rgb_to_ycbcr_4 同一级寄存（都是 I_tdata 打一拍），所以与 s1_y 同拍。
    // ========================================================================
    localparam [7:0] COL_SAT = 8'd40;
    function [1:0] col_code;
        input [23:0] rgb;
        reg [7:0] r, g, b, mx, mn;
        begin
            r = rgb[23:16]; g = rgb[15:8]; b = rgb[7:0];
            mx = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
            mn = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
            if      ((mx - mn) < COL_SAT) col_code = 2'd3;   // 无彩（黑/白/灰）
            else if (mx == r)             col_code = 2'd0;   // 红
            else if (mx == g)             col_code = 2'd1;   // 绿
            else                          col_code = 2'd2;   // 蓝
        end
    endfunction

    reg [7:0] s1_col4;
    always @(posedge I_clk) begin
        s1_col4 <= { col_code(I_tdata[127:96]), col_code(I_tdata[95:64]),
                     col_code(I_tdata[63:32]),  col_code(I_tdata[31:0]) };
    end

    reg        s1_valid, s1_last, s1_user;
    reg [11:0] s1_row;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s1_valid <= 1'b0; s1_last <= 1'b0; s1_user <= 1'b0; s1_row <= 12'd0;
        end else begin
            s1_valid <= I_tvalid; s1_last <= I_tlast; s1_user <= I_tuser;
            s1_row   <= row_c;
        end
    end

    reg        s2_valid, s2_last, s2_user;
    reg [11:0] s2_row;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s2_valid <= 1'b0; s2_last <= 1'b0; s2_user <= 1'b0; s2_row <= 12'd0;
        end else begin
            // 必须跟着 s1 走：s2_bin 是两级寄存的结果，控制少打一拍会让窗口横向偏一整拍
            s2_valid <= s1_valid; s2_last <= s1_last; s2_user <= s1_user;
            s2_row   <= s1_row;
        end
    end

    // s2 级（二值化输出）的拍号：bin_rowwin 内部有自己的 beat 计数器，
    // 但投影模块在 sobel_edge_128 这一层，需要自己数一份，且必须与 s2_bin 同拍。
    reg [8:0] s2b;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)      s2b <= 9'd0;
        else if (s2_valid) s2b <= (s2_last || s2_user) ? 9'd0 : s2b + 9'd1;
    end

    // ========================================================================
    // 五级窗口
    // ========================================================================
    // ---- 开运算第 1 步：腐蚀 ----
    wire [5:0] w1_t, w1_m, w1_b;
    wire       v1, l1, u1;  wire [11:0] r1;
    bin_rowwin u_win1 (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_bin(s2_bin), .I_valid(s2_valid), .I_last(s2_last), .I_user(s2_user),
        .I_row(s2_row),
        .O_wt(w1_t), .O_wm(w1_m), .O_wb(w1_b),
        .O_valid(v1), .O_last(l1), .O_user(u1), .O_row(r1)
    );

    wire [3:0] ero_bin;
    bin_morph3x3 u_morph1 (
        .I_op(MORPH3X3_EN ? 2'b01 : 2'b00),      // 01 = 腐蚀
        .I_wt(w1_t), .I_wm(w1_m), .I_wb(w1_b), .O_bin(ero_bin)
    );

    // ---- Sobel 的 3x3 窗口（复用 win2 的三行）----
    // 链路：二值化 -> win1 -> **腐蚀** -> win2 -> **Sobel** -> win3 -> **膨胀** -> 输出
    // 共 3 级行缓存，延迟 3 行（原设计 5 行）。
    wire [5:0] w2_t, w2_m, w2_b;
    wire       v2, l2, u2;  wire [11:0] r2;
    bin_rowwin u_win2 (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_bin(ero_bin), .I_valid(v1), .I_last(l1), .I_user(u1),
        .I_row(r1),
        .O_wt(w2_t), .O_wm(w2_m), .O_wb(w2_b),
        .O_valid(v2), .O_last(l2), .O_user(u2), .O_row(r2)
    );

    // ---- Sobel（组合；作用在腐蚀后的二值图上）----
    wire [3:0] edge_bits;
    genvar gm;
    generate
        for (gm = 0; gm < 4; gm = gm + 1) begin : SOBEL_M
            wire [4:0] tl = {4'b0, w2_t[gm]};
            wire [4:0] tm = {4'b0, w2_t[gm+1]};
            wire [4:0] tr = {4'b0, w2_t[gm+2]};
            wire [4:0] ml = {4'b0, w2_m[gm]};
            wire [4:0] mr = {4'b0, w2_m[gm+2]};
            wire [4:0] bl = {4'b0, w2_b[gm]};
            wire [4:0] bm = {4'b0, w2_b[gm+1]};
            wire [4:0] br = {4'b0, w2_b[gm+2]};

            wire signed [4:0] gx = $signed(tr) + $signed({mr, 1'b0}) + $signed(br)
                                 - $signed(tl) - $signed({ml, 1'b0}) - $signed(bl);
            wire signed [4:0] gy = $signed(bl) + $signed({bm, 1'b0}) + $signed(br)
                                 - $signed(tl) - $signed({tm, 1'b0}) - $signed(tr);

            wire [4:0] ax = gx[4] ? (~gx + 5'd1) : gx[4:0];
            wire [4:0] ay = gy[4] ? (~gy + 5'd1) : gy[4:0];
            wire [4:0] grad = ax + ay;

            assign edge_bits[gm] = (grad >= {1'b0, I_g_thresh});
        end
    endgenerate

    // ---- 收尾膨胀（作用在边缘图上，win3 的三行窗口）----
    wire [5:0] w3_t, w3_m, w3_b;
    wire       v3, l3, u3;  wire [11:0] r3;
    bin_rowwin u_win3 (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_bin(edge_bits), .I_valid(v2), .I_last(l2), .I_user(u2),
        .I_row(r2),
        .O_wt(w3_t), .O_wm(w3_m), .O_wb(w3_b),
        .O_valid(v3), .O_last(l3), .O_user(u3), .O_row(r3)
    );

    wire [3:0] open_bin;
    bin_morph3x3 u_morph3 (
        .I_op(MORPH3X3_EN ? 2'b10 : 2'b00),      // 10 = 膨胀
        .I_wt(w3_t), .I_wm(w3_m), .I_wb(w3_b), .O_bin(open_bin)
    );

    // ========================================================================
    //  ★ 新增【闭运算】：膨胀 -> 腐蚀（弥补边缘断口/填小孔）
    //    在开运算结果 open_bin 上再做一次 3x3，需 2 个新的行缓存窗口
    //    ERAM：+2 个 bin_rowwin x 2 块 = +4 块
    // ========================================================================
    wire [5:0] w4_t, w4_m, w4_b;
    wire       v4, l4, u4;  wire [11:0] r4;
    bin_rowwin u_win4 (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_bin(open_bin), .I_valid(v3), .I_last(l3), .I_user(u3),
        .I_row(r3),
        .O_wt(w4_t), .O_wm(w4_m), .O_wb(w4_b),
        .O_valid(v4), .O_last(l4), .O_user(u4), .O_row(r4)
    );

    wire [3:0] dil_bin;
    bin_morph3x3 u_morph4 (
        .I_op(MORPH3X3_EN ? 2'b10 : 2'b00),      // 10 = 膨胀
        .I_wt(w4_t), .I_wm(w4_m), .I_wb(w4_b), .O_bin(dil_bin)
    );

    wire [5:0] w5_t, w5_m, w5_b;
    wire       v5, l5, u5;  wire [11:0] r5;
    bin_rowwin u_win5 (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_bin(dil_bin), .I_valid(v4), .I_last(l4), .I_user(u4),
        .I_row(r4),
        .O_wt(w5_t), .O_wm(w5_m), .O_wb(w5_b),
        .O_valid(v5), .O_last(l5), .O_user(u5), .O_row(r5)
    );

    wire [3:0] fin_bin;
    bin_morph3x3 u_morph5 (
        .I_op(MORPH3X3_EN ? 2'b01 : 2'b00),      // 01 = 腐蚀
        .I_wt(w5_t), .I_wm(w5_m), .I_wb(w5_b), .O_bin(fin_bin)
    );

    // ========================================================================
    // 消隐带：**已停用**
    // ------------------------------------------------------------------------
    // 原本把帧首 3 行 / 帧尾 3 行涂黑（那里 3x3 窗口还没成形）。
    // 实测：SW3（同一份 edge_bits、但不消隐）稳定，而 SW1（带消隐）闪 ——
    //       两者代码上**唯一**的差别就是这个掩码，所以闪烁来自这条消隐带。
    // 机理：消隐带的位置由行计数 r5 决定，只要 r5 与实际行有偏差，黑带就会
    //       跑到画面中间；偏差逐帧变化 = 黑带在屏幕上移动 = 闪烁。
    //       而 r5 在整个设计里**只**用于这个消隐，别处都不用。
    // 停用后：帧首/帧尾各 3 行显示未成形的窗口（噪声），但它们在图像里的
    //       位置是固定的，不再随行计数漂移。拍数完全不变。
    // 恢复：把下行改成 (r5 < 12'd3) || (r5 >= (IMG_HEIGHT - 3))
    // ========================================================================
    wire blank = 1'b0;

    // ========================================================================
    // 水平边界红框：**已按需求临时停用**（用于排查闪烁到底是不是它引起的）。
    //   bin_row_bound 模块定义仍保留在本文件末尾（未例化 = 综合时会被裁掉，零代价）。
    //   恢复方法：把下面这段注释打开，并把 norm_img 里的 32'h00FF_FFFF 换成
    //             (red_row ? 32'h00FF_0000 : 32'h00FF_FFFF)
    // ========================================================================
    // wire red_row;
    // bin_row_bound #(
    //     .DIFF_TH (12'd16)
    // ) u_rowbnd (
    //     .I_clk  (I_clk),
    //     .I_rst_n(I_rst_n),
    //     .I_bin  (fin_bin & {4{~blank}}),        // 消隐行不参与累加
    //     .I_valid(v5),
    //     .I_last (l5),
    //     .I_ena  (~blank),
    //     .O_red_row(red_row)
    // );

    // ========================================================================
    // 竖直方向：**不求上下边界**，改成"高峰行的内部填满"
    //   每行的前景像素数（s2_bin）在行末与**上一帧的峰值 × 比例**比较：
    //        row_sum >= thr  ->  这一行"在峰内"
    //   门限取上一帧的峰值（架构本身就有 1 帧延迟），于是：
    //       * 不需要 ERAM 存剖面
    //       * 不需要帧末扫描、不需要状态机
    //       * 在线一拍一拍判定即可
    //   同时在线记录 ROI 行范围：第一个在峰内的行 = y1，最后一个 = y2。
    // ========================================================================
    localparam integer ROW_THR_SHIFT = 1;      // 门限 = 上一帧峰值 >> 1（即 1/2，可改）
    // 区域选择由外部输入（选第几组列段/行段），不接就默认 0
    wire [3:0] roi_idx  = I_roi_idx;
    wire [3:0] roi_yidx = I_roi_yidx;

    reg  [11:0] r_row      = 12'd0;    // s2 阶段行计数
    reg  [11:0] r_rowsum   = 12'd0;    // 本行前景计数（累加中）
    reg         r_inpeak   = 1'b0;     // 本行是否在峰内
    reg  [11:0] r_y1       = 12'hFFF;  // 峰内第一行（工作寄存器）
    reg  [11:0] r_y2       = 12'd0;    // 峰内最后一行（工作寄存器）
    reg  [11:0] roi_y1_l   = 12'd0;    // 帧末锁存，供对外接口稳定读取
    reg  [11:0] roi_y2_l   = 12'd0;

    // "在峰内"标志延迟 3 个行末：s2 到输出隔着 3 级行缓存（win1/win2/win3）
    reg r_ip0 = 1'b0, r_ip1 = 1'b0, r_ip2 = 1'b0, r_ip3 = 1'b0;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin r_ip0 <= 1'b0; r_ip1 <= 1'b0; r_ip2 <= 1'b0; r_ip3 <= 1'b0; end
        else if (v5 && l5) begin
            r_ip0 <= r_inpeak; r_ip1 <= r_ip0; r_ip2 <= r_ip1; r_ip3 <= r_ip2;
        end
    end
    wire r_fill_d = r_ip3;      // 与像素对齐三行后的填充标志

    // 对外接口（都是上一帧的结果）
    // 对外接口直接输出当帧扫描结果（死区锁定已去掉）
    assign O_roi_x1    = bxl;
    assign O_roi_x2    = bxr;
    // 上下边界：指向新的单识别结果（第一行高峰 / 最后一行高峰）
    //   byt / byb 是 bin_proj_bound 的输出（O_yt/O_yb）——注意不能用内部的 sy1_f/sy2_f
    assign O_roi_y1    = (byb >= byt) ? byt : 12'd0;
    assign O_roi_y2    = (byb >= byt) ? byb : 12'd0;
    assign O_roi_valid = (roi_cnt != 4'd0);   // 改用列段数（行峰值已停用）
    // 注意：O_roi_cnt / O_rcnt 在下面统一赋值，这里**不能**再赋一次
    //（同一个输出两个驱动，TD 会报 constantly driven from multiple places）

    // ------------------------------------------------------------------------
    // 拍号（输出级），用于把像素坐标算成列号 x = 拍号*4 + m
    // ------------------------------------------------------------------------
    reg [8:0] obeat;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)  obeat <= 9'd0;
        else if (v5)   obeat <= l5 ? 9'd0 : obeat + 9'd1;
    end

    // ------------------------------------------------------------------------
    // 投影边界：列投影（垂直方向累加）+ 行投影（水平方向累加）
    //   帧末消隐期算完，锁存成一帧，下一帧用来画红色矩形框。
    // ------------------------------------------------------------------------
    // 投影对象 = **填充的二值图**（s2_bin，腐蚀之前）。
    //   边缘图对"实心/平滑目标"是空心轮廓 -> 行剖面只在上下边缘高 -> 阈值一切只剩薄条。
    //   填充图是实心的 -> 剖面宽单峰 -> 框住整个目标。
    wire [11:0] bxl, bxr, byt, byb, prow_raw;
    wire [11:0] proj_rowpeak;      // 行剖面峰值（bin_proj_bound 输出，供 O_roi_valid 用）
    bin_proj_bound #(
        .BEATS     (IMG_WIDTH / 4),
        .IMG_HEIGHT(IMG_HEIGHT),
        .RW        ((IMG_HEIGHT + 2) / 3),
        .ROW_LO    (ROW_LO),
        .ROW_HI    (ROW_HI)
    ) u_proj (
        .I_clk  (I_clk),
        .I_rst_n(I_rst_n),
        .I_bin  (s2_bin),
        .I_valid(s2_valid),
        .I_last (s2_last),
        .I_user (s2_user),
        .I_mid_ratio(I_mid_ratio),
        .I_beat (s2b),
        .I_row  (r5),
        .I_fill_thr(I_bnd_ratio),
        .I_k4_en (I_k4_en),
        .O_xl   (bxl), .O_xr(bxr), .O_yt(byt), .O_yb(byb), .O_rowpeak(proj_rowpeak),
        .O_row  (prow_raw),
        // 多区域识别接口：roi_idx 选第几个区域（当前固定 0 = 最左边那个），
        //     下游要遍历所有区域就把它从 0 扫到 roi_cnt-1。
        .I_roi_idx(roi_idx), .O_roi_cnt(roi_cnt),
        .I_roi_yidx(roi_yidx), .O_rcnt(roi_rcnt),
        .O_x1_all(roi_x1_all), .O_x2_all(roi_x2_all),
        .O_rowhas (proj_rowhas),
        .O_fill_h (proj_fill_h)
    );

    // 区域选择：先用固定 0（行为与单区域时完全一致）；如需遍历，把它接到输入端口即可。
    wire [3:0] roi_cnt;
    wire [3:0] roi_rcnt;
    wire [191:0] roi_x1_all;      // 全部段的左边界（段 k 在 [12k +: 12]）
    wire [191:0] roi_x2_all;      // 全部段的右边界
    wire         proj_rowhas;     // 本显示行是否有值（来自 bin_proj_bound）
    wire [11:0]  proj_fill_h;     // 被填充的行数 = 高度 H
    assign O_fill_h = proj_fill_h;
    // 行基准：直接用投影自己的行计数（不做延迟链，避免帧边界错拍导致漂移）
    wire [11:0] prow = prow_raw;
    // ================= 左右边界（单识别，只有一对）=================
    wire [11:0] xl_c = bxl;
    wire [11:0] xr_c = bxr;
    wire [11:0] xb   = {obeat, 2'b00};       // 本拍第一个像素的列号

    // ================= 绿色矩形框（仅显示，纯叠加）=================
    //   高 = 100 像素；宽 = 14 张牌 = 14 × (100 × 0.708) ≈ 991 像素
    //   （一张牌的 宽/高 = 425/600 ≈ 0.708，与你实测 1700x600 放 4 张牌一致）
    //   水平居中、垂直居中；只画 1 像素边框；绿色；优先级最高
    localparam [11:0] GBX_H  = 12'd100;                                   // 框高
    localparam [11:0] GBX_CH = (GBX_H * 708) / 1000;                      // 一张牌宽 ≈ 70
    localparam [11:0] GBX_W  = GBX_CH * 12'd14;                           // 14 张牌 ≈ 991
    localparam [11:0] GBX_X0 = (12'd1280 - GBX_W) >> 1;                   // 水平居中
    localparam [11:0] GBX_Y0 = (12'd720 - GBX_H) >> 1;                   // 垂直居中
    localparam [11:0] GBX_X1 = GBX_X0 + GBX_W - 12'd1;
    localparam [11:0] GBX_Y1 = GBX_Y0 + GBX_H - 12'd1;
    localparam [23:0] GBX_COL = 24'h00FF00;                               // 绿色

    wire gbx_hx = (xb   >= GBX_X0) && (xb   <= GBX_X1);
    wire gbx_hx3= (xb+3 >= GBX_X0) && (xb+3 <= GBX_X1);
    wire gbx_vy = (prow >= GBX_Y0) && (prow <= GBX_Y1);
    // 上/下边：在框内的列上；左/右边：在框内的行上
    wire gbx_top = (prow == GBX_Y0) || (prow == GBX_Y1);
    wire gbx3 = (gbx_top && gbx_hx3) || (gbx_vy && ((xb+3 == GBX_X0) || (xb+3 == GBX_X1)));
    wire gbx2 = (gbx_top && ((xb+2 >= GBX_X0) && (xb+2 <= GBX_X1))) ||
                (gbx_vy && ((xb+2 == GBX_X0) || (xb+2 == GBX_X1)));
    wire gbx1 = (gbx_top && ((xb+1 >= GBX_X0) && (xb+1 <= GBX_X1))) ||
                (gbx_vy && ((xb+1 == GBX_X0) || (xb+1 == GBX_X1)));
    wire gbx0 = (gbx_top && gbx_hx) ||
                (gbx_vy && ((xb   == GBX_X0) || (xb   == GBX_X1)));
    // 不区分上下边界：改为"有值的行 -> 在 [xl, xr] 内整行填红"
    wire        bH   = 1'b0;                 // 不再画上下横线
    // 该行有值 且 本像素列落在 [xl, xr] 内 -> 填充
    // ================= 原来的填充：有值的行 -> 在 [xl, xr] 内整行填红 =================
    wire        f3 = proj_rowhas && (xb + 12'd3 >= xl_c) && (xb + 12'd3 <= xr_c);
    wire        f2 = proj_rowhas && (xb + 12'd2 >= xl_c) && (xb + 12'd2 <= xr_c);
    wire        f1 = proj_rowhas && (xb + 12'd1 >= xl_c) && (xb + 12'd1 <= xr_c);
    wire        f0 = proj_rowhas && (xb       >= xl_c) && (xb       <= xr_c);
    // ================= 上下边界（第一行高峰 / 最后一行高峰）=================
    //   byt / byb 来自行剖面扫描：第一次"进入"窗口的行 = 上边界，最后一次"离开" = 下边界
    wire [11:0] yt_c = byt;                                  // 上边界
    wire [11:0] yb_c = byb;                                  // 下边界
    wire [11:0] roi_h = (yb_c >= yt_c) ? (yb_c - yt_c + 12'd1) : 12'd0;   // 长度
    assign O_roi_h = roi_h;
    // 帧差法用的 Y 分量（4 像素/拍）及其控制信号：都取 s1 级，严格同拍
    assign O_y_out   = s1_y;
    assign O_y_valid = s1_valid;
    assign O_y_last  = s1_last;
    assign O_y_user  = s1_user;
    // ★ 色系码（麻将识别用）：与 s1_y 同拍，见下面的 col_code/寄存器
    assign O_col4    = s1_col4;
    // ★ 消隐已全部取消：竖线恢复贯穿整高
    wire        v_in = 1'b1;
    wire        build_mark = 1'b0;           // 版本标记已去掉
    wire [31:0] bnd_col    = 32'h00FF_0000;  // 边界线一律红色
    wire        bnd_go     = I_bnd_en;       // SW1 拨上就画（已无尺寸/比例筛选）
    // 横向范围：只在 xl..xr 之间画
    wire        h3 = (xb + 12'd3 >= xl_c) && (xb + 12'd3 <= xr_c);
    wire        h2 = (xb + 12'd2 >= xl_c) && (xb + 12'd2 <= xr_c);
    wire        h1 = (xb + 12'd1 >= xl_c) && (xb + 12'd1 <= xr_c);
    wire        h0 = (xb       >= xl_c) && (xb       <= xr_c);
    // 竖边：在 xl / xr 两列上，且行落在行带内
    // 竖边：本拍 4 个像素列，落在**任一段**左右边界上的就点亮（多段同时画）
    // 单识别：本拍 4 个像素列里，落在 xl_c / xr_c 上的就点亮
    wire        vb3 = v_in && ((xb + 12'd3 == xl_c) || (xb + 12'd3 == xr_c));
    wire        vb2 = v_in && ((xb + 12'd2 == xl_c) || (xb + 12'd2 == xr_c));
    wire        vb1 = v_in && ((xb + 12'd1 == xl_c) || (xb + 12'd1 == xr_c));
    wire        vb0 = v_in && ((xb        == xl_c) || (xb        == xr_c));
    wire [3:0] bmark = { build_mark && (xb + 12'd3 < 12'd32),
                         build_mark && (xb + 12'd2 < 12'd32),
                         build_mark && (xb + 12'd1 < 12'd32),
                         build_mark && (xb       < 12'd32) };
    wire        b3 = bnd_go && (f3 || vb3);
    wire        b2 = bnd_go && (f2 || vb2);
    wire        b1 = bnd_go && (f1 || vb1);
    wire        b0 = bnd_go && (f0 || vb0);

    // 正常输出：边缘图 + 边界框（颜色由三色诊断决定）
    // 边界线一律红色，其余维持原来的边缘图（白/黑）
    // 优先级：绿色框（最高，仅显示）> 填充/边界（红）> 边缘图（白/黑）
    wire gshow = I_bnd_en;      // SW1 打开时才显示这个框
    wire [127:0] norm_img = {
        (gshow && gbx3) ? GBX_COL : (f3 ? bnd_col : (b3 ? bnd_col : ((fin_bin[3] & ~blank) ? 32'h00FF_FFFF : 32'h0000_0000))),
        (gshow && gbx2) ? GBX_COL : (f2 ? bnd_col : (b2 ? bnd_col : ((fin_bin[2] & ~blank) ? 32'h00FF_FFFF : 32'h0000_0000))),
        (gshow && gbx1) ? GBX_COL : (f1 ? bnd_col : (b1 ? bnd_col : ((fin_bin[1] & ~blank) ? 32'h00FF_FFFF : 32'h0000_0000))),
        (gshow && gbx0) ? GBX_COL : (f0 ? bnd_col : (b0 ? bnd_col : ((fin_bin[0] & ~blank) ? 32'h00FF_FFFF : 32'h0000_0000))) };

    // ---- 调试观测点 ----------------------------------------------------------
    // bin_morph3x3 / binarize_4 的输出位序：bit m 对应第 m 个像素（lane m = bits[32m+:32]）
    // 观测点 3：Y 灰度。rgb_to_ycbcr_4 只打 1 拍，**完全没有行缓存**，
    //           所以这一路如果就出现横/竖线，问题一定在上游（MIPI/解拜耳/AWB/gamma）。
    wire [127:0] dbg_y_img = { 8'h00, s1_y[31:24], s1_y[31:24], s1_y[31:24],
                               8'h00, s1_y[23:16], s1_y[23:16], s1_y[23:16],
                               8'h00, s1_y[15:8],  s1_y[15:8],  s1_y[15:8],
                               8'h00, s1_y[7:0],   s1_y[7:0],   s1_y[7:0] };
    // 观测点 1：二值化输出（binarize_4 打 2 拍，**没有行缓存**）
    wire [127:0] dbg_bin_img = { s2_bin[3] ? 32'h00FF_FFFF : 32'h0000_0000,
                                 s2_bin[2] ? 32'h00FF_FFFF : 32'h0000_0000,
                                 s2_bin[1] ? 32'h00FF_FFFF : 32'h0000_0000,
                                 s2_bin[0] ? 32'h00FF_FFFF : 32'h0000_0000 };
    // 观测点 2：Sobel 边沿输出（win3 的窗口、膨胀**之前**；正常输出在其后再做一次膨胀）
    wire [127:0] dbg_edge_img = { edge_bits[3] ? 32'h00FF_FFFF : 32'h0000_0000,
                                  edge_bits[2] ? 32'h00FF_FFFF : 32'h0000_0000,
                                  edge_bits[1] ? 32'h00FF_FFFF : 32'h0000_0000,
                                  edge_bits[0] ? 32'h00FF_FFFF : 32'h0000_0000 };

    // 每一路配自己的 valid/last/user，这样各路显示的行位置都是正确的
    wire [127:0] sel_tdata = (I_dbg_sel == 2'b11) ? dbg_y_img    :
                             (I_dbg_sel == 2'b10) ? dbg_edge_img :
                             (I_dbg_sel == 2'b01) ? dbg_bin_img  : norm_img;
    wire         sel_valid = (I_dbg_sel == 2'b11) ? s1_valid   :
                             (I_dbg_sel == 2'b10) ? v2         :
                             (I_dbg_sel == 2'b01) ? s2_valid   : v5;
    wire         sel_last  = (I_dbg_sel == 2'b11) ? s1_last    :
                             (I_dbg_sel == 2'b10) ? l2         :
                             (I_dbg_sel == 2'b01) ? s2_last    : l5;
    wire         sel_user  = (I_dbg_sel == 2'b11) ? s1_user    :
                             (I_dbg_sel == 2'b10) ? u2         :
                             (I_dbg_sel == 2'b01) ? s2_user    : u5;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tdata  <= 128'd0;
            O_tvalid <= 1'b0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
        end else begin
            O_tdata  <= sel_tdata;
            O_tvalid <= sel_valid;
            O_tlast  <= sel_last;
            O_tuser  <= sel_user;
        end
    end

endmodule

// ----------------------------------------------------------------------------
//  投影边界提取：对边缘二值图做行/列投影，找波峰波谷，输出一个矩形边界
// ----------------------------------------------------------------------------
//  列投影（垂直方向累加）：
//      ERAM 512x48，**一词 = 4 列 x 12 位**，地址 = 拍号 —— 每拍正好 4 个像素对
//      应 4 列，所以一拍就能完成这 4 列的"读-加-写"，不需要额外端口。
//  行投影（水平方向累加）：
//      每行边缘像素数累加，3 行打包成一个 36 位词写进 ERAM 512x36。
//  帧末消隐期（约 47 行 x 480 拍 ≈ 22500 空闲周期）扫两套剖面：
//      vmax / vmin -> mid = (vmax+vmin)/2
//      从峰值位置向两侧扫，第一个 v < mid 的位置就是边界
//        （波谷->波峰 为左边界，波峰->波谷 为右边界；行方向同理 -> 上/下边界）
//  边界值锁存后**下一帧**使用（只占 4 个寄存器）。区域不明显（剖面是平的）时
//  扫不到穿越点，直接用画面边界，框住整幅。
//
//  说明：ERAM 读延迟 1 拍，所以比较时用的是上一拍发出的地址（wcnt_d / rwcnt_d）。
//        列索引 = {wcnt_d, slot}（4 列/词）；行索引 = 3*rwcnt_d + slot（3 行/词，
//        用 {rwcnt_d,1'b0}+rwcnt_d+slot 算，避免乘法器）。
// ----------------------------------------------------------------------------
module bin_proj_bound #(
    parameter integer BEATS      = 480,     // 每行拍数 = 宽/4
    parameter integer IMG_HEIGHT = 720,
    parameter integer RW         = 360,     // 行投影词数 = ceil(IMG_HEIGHT/3)
    // 竖直投影的有效行范围 = 画面高度的中间 **11/12**（上下各砍 1/24）。
    // 帧首几十行是 ISP 流水线里的上一帧残留（二值化接近全白 -> 行投影出现巨大假峰，
    // 门限 midr 被抬到高于真实目标的每行值 -> 上下边界双双走兜底：
    // 上红线到屏幕顶、下红线到屏幕底）。帧尾几行则是流水线冲刷的残缺窗口。
    // 两头都排除掉最稳。缺省值由 IMG_HEIGHT 自动推导，小尺寸 TB 无需覆盖。
    //   1080 -> 45 .. 1034，共 990 行 = 1080 x 11/12
    parameter integer ROW_LO     = IMG_HEIGHT / 24,
    parameter integer ROW_HI     = IMG_HEIGHT - (IMG_HEIGHT / 24),
    // 识别门限的"噪声底"：mid = (列峰值 >> MID_SHIFT) + MID_BASE
    //   MID_SHIFT 越小 -> 门限越高（越不容易被噪点拉出边界）
    //   现在是 3 / 1（约 1/8），比最初的 5/1（1/32）大了 4 倍。
    //   底数取 1 而不是大常数：这样大画面（列计数上百）与小图案（列计数个位数）都能自适应。
    parameter integer MID_SHIFT  = 3,
    parameter integer MID_BASE   = 1,
    // 左右边界各自的"噪声窗口"宽度（列）：
    //   找到一个左边界后，往后 BND_WIN 列内不再做左边界判定；
    //   右边界同理。左右之间互不影响（异类边界不限）。
    parameter integer BND_WIN    = 30,
    // 边界"最小宽度确认"（只用于列方向）
    parameter integer MIN_RUN_W  = 4,      // 列方向：连续 >=4 列才承认左/右边界
    // 【填充阈值】一行至少要有这么多个白像素才被填红
    //   0 = 只要有 1 个白像素就填（最敏感）
    //   4 = 默认，能挡掉零星噪点行；噪点多就继续调大（8 / 16 / 32…）
    parameter integer ROW_THR    = 4
) (
    input  wire         I_clk,
    input  wire         I_rst_n,
    input  wire [3:0]   I_bin,        // 本拍 4 个像素（= s2_bin，二值化图，**不是**边缘图）
    input  wire         I_valid,
    input  wire         I_last,       // 行末
    input  wire         I_user,       // 帧第一个拍（真正的帧界）
    input  wire [1:0]   I_mid_ratio,  // 投影阈值 = vmax 的比例 0=1:1 1=2:3 2=1:2 3=1:3
    input  wire [8:0]   I_beat,       // 拍号 0..BEATS-1
    input  wire [11:0]  I_row,        // 行号 0..IMG_HEIGHT-1
    input  wire         I_k4_en,      // KEY4：填充行计数器使能
    input  wire [7:0]   I_fill_thr,   // 填充阈值档位（KEY2，0..160，下游 x8）
    // 边界值**不在复位里清零**（只给上电初值）：否则任何一次复位都会把刚算好的框丢掉，
    // 而扫描与用户复位是异步的。上电初值 0 相当于"框住整幅"。
    input  wire [3:0]   I_roi_idx,    // 选第几个**列段**（0..MAX_ROI-1）
    input  wire [3:0]   I_roi_yidx,   // 选第几个**行段**（0..MAX_ROI-1）
    output wire [3:0]   O_roi_cnt,    // 本帧识别到的**列段**个数
    // 全部段一次性输出：段 k 的左右边界在 [12k +: 12]（MAX_ROI=15 -> 16*12=192 位）
    output wire [191:0] O_x1_all,
    output wire [191:0] O_x2_all,
    // 本显示行是否有值（已是延迟对齐后的标记），供上层做"有值行填充"
    output wire         O_rowhas,
    output wire [11:0]  O_fill_h,      // 被填充的行数（每帧重数 = 高度 H）
    output wire [3:0]   O_rcnt,       // 本帧识别到的**行段**个数
    // 注意：O_xl/O_xr 现在输出的是**由 I_roi_idx 选中的那个区域**的左右边界，
    //       下游（锁定/筛选/填涂/对外接口）全部不用改。
    output wire [11:0]  O_xl,         // 选中区域的左边界列号
    output wire [11:0]  O_xr,         // 选中区域的右边界列号
    output wire [11:0]  O_yt,         // **选中行段**的上边界行号
    output wire [11:0]  O_yb,         // **选中行段**的下边界行号
    output wire [11:0]  O_row,         // 当前正在累加的行号（供输出级对齐用）
    output wire  [11:0]  O_rowpeak
);
    // ========================================================================
    //  ★ 已停用（用户要求：图像检测只留帧差 + Sobel）
    //    水平/垂直投影 + 边界 + ROI + 竖线计数全部不再使用，输出接常量 0，
    //    下游（边界绘制 / GBX 绿框 / OSD 的 H、D 两行）随之被综合自动剪掉。
    //    要恢复：从 git 历史取回本模块体即可（git log -- user_source/hdl_source/isp/sobel_edge_128.v）。
    // ========================================================================
    assign O_xl      = 0;
    assign O_xr      = 0;
    assign O_yt      = 0;
    assign O_yb      = 0;
    assign O_rowpeak = 0;
    assign O_row     = 0;
    assign O_roi_cnt = 0;
    assign O_rcnt    = 0;
    assign O_x1_all  = 0;
    assign O_x2_all  = 0;
    assign O_rowhas  = 0;
    assign O_fill_h  = 0;
endmodule




//    I_op = 2'b00 直通（取中心像素）  2'b01 腐蚀(9 位全 AND)  2'b10 膨胀(9 位全 OR)
//  窗口约定：对第 m 个像素，左 = win[m]，中 = win[m+1]，右 = win[m+2]
//  代价：纯门电路，不增加缓存和流水级
// ----------------------------------------------------------------------------
module bin_morph3x3 (
    input  wire [1:0] I_op,
    input  wire [5:0] I_wt,
    input  wire [5:0] I_wm,
    input  wire [5:0] I_wb,
    output wire [3:0] O_bin
);
    genvar gk;
    generate
        for (gk = 0; gk < 4; gk = gk + 1) begin : MORPH3
            wire tl = I_wt[gk],   tm = I_wt[gk+1], tr = I_wt[gk+2];
            wire ml = I_wm[gk],   mm = I_wm[gk+1], mr = I_wm[gk+2];
            wire bl = I_wb[gk],   bm = I_wb[gk+1], br = I_wb[gk+2];

            wire all9 = tl & tm & tr & ml & mm & mr & bl & bm & br;
            wire any9 = tl | tm | tr | ml | mm | mr | bl | bm | br;

            assign O_bin[gk] = (I_op == 2'b01) ? all9 :
                               (I_op == 2'b10) ? any9 : mm;
        end
    endgenerate
endmodule

// ----------------------------------------------------------------------------
//  辅助：96 位（4 x 24bit，p0 在最高位）-> 128 位（4 x {8'h00, RGB}）
//        纯组合，与工程内 data128_96 互为逆变换，往返零延迟。
// ----------------------------------------------------------------------------
module data96_128_pa (
    input  wire [95:0]  I_tdata,
    output wire [127:0] O_tdata
);
    assign O_tdata = { 8'h00, I_tdata[95:72],
                       8'h00, I_tdata[71:48],
                       8'h00, I_tdata[47:24],
                       8'h00, I_tdata[23:0]  };
endmodule
// ===== END inline sobel_edge_128.v =====
