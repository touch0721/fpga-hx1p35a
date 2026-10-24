/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2023/03/23
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/
/*********ae_set �ֶ��������AE ***********
--�汾��1.0
*********************************************************************/

module ae_set
(
input wire I_clk,
input wire I_rst,
input wire [3:0]I_btn,
input wire I_ae_cfg_done,
input wire I_cam_cfg_done,
output wire O_ae_req,
output wire [15:0] O_ae,
output wire [15:0] O_ag,
output wire [3:0]  O_g_thresh,
output wire [7:0]  O_y_thresh,
// KEY4 循环切换"几何筛选的目标长宽比"，3 位共 8 档，每档 ±30% 容差
//（档 2 = 3:4 日麻牌面，档 4 = 5:4，详见 sobel_edge_128 里的档位表）
output wire [7:0]  O_bnd_ratio,     // KEY4 长按：填充阈值 0..214（像素）
// KEY4 长按：投影阈值档（0=全开放(默认) 1=3:4 2=1:2 3=1:3）
output wire [1:0]  O_mid_ratio,
output wire        O_k4_en            // KEY4 计数器使能（按一下开，再按一下关）
);
	

reg 	[19:0] 	rc_ae_tick = 0; 
reg 			r_ae_tick = 0; 
reg 	[4:0] 	rc_ag_tick = 0; 
reg 			r_ag_tick = 0; 
//	粗步进脉冲：每 8 个 r_ae_tick 一次（约 34 ms），用于曝光与边缘阈值
reg 	[2:0] 	rc_a8_tick = 0; 
reg 			r_a8_tick = 0; 
	
always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		rc_ae_tick <= 0; 
		r_ae_tick <= 0; 
		rc_ag_tick <= 0; 
		r_ag_tick <= 0; 
		rc_a8_tick <= 0; 
		r_a8_tick <= 0; 
	end else begin
	//	Trigger AE Inc / Dec per 1 ms. 
		rc_ae_tick <= rc_ae_tick + 1; 
		r_ae_tick <= 0; 
		if(rc_ae_tick >= 99999) begin
			rc_ae_tick <= 0; 
			r_ae_tick <= 1; 
		end 
	//	Trigger AG Inc / Dec per 32 ms. 
		rc_ag_tick <= rc_ag_tick + r_ae_tick; 
		r_ag_tick <= (&rc_ag_tick) && r_ae_tick; 
	//	Trigger the coarse step per 8 ms（8 x r_ae_tick, 约 34 ms）. 
		rc_a8_tick <= rc_a8_tick + r_ae_tick; 
		r_a8_tick <= (&rc_a8_tick) && r_ae_tick; 
	end
end
	
//	板载 4 键（低有效，按下=0），r_button_1 为按下标志。三个量都**只递增，到顶清零**：
//	  I_btn[0] -> KEY1 曝光   O_ae     每次 +64 半行（约 +0.95ms），到 2244 后回 0
//	  I_btn[1] -> KEY2 边缘/梯度阈值 G_THRESH  每次 +1，1..8 走完回到 0
//	  I_btn[2] -> KEY3 亮度二值化阈值 Y_THRESH 每次 +1，0..255 走完回到 0
//	  I_btn[3] -> KEY4 几何筛选目标长宽比 每按一次 +1（3 位共 8 档，循环）
//	步进速率：曝光与边缘阈值用 r_a8_tick（约 34 ms 一档）；
//	          亮度阈值用 r_ae_tick（约 4.2 ms 一档，0..255 全程约 1.1 s）。
//	注意：按住才步进，轻点一下可能不落在 tick 上；想看变化请按住约 0.1 s 以上。
reg 	[3:0] 	r_button_0 = 0, r_button_1 = 0; 
// 原厂720p60模式VTS=1000，曝光以半行计，手册表2-2给出最大值为2*VTS-10=1990。
// 增益控制144映射为0x3e08=0x1f、0x3e09=0x3f、0x3e06=0x00、0x3e07=0x80，
// 即15.75倍模拟增益（~16x规格）、1倍数字增益。
//
// 曝光值 = VTS=1127（1080p60 原厂表）下的上限 2*VTS-10 = 2244 半行：
//   1 行 = HTS/PCLK = 2852/192MHz = 14.854us；曝光单位是 1/2 行。
//   2244 半行 = 1122 行 = 16.667ms，已接近整帧（帧周期 1127 行 = 16.741ms，59.7fps）。
//   ★ 注意：16.67ms < 20ms（国内 50Hz 照明的光周期），所以工频闪烁**没有**被解决，
//     这是本轮“先不管频闪、先上 1080p60”的既定取舍。要消闪需要帧周期或曝光
//     达到 20ms 的整数倍，即必须把 VTS 加到 2244（约 30fps），见文档说明。
reg 	[15:0] 	r_ae_set = 2244, r_ag_set = 144;
// Sobel 边缘判定阈值：|gx|+|gy| 在二值图下取值 0..8，所以有效范围就是 1..8。
//   1 = 只要有梯度就算边缘（最粗、最灵敏，噪声也进来）
//   4 = 只保留干净边界   8 = 只保留对角线角点
reg 	[3:0] 	r_g_set = 4'd4;      // 边缘阈值：只留强边缘（grad=|gx|+|gy| 范围 0..8；8 是理论极限=全黑）
// 二值化阈值：作用在 gamma 之后的 Y 上，0..255 全范围可调。
//   默认 128 等效"原始亮度 >= 95（满量程 37%）判为白"
reg 	[7:0] 	r_y_set = 8'd128;
always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		r_button_0 <= 0; 
		r_button_1 <= 0; 
		r_ae_set <= 2244;   // 与初值一致：VTS=1127（1080p60）下的最大曝光，16.667ms
		r_ag_set <= 144; 
		r_g_set  <= 4'd4;
		r_y_set  <= 8'd128;
	end else begin
	//	When button pressed, r_button_1 bit is 1. 
		r_button_0 <= ~I_btn; 
		r_button_1 <= r_button_0; 
	
		if(r_button_1[0] && r_a8_tick) begin
			// KEY1：曝光 +64 半行，到顶清零（2244 = 2*VTS-10，VTS=1127）
			if(r_ae_set >= 16'd2244)
				r_ae_set <= 16'd0; 
			else if(r_ae_set + 16'd64 >= 16'd2244)
				r_ae_set <= 16'd2244; 
			else
				r_ae_set <= r_ae_set + 16'd64; 
		end 
		// KEY2 原功能（边缘/梯度阈值 G_THRESH）已固定，不再可调。
		// KEY2 现在用来调"填充阈值"（见下方 r_thr）
		// ★ 边缘阈值固定为 4：只留强边缘
		//   grad = |gx|+|gy|，作用在二值图上，范围 0..8；8 是理论极限（等于全黑）
		r_g_set <= 4'd4;
		if(r_button_1[2] && r_ae_tick) begin
			// KEY3：亮度二值化阈值 +1，255 到顶后清零
			r_y_set <= (r_y_set >= 8'd255) ? 8'd0 : (r_y_set + 8'd1); 
		end 
		// KEY4 (r_button_1[3]) 本轮未使用
		end
	end

// ---- KEY2（引脚 A9）：唯一功能 = 填充阈值（一行至少几个白像素才算"被填充"）----
//   长按 KEY2：每约 34ms **+2 档**，走 0..160 循环；
//   下游左移 3 位 -> 实际 0..1280 个像素。
//   ★ 阈值只影响"哪些行被填充"，**不参与颜色**（颜色由竖线序号决定）。
//   ★ 本文件 I_rst 是**高电平有效**，必须用 posedge I_rst + if(I_rst)。
reg [7:0] r_thr = 8'd1;                // 档位 0..160
always @(posedge I_clk or posedge I_rst) begin
    if(I_rst) r_thr <= 8'd1;
    else if(r_button_1[1] && r_a8_tick) begin        // 按住 KEY2 + 34ms 一拍 -> 步进 +2
        r_thr <= (r_thr >= 8'd158) ? 8'd0 : (r_thr + 8'd2);
    end
end
assign O_bnd_ratio = r_thr;            // 档位 0..160（下游 x8 -> 0..1280）
// ---- KEY4（引脚 C7）：计数器使能开关（按一下开，再按一下关）----
//   按 KEY4 -> 翻转 r_k4_en；使能后 ISP 每帧重新数"被填充的行数" = 高度 H，
//   并据此算出宽度 W = H*4/5，从左边界起每 W 像素换一次颜色。
//   未使能时计数器保持上次的值不动。
reg r_k4_en = 1'b0;
reg r_k4_d  = 1'b0;
always @(posedge I_clk or posedge I_rst) begin
    if(I_rst) begin
        r_k4_en <= 1'b0;
        r_k4_d  <= 1'b0;
    end else begin
        r_k4_d <= r_button_1[3];
        if(r_button_1[3] && !r_k4_d) r_k4_en <= ~r_k4_en;   // 按下沿 -> 翻转
    end
end
// ★ 恢复固定 0：这个端口还接到 bin_proj_bound 的 I_mid_ratio（列噪声窗口档位），
//   之前被我拿去做 KEY4 标志，会造成"按 KEY4 时列门限跟着变"的串扰，已还原。
assign O_mid_ratio = 2'd0;      // 固定 0（列噪声窗口档位不受按键影响）
assign O_k4_en      = r_k4_en;   // KEY4 计数器使能，独立端口，无串扰

assign O_g_thresh = r_g_set;
assign O_y_thresh = r_y_set;
	//	Update AG / AE when changed. 
reg 	[15:0] 	r_ae ; 
reg 	[15:0] 	r_ag ; 
reg 			r_ae_req = 0; //synthesis keep
assign O_ae = r_ae;
assign O_ag = r_ag;
assign O_ae_req = r_ae_req;

always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		r_ae_req <= 0; 
		r_ae     <= 1990; 
		// Start from a value different from r_ag_set.  This deliberately
		// causes one grouped AE/AG write after the sensor setting completes;
		// otherwise the requested default gain would never reach SC500CS.
		r_ag     <= 17; 
	end else if(r_ae_req) begin
		r_ae_req <= 0; 
	end else if(((r_ae != r_ae_set) || (r_ag != r_ag_set))&&(I_cam_cfg_done==1'b1)&&(I_ae_cfg_done==1'b1)) begin
		r_ae     <= r_ae_set; 
		r_ag     <= r_ag_set; 
		r_ae_req <= 1; 
	end
end


endmodule


	
