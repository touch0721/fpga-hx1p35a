

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    // 开发板板载 4 个独立按键 KEY1~KEY4（按下为低电平，松开为高电平）：
    //   I_button[0]=D5 KEY1  I_button[1]=A9 KEY2  I_button[2]=B9 KEY3  I_button[3]=C7 KEY4
    inout wire [3:0]  I_button,

    // 拨码开关 SW1 = A4：原 RGB 彩图 / 二值(边缘)图 选择，拨上(1)=二值图
    input wire        I_sw1,

    // 算法观测点选择（拨上=1），用于定位横/竖线出自哪一级；优先级 SW4 > SW3 > SW2：
    //   SW2 = B4  观测二值化输出（开运算之前，**没有行缓存**）
    //   SW3 = E11 观测 Sobel 边沿输出（闭运算之前，3 级行缓存）
    //   SW4 = D11 观测 Y 灰度（RGB转YCbCr 的 Y，**没有行缓存**）
    // 三个都不拨 = 正常显示（由 SW1 决定彩图 or 开->Sobel->闭）
    input wire        I_sw2,
    input wire        I_sw3,
    input wire        I_sw4,

    output wire       O_screen_pwm,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
	(* keep = "true" *) wire [15:0] S_csi_data_length;
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;

	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;
    wire        S_csi_packet_start;
    reg         S_csi_length_active_d;
    reg [10:0]  S_mipi_line_count;
    reg         S_mipi_length_error;
    reg         S_mipi_lane_error;
    reg         S_mipi_frame_seen;
    reg         S_mipi_frame_done;
    reg         S_mipi_last_length_error;
    reg         S_mipi_last_line_error;
    reg         S_mipi_last_lane_error;
    wire[3:0]   S_mipi_diag_status_async;
    reg [3:0]   S_mipi_diag_status_sync1;
    reg [3:0]   S_mipi_diag_status_sync2;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    // A RAW10 line contains 1920 * 10 / 8 = 2400 bytes.  The CSI parser
    // reports the number of 16-bit lane words, therefore the expected value
    // is 2400 / 2 = 1200 = 16'h04b0.  Count one long packet whenever that
    // decoded length changes from zero to non-zero.
    assign S_csi_packet_start = (S_csi_data_length != 16'd0) &&
                                !S_csi_length_active_d;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_csi_length_active_d      <= 1'b0;
            S_mipi_line_count          <= 11'd0;
            S_mipi_length_error        <= 1'b0;
            S_mipi_lane_error          <= 1'b0;
            S_mipi_frame_seen          <= 1'b0;
            S_mipi_frame_done          <= 1'b0;
            S_mipi_last_length_error   <= 1'b0;
            S_mipi_last_line_error     <= 1'b0;
            S_mipi_last_lane_error     <= 1'b0;
        end
        else begin
            S_csi_length_active_d <= (S_csi_data_length != 16'd0);

            if(S_csi_frame_start) begin
                S_mipi_line_count   <= 11'd0;
                S_mipi_length_error <= 1'b0;
                S_mipi_lane_error   <= 1'b0;
                S_mipi_frame_seen   <= 1'b1;
            end
            else begin
                if(S_csi_packet_start) begin
                    S_mipi_line_count <= S_mipi_line_count + 11'd1;
                    if(S_csi_data_length != 16'h04b0)
                        S_mipi_length_error <= 1'b1;
                end

                if(S_lane_error_any)
                    S_mipi_lane_error <= 1'b1;
            end

            // Publish only a completed frame, so the HDMI-domain diagnostic
            // sees a stable result rather than counters changing every line.
            if(S_csi_frame_end) begin
                S_mipi_frame_done        <= 1'b1;
                S_mipi_last_length_error <= S_mipi_length_error;
                S_mipi_last_line_error   <= (S_mipi_line_count != 11'd720);
                S_mipi_last_lane_error   <= S_mipi_lane_error |
                                            S_lane_error_any;
            end
        end
    end

    // Diagnostic result codes consumed by hdmi_mixer DEBUG_MODE=4:
    // 0=config incomplete, 1=no HS traffic, 2=no completed CSI frame,
    // 3=lane error, 4=wrong RAW10 line length, 5=wrong lines/frame,
    // 6=the observed MIPI frame structure is correct, 7=I2C error.
    // I2C error has the highest priority because O_cfg_done only indicates
    // that the write sequencer reached its end; it does not prove that every
    // sensor register write was acknowledged successfully.
    assign S_mipi_diag_status_async =
        S_cam_i2c_error              ? 4'd7 :
        !S_cam_cfg_done              ? 4'd0 :
        !S_dbg_hs_seen               ? 4'd1 :
        !(S_mipi_frame_seen &&
          S_mipi_frame_done)         ? 4'd2 :
        S_mipi_last_lane_error       ? 4'd3 :
        S_mipi_last_length_error     ? 4'd4 :
        S_mipi_last_line_error       ? 4'd5 : 4'd6;

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_mipi_diag_status_sync1 <= 4'd0;
            S_mipi_diag_status_sync2 <= 4'd0;
        end
        else begin
            S_mipi_diag_status_sync1 <= S_mipi_diag_status_async;
            S_mipi_diag_status_sync2 <= S_mipi_diag_status_sync1;
        end
    end

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = S_mipi_diag_status_sync2;
	
	// J9 pin 37 is connected to the module's PWDNB/XSHUTDN input.
	// Hold the sensor in power-down after PLL lock, then allow its internal
	// power-up sequence to finish before the first I2C transaction.
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  // 板载 4 键直连 ae_set 的 4 个按键输入，映射为：
  //   KEY1(D5)/I_button[0] -> I_btn[0]：二值化阈值 (Y_THRESH) 递减
  //   KEY2(A9)/I_button[1] -> I_btn[1]：二值化阈值 (Y_THRESH) 递增
  //   KEY3(B9)/I_button[2] -> I_btn[2]：Sobel 边缘阈值 G_THRESH 递减
  //   KEY4(C7)/I_button[3] -> I_btn[3]：Sobel 边缘阈值 G_THRESH 递增
  // 曝光与模拟增益不再由按键控制（固定 1990 / 144）。
  // 按键为低有效（按下=0），与 ae_set 内部 ~I_btn 的逻辑一致；
  // pin.adc 中这 4 个脚必须保持 PULLTYPE = PULLUP，否则空闲为低会被判为"一直按住"。
  //
  // 拨码 SW1(A4)：手册第⑯节，拨上=1。这里作为"原彩图 / 二值图"选择：
  //   拨上(1) -> 输出二值化+边缘处理结果
  //   拨下(0) -> 输出原始 RGB 彩图（旁路整条 Sobel 通路，便于对比）
  // pin.adc 中 A4 必须 PULLTYPE = PULLDOWN，且因为 I_sw1 是纯输入，
  // 不能写 DRIVESTRENGTH（TD 会报 INPUT direction 不支持）。
  wire [3:0] S_sobel_g_thresh;
  wire [7:0] S_sobel_y_thresh;
  // ★ SW1 原功能（S_sobel_en = 边缘图 + 红色边界）已悬空，见下方 isp_top 连接
  wire       S_sobel_en = I_sw4;      // ★ SW4：Sobel 边缘检测（走 ISP 的边缘图通路）
  wire [7:0] S_bnd_ratio;      // KEY2 长按：填充阈值档位 0..160（x8）
wire       S_k4_en;           // KEY4：计数器使能
wire [4:0] S_line_cnt;        // 宽度竖线条数
wire [191:0] S_line_x;        // 宽度竖线坐标列表
wire [11:0]  S_fill_h;         // KEY4 计数器值（被填充行数最大值）
wire [11:0]  S_roi_x1;         // 左边界列号
wire [11:0]  S_roi_x2;         // 右边界列号
// 左右边界距离 = xr - xl + 1（给 OSD 显示用）
wire [11:0]  S_disp_d = (S_roi_x2 >= S_roi_x1) ? (S_roi_x2 - S_roi_x1 + 12'd1) : 12'd0;
wire [31:0]  S_isp_y_out;      // ISP 的 Y（4 像素/拍），给帧差法用
wire         S_isp_y_valid;    // 与 S_isp_y_out 严格同拍
wire         S_isp_y_last;
wire         S_isp_y_user;
wire [7:0]   S_isp_col4;       // ★ ISP 的色系码（4 像素 x 2 位，与 Y 严格同拍）
//   ★ 为什么不是 Cb/Cr：把 Cb/Cr 引出来会让 rgb_to_ycbcr_4 里 24 个常量乘法器
//     从死逻辑变活逻辑，实测 DSP 33->49 超上限，布局器直接失败。色系码只用比较器。
// ★ 麻将识别（tile_recog_onchip）的输出 —— OSD 上的 N/C/A 三行就是显示这些
//   （阶段 2 的细粒度计数 dots/bars/cells/maxc 走 P/G 两行）
(* keep = "true" *) wire         S_tr_v;          // ★ 选中槽的特征（选槽在识别域做）
(* keep = "true" *) wire [21:0]  S_tr_pixcnt;    //   前景像素数
(* keep = "true" *) wire [31:0]  S_tr_colsig;    //   颜色签名（红/绿/蓝/无彩）
(* keep = "true" *) wire         S_tr_blank;     //   白板判定
(* keep = "true" *) wire         S_tr_ar_ok;     //   宽高比通过（日麻 0.708 窗口）
(* keep = "true" *) wire [11:0]  S_tr_x1, S_tr_x2, S_tr_y1, S_tr_y2;  // 选中牌的框
(* keep = "true" *) wire [3:0]   S_tr_sel;       // mixer -> 识别域：看哪个槽
(* keep = "true" *) wire [3:0]   S_tr_hit_slot;
(* keep = "true" *) wire [4:0]   S_tr_nslot;     // 本帧切出几张（诊断）
(* keep = "true" *) wire         S_tr_done;
// ★ 阶段 2：4x4 格位图（数筒）+ 逐像素列投影（数条）的结果，都属【选中槽】
(* keep = "true" *) wire [4:0]   S_tr_dots;      //   连通块数（筒候选）
(* keep = "true" *) wire [4:0]   S_tr_bars;      //   列投影峰数（条候选）
(* keep = "true" *) wire [8:0]   S_tr_cells;     //   位图占据格数（诊断）
(* keep = "true" *) wire [8:0]   S_tr_maxc;      //   最大连通块格数（诊断）
// ★ 固定 ROI「有没有牌」检测的输出（触发识别用；SW4 使能、KEY4 抓背景）
(* keep = "true" *) wire         S_roi_a_present;   // 主框（≈13 张牌那排）有牌
(* keep = "true" *) wire         S_roi_b_present;   // 右侧小框有东西
(* keep = "true" *) wire [11:0]  S_roi_a_count;     // 主框运动列条数（诊断/标阈值）
(* keep = "true" *) wire [11:0]  S_roi_b_count;
(* keep = "true" *) wire         S_roi_bg_rdy;      // 背景已抓
wire [14:0]  S_fd_q_addr;      // 帧差位图查询地址（由 mixer 给）
wire [31:0]  S_fd_q_word;      // 帧差位图查询结果
wire [5:0]   S_fdv_obj_cnt;    // ★ 帧差：目标表已用槽数（上板诊断用）
wire [15:0]  S_fdv_frame_cnt;  // 帧差诊断：收到的帧数
wire [11:0]  S_fdv_mot_cnt;    // 帧差诊断：上一帧运动块数
wire         S_fdv_bg_rdy;     // 帧差诊断：背景帧是否已建立
  wire [1:0] S_mid_ratio;      // KEY4 长按：投影阈值比例 0=1:1 1=2:3 2=1:2 3=1:3
  // 算法观测点：优先级 SW3 > SW2；两个都不拨 = 正常显示（此时由 SW1 决定彩图/二值）
  // ★ SW4 已改用途：帧差法（片上版）使能。原来 SW4 选的"Y 灰度观测点"取消。
  wire [1:0] S_dbg_sel  = I_sw3 ? 2'b10 : (I_sw2 ? 2'b01 : 2'b00);
  // ★ SW4(D11)：帧差法使能，拨上 = 开启
  wire       S_fdiff_en = I_sw3;      // ★ SW3：帧差使能（合并：使能+显示物体个数）
  // ★ SW1 原功能（边缘图 + 红色边界）已悬空：S_sobel_en 固定 0
  //   新功能：SW1 拨上 = 显示帧差的二值图（运动位黑/白）
  wire       S_fdimg_en = I_sw1;      // SW1：显示帧差二值图
  // ★ KEY4(C7)：原"计数器使能"已悬空，改为【抓取/冻结背景帧】
  //   按下后 r_k4_en 翻转：=1 持续抓取背景；=0 冻结背景，开始做基于背景的物体检测
  // ★ 多目标方框：8 个独立框（每个 12 位坐标 + 1 位有效）
  wire [7:0]  S_fdv_bv;
  wire [95:0] S_fdv_bx1, S_fdv_bx2, S_fdv_by1, S_fdv_by2;

  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_btn(I_button),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag),
      .O_g_thresh(S_sobel_g_thresh),
      .O_y_thresh(S_sobel_y_thresh),
      .O_bnd_ratio(S_bnd_ratio),
           .O_k4_en   (S_k4_en   ),
      .O_mid_ratio(S_mid_ratio)
  );
  
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_cam_i2c_ready), // SC500CS power-up delay completed
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done),  //AE配置完成
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        // SC500CS 1920x1080@60 原厂表 HTS=2852、VTS=1127、960 Mbps/Lane -> 像素时钟 192 MHz，
        // 2-Lane RAW10 -> 480 Mbps/Lane（DDR 时钟 240 MHz，SDC period 4.167 ns）。
        // 接收均衡/VGA 复位为 IP 默认 0dB：数值越大补偿越强，原 3dB/8dB 是
        // 为 SC520CS 的 828 Mbps 选的。若上板出现花屏/闪点，按
        // "-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB" 逐档微调，
        // 并配合 I_data_lane*_in_delay / I_clk_lane_in_delay 扫相位。
        .HS_EQUALIZER          ( "0dB"   ),// 480 Mbps/Lane接收均衡
        .HS_VGA_GAIN           ( "0dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        ),
.O_data_length         ( S_csi_data_length )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),      // 1080p60：每行 1920/4 = 480 拍
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// SC500CS原厂HD模式已经直接输出1280x720 RAW10，无需FPGA窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.I_sobel_en      ( S_sobel_en      ),   // ★ SW4：边缘图输出
.I_sobel_y_thresh(S_sobel_y_thresh),
.I_sobel_g_thresh(S_sobel_g_thresh),
        .I_dbg_sel       (1'b0            ),   // 观测点选择关闭（SW2/SW3 已不接这里）
        .I_bnd_en        (1'b0            ),   // ★ SW1 原功能悬空（红色边界/填充也关闭）
             .I_bnd_ratio     (S_bnd_ratio     ),   // KEY2 长按：填充阈值档位
     // ★ KEY4 原功能（计数器使能）悬空：KEY4 改用于"抓取背景帧"
     .I_k4_en         (1'b0            ),   // KEY4：原计数器使能（已悬空）
     .O_line_cnt      (S_line_cnt      ),
     .O_line_x        (S_line_x        ),
     .O_fill_h        (S_fill_h        ),
     .O_roi_x1        (S_roi_x1        ),
     .O_roi_x2        (S_roi_x2        ),
     .O_y_out         (S_isp_y_out     ),   // 帧差法用的 Y（4 像素/拍）
     .O_y_valid       (S_isp_y_valid   ),   // 与 Y 严格同拍的控制信号
     .O_y_last        (S_isp_y_last    ),
     .O_y_user        (S_isp_y_user    ),
     .O_col4          (S_isp_col4      ),   // ★ 麻将识别用色系码（与 Y 同拍）
        .I_mid_ratio     (S_mid_ratio     ),   // KEY4 长按：投影阈值比例
        // 多高峰选择：选第几组列段 / 行段（先固定 0；改成寄存器就能外部遍历所有高峰）
        .I_roi_idx       (4'd0            ),
        .I_roi_yidx      (4'd0            ),
        .O_roi_y1        (),
        .O_roi_y2        (),
        .O_roi_valid     (),
        .O_roi_cnt       (),
        .O_rcnt          (),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );


    // ========================================================================
    //  帧差法（普通运动检测）★片上帧缓存版★ —— SW4 使能，完全独立，不碰 DDR
    //    8x8 块降采样 -> 同址先读后写拿上一帧 -> 阈值比较 -> 块级投影 -> 方框
    // ========================================================================
    // ========================================================================
    //  帧差法【已按用户要求解除例化】—— 代码全部保留在仓库里
    //  （frame_diff_onchip.v + 本文件末尾的内联副本），这里只断开例化：
    //  未例化的模块会被综合自动剪掉，不占资源。
    //  恢复方法：把下面这些 assign 换回 frame_diff_onchip 例化即可
    //  （原例化参数：IMG 1280x720, DIFF_THR=90, XCOMP=1, YCOMP=2, MARGIN=4,
    //    MIN_HITS=2, MIN_SIZE=3, HOLD_FRM=8, MAXO=8；I_en=SW4、I_bg_cap=KEY4）
    // ========================================================================
                                        

    // ========================================================================
    //  固定 ROI「有没有牌」检测【已按用户要求解除例化】
    //    用户新方案：不再用帧差做触发，改成【固定 14 个牌位一直跑识别】。
    //    代码（roi_motion_onchip.v + 末尾内联副本）全部保留，恢复只需把下面
    //    这些 assign 换回例化（参数：DIFF_THR=30 / THR_A=40 / THR_B=3 / HOLD=8 /
    //    AVG_SH=10，AX1=56 AX2=1047 BX1=1096 BX2=1167 CY1=240 CY2=351；I_en=SW4、
    //    I_bg_cap=KEY4 抓空桌背景）。
    // ========================================================================
    assign S_roi_a_present = 1'b0;
    assign S_roi_b_present = 1'b0;
    assign S_roi_a_count   = 12'd0;
    assign S_roi_b_count   = 12'd0;
    assign S_roi_bg_rdy    = 1'b0;


    // ========================================================================
    //  ★ 识别功能【已按用户要求全部移除】
    //    用户新方案（推翻之前所有方案）：
    //      1) 识别、帧差、ROI 检测全部不参与
    //      2) SW3 = 二值化显示
    //      3) 只在屏幕上按固定间距画红线把 14 个牌位分开
    //    ★ 这里不例化 = 综合时整个模块被剪掉，不占任何资源；
    //      模块源码仍在 user_source/hdl_source/tile_recog_onchip.v 和 git 历史里，随时可恢复。
    //    ★ 下面这些 assign 只是把 mixer 端口上还留着的 S_tr_* 拉成常量
    //      （端口没删，避免 mixer 端口表大改；反正全是常量，综合后也是 0 资源）。
    // ========================================================================
    assign S_tr_v        = 1'b0;
    assign S_tr_pixcnt   = 22'd0;
    assign S_tr_colsig   = 32'd0;
    assign S_tr_blank    = 1'b0;
    assign S_tr_ar_ok    = 1'b0;
    assign S_tr_x1       = 12'd0;
    assign S_tr_x2       = 12'd0;
    assign S_tr_y1       = 12'd0;
    assign S_tr_y2       = 12'd0;
    assign S_tr_hit_slot = 4'd0;
    assign S_tr_nslot    = 5'd0;
    assign S_tr_done     = 1'b0;
    assign S_tr_dots     = 5'd0;
    assign S_tr_bars     = 5'd0;
    assign S_tr_cells    = 9'd0;
    assign S_tr_maxc     = 9'd0;

    
// ===================== 帧差（运动检测）例化 =====================
frame_diff_onchip #(
    .BC(160), .BR(90),                       // ★ 720p 块网格（1280/8、720/8）
    .DIFF_THR(8'd70),                        // ★ 阈值 70（原 90，降低后更灵敏）
    .MARGIN(4), .MIN_HITS(2), .MIN_SIZE(3),  // ★ 同样恢复原例化值
    .HOLD_FRM(8), .MAXO(8), .XCOMP(1), .YCOMP(2)
) u_frame_diff (
    .I_pix_clk   ( S_csi_rx_clk   ),
    .I_rst_n     ( S_rst_n        ),
    .I_pix_valid ( S_isp_y_valid  ),
    .I_pix_last  ( S_isp_y_last   ),
    .I_pix_user  ( S_isp_y_user   ),
    .I_y_now     ( S_isp_y_out    ),
    .I_en        ( S_fdiff_en     ),
    .I_bg_cap    ( ~I_button[3]   ),   // ★ KEY4：按下抓当前画面当背景帧
    .O_bv        ( S_fdv_bv       ),
    .O_bx1       ( S_fdv_bx1      ),
    .O_bx2       ( S_fdv_bx2      ),
    .O_by1       ( S_fdv_by1      ),
    .O_by2       ( S_fdv_by2      ),
    .O_frame_cnt ( S_fdv_frame_cnt),
    .O_mot_cnt   ( S_fdv_mot_cnt  ),
    .O_obj_cnt   ( S_fdv_obj_cnt  ),
    .I_q_addr    ( S_fd_q_addr    ),
    .O_q_word    ( S_fd_q_word    ),
    .O_bg_rdy    ( S_fdv_bg_rdy   )
);

video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
    .H_FrameSize  ( 1650 ),   // ★ 回退到 2222 之前的值（2222 导致屏幕无显示）
                              //   150 MHz / (2223 x 1125) = 59.979 Hz
                              //   ★ 想要严格 60 Hz 必须把像素时钟做成 148.5 MHz，
                              //     那时 H_FrameSize 用标准的 2200。
                              //     在 150 MHz 下无论 2222 还是 2223 都不是准确 60，
                              //     而改动会破坏显示，所以保持原值不动。
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720 ),
        .V_FrameSize  ( 750 ),
        .V_SyncStart  ( 725 ),
        .V_SyncEnd    ( 730 )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720 ),
        // Normal camera display.  Set to 4 only when the MIPI diagnostic
        // full-screen status page is needed.
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        // OSD 数字显示：KEY4 计数器值 + 左右边界距离；logo 钩子本版接 0
        .I_disp_h        ( {4'd0, S_fdv_obj_cnt} ),   // ★ H 行 = 目标表已用槽数
        .I_disp_d        ( S_disp_d           ),
        .I_logo_hit      ( 1'b0               ),
        .I_logo_color    ( 24'd0              ),
        // ★ 帧差已解除例化：SW1 的二值图跟着停用（否则拨上 SW1 会得到全黑画面）
        .I_fdimg_en      ( I_sw1              ),   // ★ SW1：普通视频输出 + 安路 logo 叠加（最高优先级）
        .O_fd_q_addr     ( S_fd_q_addr        ),
        .I_fd_q_word     ( S_fd_q_word        ),
        // ★ 帧差已解除例化：白色测试框也停用（现在屏上只留摄像头画面 + 红框）
        .I_tbox_en       ( 1'b0               ),   // SW4 已不再用于帧差二值图
        // ★ 固定 ROI 检测结果（用来给红框染成绿/红 + OSD 显示计数）
        .I_a_present     ( S_roi_a_present    ),
        .I_b_present     ( S_roi_b_present    ),
        .I_a_count       ( S_roi_a_count      ),
        .I_b_count       ( S_roi_b_count      ),
        // 帧差法诊断（左上角第 3/4 行）
        .I_fd_frame_cnt  ( S_fdv_frame_cnt    ),
        .I_fd_mot_cnt    ( S_fdv_mot_cnt      ),
        .I_fd_osd_en     ( I_sw3              ),   // ★ SW3：显示帧差物体个数
        // 帧差法运动方框（SW4 打开时有效）
        .I_mb_v          ( S_fdv_bv           ),
        .I_mb_x1         ( S_fdv_bx1          ),
        .I_mb_x2         ( S_fdv_bx2          ),
        .I_mb_y1         ( S_fdv_by1          ),
        .I_mb_y2         ( S_fdv_by2          ),
        // ★ 麻将识别特征（准静态，帧末更新一次）+ SW2/SW3 控制轮播
        //   SW2 拨上 = 冻结轮播（停在当前槽）；SW3 拨上 = 只轮播有效槽
        //   ★ 之前这里写的是 S_tr_valid（拼错，是个未声明的隐式 wire，恒 0），
        //     导致 mixer 里 I_tr_v 永远是 0、SW3"跳过空槽"根本没生效；现已改回 S_tr_v
        .I_tr_v          ( S_tr_v             ),
        .I_tr_pixcnt     ( S_tr_pixcnt        ),
        .I_tr_colsig     ( S_tr_colsig        ),
        .I_tr_blank      ( S_tr_blank         ),
        .I_tr_ar_ok      ( S_tr_ar_ok         ),
        .I_tr_x1         ( S_tr_x1            ),
        .I_tr_x2         ( S_tr_x2            ),
        .I_tr_y1         ( S_tr_y1            ),
        .I_tr_y2         ( S_tr_y2            ),
        // ★ 阶段 2：细粒度计数（OSD 的 P/G 两行）
        .I_tr_dots       ( S_tr_dots          ),
        .I_tr_bars       ( S_tr_bars          ),
        .I_tr_cells      ( S_tr_cells         ),
        .I_tr_maxc       ( S_tr_maxc          ),
        .I_tr_freeze     ( ~I_button[2]       ),   // ★ KEY3：复位绿框 + 回到框0（原 SW2 已无功能）
        // ★ SW3 已从"轮播只停有效槽"改接到【二值化显示】
        .I_tr_skipinv    ( ~I_button[1]       ),   // ★ KEY2（按下=低电平 -> 取反给 mixer）
        .I_bin_en        ( I_sw2              ),   // ★ SW3：二值化显示
        .O_tr_sel        ( S_tr_sel           ),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );

    
endmodule

// ============================================================================
//  以下为内联的 frame_diff_onchip.v（避免往 TD 的 .al 文件列表里加新文件）
// ============================================================================
// ============================================================================
//  frame_diff_onchip.v — 基于【背景帧】的物体检测 ★多目标：邻域投票法★
// ----------------------------------------------------------------------------
//  一、检测
//    8×8 块降采样 -> 块平均 Y；与背景帧同位置块平均比较
//    KEY4：I_bg_cap=1 持续抓取背景；=0 冻结背景并检测
//    |本帧块平均 − 背景块平均| > DIFF_THR  ->  该块是"运动块"
//
//  二、多目标分离（邻域投票法，帧内实时，不需要运动图）
//    对每一个运动块 (bx, by)，向 16 个目标各投一票：
//        t_valid[j]==0                          -> 投"新目标"票
//        (bx,by) 落在 t 的邻域之外              -> 投"新目标"票
//        否则                                   -> 不投
//    · 全票（16'hFFFF）      -> 这是一个全新的目标，占用一个空槽
//    · 部分票（>0）          -> 落在"未投票那些目标"的邻域内 -> 扩展它们的 bbox
//    ★ 邻域判定同时用 x 和 y，所以"上下分开、左右重叠"的物体也能分开
//    ★ 帧首清空目标列表（每帧重新检测）
//    ★ 最多 16 个内部目标，对外输出前 4 个
//
//  三、I_en=0（SW4 关）时全部复位
// ============================================================================
module frame_diff_onchip #(
    parameter integer IMG_W    = 1280,
    parameter integer IMG_H    = 720,
    parameter integer BC       = IMG_W/8,          // 块列数 = IMG_W/8
    parameter integer BR       = IMG_H/8,          // 块行数 = IMG_H/8
    parameter [7:0]   DIFF_THR = 8'd12,        // 块平均绝对差阈值
    parameter integer NT       = 32,           // ★ 目标表容量 16 -> 32（放多个物体时表被占满 -> 全不识别）
    parameter integer MAXO     = 8,            // 对外输出方框数
    parameter integer MARGIN   = 2,            // 邻域外扩（块数），使跟踪更稳
    parameter integer MIN_HITS = 4,            // ★ 目标至少被命中几次才算有效（2->4，减少小框）
    parameter integer OPEN_EN  = 1,            // ★ 投票前对运动图做 3×3 开运算（去噪点）
    parameter integer MIN_SIZE = 4,            // ★ 目标最小尺寸（块数）：宽高都要 >= 这个值
    parameter integer HOLD_FRM = 8,           // ★ 输出框保持帧数（防闪烁）
    parameter integer XCOMP = 1,              // ★ 投票坐标 X 补偿（实测标定）
    parameter integer YCOMP = 2               // ★ 投票坐标 Y 补偿（最后一项不能有逗号）
) (
    input  wire        I_pix_clk,
    input  wire        I_rst_n,
    input  wire        I_pix_valid,
    input  wire        I_pix_last,
    input  wire        I_pix_user,
    input  wire [31:0] I_y_now,
    input  wire        I_en,
    input  wire        I_bg_cap,

    output reg  [MAXO-1:0]    O_bv,
    output reg  [12*MAXO-1:0] O_bx1,
    output reg  [12*MAXO-1:0] O_bx2,
    output reg  [12*MAXO-1:0] O_by1,
    output reg  [12*MAXO-1:0] O_by2,

    output reg  [15:0] O_frame_cnt,
    output reg  [11:0] O_mot_cnt,
    output reg  [4:0]  O_obj_cnt,
    // ★ 调试：运动二值图按【坐标查询】而不是流式输出
    //   因为帧差跟 ISP 输出同拍，而 mixer 跟 HDMI 显示同拍，两者差整整一帧，
    //   流式输出会被 mixer 当成"当前帧的图"用 -> 横条纹 + 频闪。
    //   所以要先把位图存下来，mixer 按显示坐标来查。
    input  wire [14:0] I_q_addr,        // 查询地址 = 块行×WPR + 块列/32
    output reg  [31:0] O_q_word,        // 该字（同步读，1 拍延迟）
    output reg         O_bg_rdy
);
    localparam integer LB_MAX = (IMG_W/4) - 1;
    localparam integer LN_MAX = IMG_H - 1;
    localparam integer WPR    = (BC + 31) / 32;   // 每行几个 32 位字

    wire [9:0] s_beat_sum = I_y_now[7:0] + I_y_now[15:8] +
                            I_y_now[23:16] + I_y_now[31:24];

    // ======================================================================
    //  ① 行/拍计数
    // ======================================================================
    reg  [8:0]  s_lb;
    reg  [10:0] s_line;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n)        begin s_lb <= 9'd0; s_line <= 11'd0; end
        else if (!I_en)      begin s_lb <= 9'd0; s_line <= 11'd0; end
        else if (I_pix_user) begin s_lb <= 9'd0; s_line <= 11'd0; end
        else if (I_pix_valid) begin
            if (s_lb == LB_MAX[8:0]) begin
                s_lb <= 9'd0;
                if (s_line == LN_MAX[10:0]) s_line <= 11'd0;
                else                        s_line <= s_line + 11'd1;
            end else s_lb <= s_lb + 9'd1;
        end
    end
    wire s_vend = I_pix_valid && (s_lb == LB_MAX[8:0]) && (s_line == LN_MAX[10:0]);

    wire [7:0]  s_bc    = s_lb[8:1];
    wire        s_bcend = s_lb[0];
    wire [7:0]  s_brow  = s_line[10:3];
    wire        s_lastr = (s_line[2:0] == 3'd7);

    reg s_user_d;
    always @(posedge I_pix_clk) s_user_d <= I_pix_user;
    wire s_fstart = I_pix_user & ~s_user_d;

    // ======================================================================
    //  ② 块累加 -> 块平均
    // ======================================================================
    reg  [9:0]  s_sum_a;
    wire [10:0] s_rowsum = s_sum_a + s_beat_sum;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) s_sum_a <= 10'd0;
        else if (!I_en) s_sum_a <= 10'd0;
        else if (I_pix_valid) begin
            if (s_bcend) s_sum_a <= 10'd0;
            else         s_sum_a <= s_beat_sum;
        end
    end

    reg  [7:0]  s_bc_r, s_brow_r;
    reg  [13:0] s_rs_r;
    reg         s_lastr_r, s_val_r, s_firstr_r;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_bc_r <= 8'd0; s_brow_r <= 8'd0; s_rs_r <= 14'd0;
            s_lastr_r <= 1'b0; s_val_r <= 1'b0; s_firstr_r <= 1'b0;
        end else begin
            s_val_r    <= I_en && I_pix_valid && s_bcend;
            s_bc_r     <= s_bc;
            s_brow_r   <= s_brow;
            s_rs_r     <= {3'd0, s_rowsum};
            s_lastr_r  <= s_lastr;
            s_firstr_r <= (s_line[2:0] == 3'd0);
        end
    end

    reg  [15:0] s_sum_ram [0:BC-1];
    wire [15:0] s_part = s_sum_ram[s_bc_r];
    wire [15:0] s_new  = s_firstr_r ? {2'd0, s_rs_r} : (s_part + {2'd0, s_rs_r});

    reg  [7:0]  s_bc_s, s_brow_s, s_avg_s;
    reg         s_settle_s;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_bc_s <= 8'd0; s_brow_s <= 8'd0; s_avg_s <= 8'd0; s_settle_s <= 1'b0;
        end else begin
            s_bc_s     <= s_bc_r;
            s_brow_s   <= s_brow_r;
            s_avg_s    <= s_new[13:6];
            s_settle_s <= s_val_r && s_lastr_r;
        end
    end
    // ★ 方案 A：块地址用累加（按光栅顺序，完成一个块 +1）
    //   不写死 160/5 也不用乘法器，任何 BC/BR 都对
    reg  [14:0] s_paddr_r;
    always @(posedge I_pix_clk or negedge I_rst_n)
        if (!I_rst_n)         s_paddr_r <= 15'd0;
        else if (s_fstart)    s_paddr_r <= 15'd0;
        else if (s_settle_s)  s_paddr_r <= s_paddr_r + 15'd1;   // 一个块的平均完成
    wire [14:0] s_paddr = s_paddr_r;

    always @(posedge I_pix_clk) begin
        if (I_en && s_val_r) begin
            if (s_lastr_r) s_sum_ram[s_bc_r] <= 16'd0;
            else           s_sum_ram[s_bc_r] <= s_new;
        end
    end

    // ======================================================================
    //  ③ 背景帧
    // ======================================================================
    reg  [7:0] s_bg_ram [0:(BC*BR)-1];
    reg        s_bg_rdy = 1'b0;
    wire       s_bg_write = I_bg_cap | ~s_bg_rdy;

    reg  [14:0] s_paddr_s;
    reg  [7:0]  s_avg_s2;
    reg         s_settle_s2;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_paddr_s <= 15'd0; s_avg_s2 <= 8'd0; s_settle_s2 <= 1'b0;
        end else begin
            s_paddr_s   <= s_paddr;
            s_avg_s2    <= s_avg_s;
            s_settle_s2 <= s_settle_s;
        end
    end

    reg  [7:0] s_bg_val, s_avg_cmp;
    reg        s_settle_cmp;
    always @(posedge I_pix_clk) begin
        s_bg_val     <= s_bg_ram[s_paddr_s];
        s_avg_cmp    <= s_avg_s2;
        s_settle_cmp <= s_settle_s2;
        if (I_en && s_settle_s2 && s_bg_write) s_bg_ram[s_paddr_s] <= s_avg_s2;
    end
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n)   s_bg_rdy <= 1'b0;
        else if (!I_en) s_bg_rdy <= 1'b0;
        else if (s_vend && s_bg_write) s_bg_rdy <= 1'b1;
    end
    always @(posedge I_pix_clk) O_bg_rdy <= s_bg_rdy;

    // ======================================================================
    //  ④ 运动判定（块级）
    // ======================================================================
    wire [7:0] s_da = s_avg_cmp, s_db = s_bg_val;
    wire       s_diff_big = (s_da > s_db) ? ((s_da - s_db) > DIFF_THR)
                                          : ((s_db - s_da) > DIFF_THR);
    wire       s_mot   = s_diff_big && ~s_bg_write && s_bg_rdy;
    wire       s_mot_v = I_en && s_settle_cmp && s_mot;

    reg  [7:0] s_bc_c, s_brow_c;
    always @(posedge I_pix_clk) begin
        s_bc_c <= s_bc_s; s_brow_c <= s_brow_s;
    end

    // ======================================================================
    //  ④b 开运算（先腐蚀后膨胀，3×3）—— 在投票之前去掉孤立噪点
    //      3 个 240 位行位图；每来一个运动块在 mrow_cur 对应位置置 1；
    //      块行切换时对 3 行做腐蚀+膨胀，得到"往回第 2 行"的开运算结果，
    //      再由一个小状态机逐位送进投票（by = brow-2）。
    // ======================================================================
    reg [BC-1:0] mrow_cur, mrow_p1, mrow_p2;   // 运动位（当前行 / 前1行 / 前2行）
    reg [BC-1:0] ero_p2,   ero_p1;             // 腐蚀结果（前2行 / 前1行）
    reg [7:0]    s_brow_d;
    reg          s_rowend;
    always @(posedge I_pix_clk) begin
        s_brow_d <= s_brow_c;
        s_rowend <= (s_brow_c != s_brow_d) && (s_brow_c != 8'd0);
    end

    // 逐位送出用的小状态机
    reg [7:0]  s_obx;          // 正在送的列
    reg [7:0]  s_oby;          // 这一批对应的块行（= 往回第 2 行）
    reg        s_obusy;
    reg [BC-1:0] s_obits;      // 开运算结果（待逐位送出）
    integer ci2;
    wire [BC-1:0] s_ero_mid;   // 中间行腐蚀（先用组合算，见下）

    // 3×3 腐蚀：对 3 行分别做 1×3 腐蚀，再按列与
    genvar gc;

    // ★ 膨胀也必须是 3×3（否则开运算后物体在垂直方向会短 1 块 = 8 像素，
    //   表现就是"框不住物体上下"）：
    //   先垂直膨胀（三行的 3×3 腐蚀结果相或），再横向 1×3 膨胀
    wire [BC-1:0] ero_r2h, ero_r1h, ero_r0h, dil_v, dil_all;
    generate
        for (gc = 0; gc < BC; gc = gc + 1) begin : EROH
            // 三行各自的横向 1×3 腐蚀（垂直方向在 ero_all 里与过）
            wire l2 = (gc == 0)    ? 1'b0 : mrow_p2[gc-1];
            wire r2 = (gc == BC-1) ? 1'b0 : mrow_p2[gc+1];
            wire l1 = (gc == 0)    ? 1'b0 : mrow_p1[gc-1];
            wire r1 = (gc == BC-1) ? 1'b0 : mrow_p1[gc+1];
            wire l0 = (gc == 0)    ? 1'b0 : mrow_cur[gc-1];
            wire r0 = (gc == BC-1) ? 1'b0 : mrow_cur[gc+1];
            assign ero_r2h[gc] = l2 & mrow_p2[gc] & r2;
            assign ero_r1h[gc] = l1 & mrow_p1[gc] & r1;
            assign ero_r0h[gc] = l0 & mrow_cur[gc] & r0;
        end
    endgenerate
    assign ero_all = ero_r2h & ero_r1h & ero_r0h;      // 3×3 腐蚀（完整）
    assign dil_v   = ero_r2h | ero_r1h | ero_r0h;      // ★ 垂直 3 行膨胀
    generate
        for (gc = 0; gc < BC; gc = gc + 1) begin : DIL
            wire l = (gc == 0)    ? 1'b0 : dil_v[gc-1];
            wire r = (gc == BC-1) ? 1'b0 : dil_v[gc+1];
            assign dil_all[gc] = l | dil_v[gc] | r;    // ★ 横向 1×3 膨胀
        end
    endgenerate

    // 行缓存推进 + 收集开运算结果
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            mrow_cur <= {BC{1'b0}}; mrow_p1 <= {BC{1'b0}}; mrow_p2 <= {BC{1'b0}};
            ero_p2 <= {BC{1'b0}}; ero_p1 <= {BC{1'b0}};
        end else if (!I_en || s_fstart || s_vend) begin
            mrow_cur <= {BC{1'b0}}; mrow_p1 <= {BC{1'b0}}; mrow_p2 <= {BC{1'b0}};
            ero_p2 <= {BC{1'b0}}; ero_p1 <= {BC{1'b0}};
        end else begin
            // 运动块写入当前行
            if (s_mot_v) mrow_cur[s_bc_c] <= 1'b1;
            // 块行切换：整行移位 + 记录开运算结果
            if (s_rowend) begin
                mrow_p2  <= mrow_p1;
                mrow_p1  <= mrow_cur;
                mrow_cur <= {BC{1'b0}};
                s_obits  <= OPEN_EN ? dil_all : mrow_p2;   // 关掉开运算就用原始运动位
                s_oby    <= (s_brow_c == 8'd0) ? 8'd0 : (s_brow_c - 8'd1);
                // 注意：参与运算的 mrow_p2 是"往回第 2 行"，所以 by = brow-1-... 见下方说明
            end
        end
    end

    // ★ 逐位送出：**每 2 拍送一个块**
    //   为什么必须留一拍间隙：投票在第 1 拍用当前目标表做判断，更新在第 2 拍才落地。
    //   若两个运动块背靠背投票，第 2 个块读到的还是旧表（第 1 个块的 bbox 扩展
    //   没写进去）-> 投票不准、同一物体被重复建成多个目标。
    //   一行有 240×16 = 3840 拍，240 个块每块 2 拍只需 480 拍，余量极大。
    reg s_obgap;      // 0 = 本拍送块；1 = 间隔拍
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_obusy <= 1'b0; s_obx <= 8'd0; s_obgap <= 1'b0;
        end else if (!I_en) begin
            s_obusy <= 1'b0; s_obx <= 8'd0; s_obgap <= 1'b0;
        end else begin
            if (s_rowend && !s_obusy) begin
                s_obusy <= 1'b1; s_obx <= 8'd0; s_obgap <= 1'b0;
            end else if (s_obusy) begin
                if (s_obgap == 1'b0) begin
                    s_obgap <= 1'b1;                 // 下一拍是间隔拍
                end else begin
                    s_obgap <= 1'b0;                 // 下一拍送下一个块
                    if (s_obx >= (BC-1)) s_obusy <= 1'b0;
                    else                 s_obx <= s_obx + 8'd1;
                end
            end
        end
    end

    // 送出的这一位就是一个"运动块"（坐标 s_obx, s_oby）；间隔拍不送
    wire s_open_mot = s_obusy && !s_obgap && s_obits[s_obx];

    // ★ 运动二值图（调试用）：当前拍的 4 个像素各自查自己所在块的位
    //   像素列 = s_lb*4 + k，块列 = 像素列 >> 3
    //   mrow_p2 对应的是"往回第 2 行"，所以画面会比实际下移 2 个块（16 行），
    //   调试观察无所谓。
    wire [8:0] px0 = {s_lb, 2'b00};                 // 本拍第 0 个像素的列号
    wire [7:0] b0 = px0[8:3];
    wire [7:0] b1 = (px0 + 9'd1) >> 3;
    wire [7:0] b2 = (px0 + 9'd2) >> 3;
    wire [7:0] b3 = (px0 + 9'd3) >> 3;

    // ======================================================================
    //  ④c 运动位图 RAM（调试显示用）：240×135 位，32 位打包 -> 8 字/行 × 135 行
    //      在开运算逐位送出时顺便写进去；mixer 按显示坐标查询
    // ======================================================================
    integer di;

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
        end else if (!I_en || s_fstart || s_vend) begin
        end else begin
            if (s_obusy && !s_obgap) begin
                // 这一拍送出的是块列 s_obx 的运动位
                if (s_obx[4:0] == 5'd31) begin
                end
            end
        end
    end

    // ★ 真正的写入
    always @(posedge I_pix_clk) begin
    end

    // ★ 查询端口（同步读，1 拍延迟）
    
    // ======================================================================
    //  ⑤ 目标表（16 个）
    // ======================================================================
        // (c) 运动位图 RAM（供 SW4 二值图显示）
    // ★ 调试位图已删除（SW4 二值图功能已取消）
    always @(posedge I_pix_clk) O_q_word <= 32'd0;

reg        t_val [0:NT-1];
    reg [9:0]  t_x1  [0:NT-1], t_x2 [0:NT-1];
    reg [9:0]  t_y1  [0:NT-1], t_y2 [0:NT-1];
    reg [11:0] t_hit [0:NT-1];
    reg [5:0]  t_cnt;                  // 已用目标数（NT 最大 63）
    // 投票延迟一拍后使用（与目标表更新对齐）
    reg [7:0]  s_vbx, s_vby;
    reg        s_vote_v;
    reg [NT-1:0] s_vote;               // 每个目标投"新目标"票
    reg [NT-1:0] s_inslot;             // 部分票时：落在邻域内的目标（要扩展的）

    integer vi;
    // ---- 组合投票：每个运动块对 16 个目标投票 ----
    //   ★ 邻域判定必须写成"全加法"形式：
    //     (bx + M >= x1) && (bx <= x2 + M)
    //   若写成 (bx < x1 - M)，当 x1 < M 时 x1-M 变成负数，
    //   而无符号比较会把它当成极大的正数 -> 判定恒真 -> 所有目标都投"新目标"票。
    always @(*) begin
        for (vi = 0; vi < NT; vi = vi + 1) begin
            s_vote[vi] = ~t_val[vi] ||
                         ~(({3'd0, s_vbx} + MARGIN >= {1'd0, t_x1[vi]}) &&
                           ({3'd0, s_vbx}          <= {1'd0, t_x2[vi]} + MARGIN)) ||
                         ~(({3'd0, s_vby} + MARGIN >= {1'd0, t_y1[vi]}) &&
                           ({3'd0, s_vby}          <= {1'd0, t_y2[vi]} + MARGIN));
        end
    end
    wire s_allvote = &s_vote;          // 全票 -> 新目标

    integer ui;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            t_cnt <= 6'd0;
            s_vbx <= 8'd0; s_vby <= 8'd0; s_vote_v <= 1'b0;
            for (ui = 0; ui < NT; ui = ui + 1) begin
                t_val[ui] <= 1'b0;
                t_x1[ui] <= 10'd0; t_x2[ui] <= 10'd0;
                t_y1[ui] <= 10'd0; t_y2[ui] <= 10'd0;
                t_hit[ui] <= 12'd0;
            end
        end else if (!I_en) begin
            t_cnt <= 6'd0;
            for (ui = 0; ui < NT; ui = ui + 1) t_val[ui] <= 1'b0;
        end else begin
            // ---- 帧首清空目标表（每帧重新检测）----
            if (s_fstart) begin
                t_cnt <= 6'd0;
                for (ui = 0; ui < NT; ui = ui + 1) begin
                    t_val[ui] <= 1'b0;
                    t_hit[ui] <= 12'd0;
                end
            end
            // ---- 第 1 拍：锁存"开运算后送出的运动块"坐标（投票用）----
            s_vote_v <= s_open_mot;
            if (s_open_mot) begin
                // ★ 块坐标补偿必须限幅到 0：无符号减法在 s_obx<XCOMP / s_oby<YCOMP 时会下溢
                //   s_oby=1, YCOMP=2 -> 255 → 目标行坐标变成 255 → 屏上出现巨大框
                s_vbx <= (s_obx >= XCOMP[7:0]) ? (s_obx - XCOMP[7:0]) : 8'd0;
                s_vby <= (s_oby >= YCOMP[7:0]) ? (s_oby - YCOMP[7:0]) : 8'd0;
            end
            // ★ 补偿量是实测标定的：
            //   XCOMP=1：开运算逐位送出时列号比真实块列大 1，减 1 后框与物体完全对齐
            //   YCOMP=2：行号同样偏大 2（开运算取的是往回第 2 行），减 2 后上下边界一致
            // ---- 第 2 拍：按投票结果更新目标表 ----
            if (s_vote_v) begin
                if (s_allvote) begin
                    // 全票：新目标（占用第一个空槽）
                    // ★ 用显式循环"按 t_cnt 命中写入"，避免变量索引的非阻塞赋值
                    //   在部分工具里不生效（实测 iverilog 下 t_val[t_cnt]<=1 没写进去，
                    //   导致同一物体的后续块又被判成新目标）
                    if (t_cnt < NT[5:0]) begin
                        for (ui = 0; ui < NT; ui = ui + 1) begin
                            if (ui[4:0] == t_cnt) begin
                                t_val[ui] <= 1'b1;
                                t_x1[ui]  <= {2'd0, s_vbx};
                                t_x2[ui]  <= {2'd0, s_vbx};
                                t_y1[ui]  <= {2'd0, s_vby};
                                t_y2[ui]  <= {2'd0, s_vby};
                                t_hit[ui] <= 12'd1;
                            end
                        end
                        t_cnt <= t_cnt + 6'd1;
                    end
                end else begin
                    // 部分票：对"未投票（=落在邻域内）"的目标扩展 bbox
                    for (ui = 0; ui < NT; ui = ui + 1) begin
                        if (t_val[ui] && !s_vote[ui]) begin
                            if ({2'd0, s_vbx} < t_x1[ui]) t_x1[ui] <= {2'd0, s_vbx};
                            if ({2'd0, s_vbx} > t_x2[ui]) t_x2[ui] <= {2'd0, s_vbx};
                            if ({2'd0, s_vby} < t_y1[ui]) t_y1[ui] <= {2'd0, s_vby};
                            if ({2'd0, s_vby} > t_y2[ui]) t_y2[ui] <= {2'd0, s_vby};
                            t_hit[ui] <= t_hit[ui] + 12'd1;
                        end
                    end
                end
            end
        end
    end

    // ======================================================================
    //  ⑥ 帧末：把有效目标（hits >= MIN_HITS）整体输出到 O_b*
    // ======================================================================
    // ★ 候选目标（块坐标）与已输出框（像素坐标）的重叠判定
    //   候选的像素坐标 = 块坐标 << 3
    // ★ 用变量下标读数组时，必须放在 always @(*) 里；
    //   写成 wire 连续赋值在部分工具/综合器下会得到错误值 -> 屏上出现巨大框/错位框
    reg [11:0] c_x1, c_x2, c_y1, c_y2;
    always @(*) begin
        c_x1 = {4'd0, t_x1[s_scan][7:0]} << 3;
        c_x2 = ({4'd0, t_x2[s_scan][7:0]} << 3) + 12'd7;
        c_y1 = {4'd0, t_y1[s_scan][7:0]} << 3;
        c_y2 = ({4'd0, t_y2[s_scan][7:0]} << 3) + 12'd7;
    end

    reg [5:0]  s_sel;                  // 已选出几个
    reg [5:0]  s_scan;                 // 扫描目标表用（要能到 NT）
    reg [3:0]  s_fo;                   // ★ 输出了几个（2 位只能到 3，MAXO=8 会回绕）
    localparam [1:0] FO_IDLE=2'd0, FO_SCAN=2'd1, FO_DONE=2'd2;
    reg [3:0] s_hold [0:MAXO-1];       // ★ 每个输出框的保持计数（2 位装不下 HOLD_FRM=8）
    reg [1:0]  s_fo_st;
    integer qi, ok;

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_fo_st <= FO_IDLE; s_scan <= 6'd0; s_sel <= 6'd0; s_fo <= 4'd0;
            for (qi = 0; qi < MAXO; qi = qi + 1) s_hold[qi] <= 4'd0;
            O_bv <= {MAXO{1'b0}}; O_obj_cnt <= 5'd0;
            for (qi = 0; qi < MAXO; qi = qi + 1) begin
                O_bx1[((qi<<3)+(qi<<2)) +: 12] <= 12'd0; O_bx2[((qi<<3)+(qi<<2)) +: 12] <= 12'd0;
                O_by1[((qi<<3)+(qi<<2)) +: 12] <= 12'd0; O_by2[((qi<<3)+(qi<<2)) +: 12] <= 12'd0;
            end
        end else if (!I_en) begin
            s_fo_st <= FO_IDLE; O_bv <= {MAXO{1'b0}}; O_obj_cnt <= 5'd0;
        end else begin
            case (s_fo_st)
            FO_IDLE: begin
                if (s_vend) begin s_scan <= 6'd0; s_sel <= 6'd0; s_fo <= 4'd0;
                    // ★ 不清 O_bx1..O_by2：防闪烁保持(s_hold)还亮着时要继续用旧坐标，
                    //   清了会让"有效但坐标=0"的框画在左上角 -> 异常残框
                    s_fo_st <= FO_SCAN; end
                else begin
                    // 平时保持上一帧的结果（不在这里清零，否则显示期看不到框）
                end
            end
            FO_SCAN: begin
                if (s_scan >= NT[5:0]) begin s_fo_st <= FO_DONE; end
                else begin
                    // ★ 还要满足最小尺寸（宽、高都至少 MIN_SIZE 个块），滤掉极小的框
                    if (t_val[s_scan] && (t_hit[s_scan] >= MIN_HITS) &&
                        ((t_x2[s_scan] - t_x1[s_scan]) >= (MIN_SIZE-1)) &&
                        ((t_y2[s_scan] - t_y1[s_scan]) >= (MIN_SIZE-1))) begin
                        // ★ 去掉"重叠就扩展已有框"的分支（3~4 个物体会链式扩展成一个大框）
                        //   每个合格目标各占一个框，与 H 计数严格 1:1
                        if (s_fo < MAXO) begin
                            // ★ 用 case 写固定位段：变量动态 part-select 写 reg 向量会写错位
                            //   -> 屏上会出现巨大框 / 错位框
                            O_bv[s_fo] <= 1'b1;
                            case (s_fo)
                            4'd0: begin O_bx1[0*12 +: 12]<=c_x1; O_bx2[0*12 +: 12]<=c_x2; O_by1[0*12 +: 12]<=c_y1; O_by2[0*12 +: 12]<=c_y2; end
                            4'd1: begin O_bx1[1*12 +: 12]<=c_x1; O_bx2[1*12 +: 12]<=c_x2; O_by1[1*12 +: 12]<=c_y1; O_by2[1*12 +: 12]<=c_y2; end
                            4'd2: begin O_bx1[2*12 +: 12]<=c_x1; O_bx2[2*12 +: 12]<=c_x2; O_by1[2*12 +: 12]<=c_y1; O_by2[2*12 +: 12]<=c_y2; end
                            4'd3: begin O_bx1[3*12 +: 12]<=c_x1; O_bx2[3*12 +: 12]<=c_x2; O_by1[3*12 +: 12]<=c_y1; O_by2[3*12 +: 12]<=c_y2; end
                            4'd4: begin O_bx1[4*12 +: 12]<=c_x1; O_bx2[4*12 +: 12]<=c_x2; O_by1[4*12 +: 12]<=c_y1; O_by2[4*12 +: 12]<=c_y2; end
                            4'd5: begin O_bx1[5*12 +: 12]<=c_x1; O_bx2[5*12 +: 12]<=c_x2; O_by1[5*12 +: 12]<=c_y1; O_by2[5*12 +: 12]<=c_y2; end
                            4'd6: begin O_bx1[6*12 +: 12]<=c_x1; O_bx2[6*12 +: 12]<=c_x2; O_by1[6*12 +: 12]<=c_y1; O_by2[6*12 +: 12]<=c_y2; end
                            default: begin O_bx1[7*12 +: 12]<=c_x1; O_bx2[7*12 +: 12]<=c_x2; O_by1[7*12 +: 12]<=c_y1; O_by2[7*12 +: 12]<=c_y2; end
                            endcase
                            s_fo <= s_fo + 4'd1;
                        end
                    end
                    s_scan <= s_scan + 6'd1;
                end
            end
            FO_DONE: begin
                // ★ 防闪烁：本帧没选中的框不立刻熄灭，先减保持计数；
                //   连续 HOLD_FRM 帧都无效才真正熄灭。
                for (qi = 0; qi < MAXO; qi = qi + 1) begin
                    if (qi >= s_fo) begin
                        if (s_hold[qi] == 0) O_bv[qi] <= 1'b0;
                        else begin
                            s_hold[qi] <= s_hold[qi] - 4'd1;
                            // 坐标保持不动（继续显示上一帧的框）
                        end
                    end else begin
                        s_hold[qi] <= HOLD_FRM[3:0];      // 本帧有效 -> 重置保持计数
                    end
                end
                O_obj_cnt <= s_fo;
                s_fo_st <= FO_IDLE;
            end
            default: s_fo_st <= FO_IDLE;
            endcase
        end
    end

    // ======================================================================
    //  ⑦ 诊断
    // ======================================================================
    reg [11:0] s_mot_run;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n)               s_mot_run <= 12'd0;
        else if (!I_en || s_fstart) s_mot_run <= 12'd0;   // 一帧内累计（开运算后的运动块）
        else if (s_open_mot)        s_mot_run <= s_mot_run + 12'd1;
    end
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin O_frame_cnt <= 16'd0; O_mot_cnt <= 12'd0; end
        else if (!I_en) begin O_frame_cnt <= 16'd0; O_mot_cnt <= 12'd0; end
        else if (s_vend) begin
            O_frame_cnt <= O_frame_cnt + 16'd1;
            O_mot_cnt   <= s_mot_run;
        end
    end

endmodule












// ===== BEGIN inline roi_motion_onchip.v (由 sim/inline_roi_motion.py 生成，请勿手改) =====
`timescale 1ns / 1ps
// ============================================================================
//  roi_motion_onchip.v — 固定 ROI 的背景帧差分"有没有牌"检测（两路）
// ----------------------------------------------------------------------------
//  用途：牌永远摆在两个固定的矩形框里（主框 ≈ 13 张，右侧小框另有一路）。
//        用【背景帧差分】判断 ROI 里"有没有东西"，作为识别模块的启动触发。
//
//  ★ 为什么比 frame_diff_onchip 省得多：
//    · 只在两个固定 ROI 内做：不做全画面扫描、不做多目标、不需要运动位图 /
//      3x3 开运算 / 邻域投票 / 重叠合并 / 多框输出；
//    · 判据就是一个数：ROI 内"块平均与背景差超阈值"的块个数 >= THR -> 有牌；
//    · 噪点靠"个数阈值"滤掉（孤立一两块进不了阈值），所以开运算整块可以删。
//    · 两个 ROI 行范围相同 -> 行/拍计数与门控逻辑共用一份。
//
//  数据流（ISP 输出域，与 Y 严格同拍，4 像素/拍）：
//    8x8 块：一行 8 像素 = 2 拍；块列累加器把 ROI 的 8 行加起来（共 64 像素）
//      -> ROI 最后一行结束后逐块结算：平均 = 累加 >> 6（纯移位）与背景比
//      -> |差| > DIFF_THR 记 1 个运动块，按主框/小框分别累加
//      -> 帧末与 THR_A / THR_B 比 -> present；连续 HOLD 帧不达标才熄（防抖）
//
//  KEY4（I_bg_cap）：拨上 = 抓背景（把当前 ROI 块平均写进背景），
//                    拨下 = 冻结背景开始检测；没抓过背景前 O_bg_rdy=0、不报 present
//
//  ★ 零乘法：块平均是右移；地址/边界全是常数比较
//  ★ 存储用寄存器不用 ERAM：主框 124 + 小框 9 = 133 个块列，
//    133 x (16 位累加 + 8 位背景) ≈ 3.2K FF —— 器件 FF 只用 47%，
//    用 FF 换掉原来 frame_diff 的 17 块 ERAM 很划算，也免了 RAM 读延迟的麻烦。
// ============================================================================
module roi_motion_onchip #(
    parameter integer IMG_W    = 1280,
    parameter integer IMG_H    = 720,
    parameter [7:0]   DIFF_THR = 8'd40,      // 块平均绝对差阈值（空桌 vs 有牌）
    parameter integer THR_A    = 40,         // 主框：运动【列条】数 >= 它 -> 有牌（主框只有 124 条！）
    parameter integer THR_B    = 3,          // 小框阈值（小框只有 9 条！）
    parameter integer HOLD     = 8,          // 连续多少帧不达标才熄（防抖，约 0.13 秒）
    // ★ 平均的右移位数 = ceil(log2(8 x ROI 高))，让 avg = sum >> AVG_SH 落在 0..255
    //   真设计：ROI 高 112 像素 -> 一个竖条 = 8x112 = 896 像素 -> AVG_SH = 10（÷1024）
    //   注意这是"近似整除"（0.875 倍），全画面统一缩放，只需把 DIFF_THR 按比例调即可
    parameter integer AVG_SH   = 10,
    // 两个 ROI 的像素范围（★ 必须是 8 的倍数对齐，边界落在块边界上）
    parameter integer AX1      = 56,         // 主框左右（块列 7..130）
    parameter integer AX2      = 1047,
    parameter integer BX1      = 1096,       // 小框左右（块列 137..145）
    parameter integer BX2      = 1167,
    parameter integer CY1      = 240,        // 两框共用的行范围（块行 30..43）
    parameter integer CY2      = 351
) (
    input  wire        I_pix_clk,
    input  wire        I_rst_n,
    input  wire        I_pix_valid,
    input  wire        I_pix_last,
    input  wire        I_pix_user,
    input  wire [31:0] I_y_now,              // 4 像素 Y（与 valid/last/user 严格同拍）
    input  wire        I_en,                 // SW4 使能
    input  wire        I_bg_cap,             // KEY4：1 = 抓背景，0 = 冻结并检测

    output reg         O_a_present,          // 主框：有牌
    output reg         O_b_present,          // 小框：有东西
    output reg  [11:0] O_a_count,            // 主框本帧运动块数（诊断 / 现场标阈值）
    output reg  [11:0] O_b_count,            // 小框本帧运动块数
    output reg         O_bg_rdy              // 背景已抓
);
    // ------------------------------------------------------------------
    //  几何常量（全部参数推出，综合期就是常数，不占逻辑）
    //   ★ 注意：不要对 integer 参数做部分选择（会得到 x），所以先定宽再比较
    // ------------------------------------------------------------------
    localparam integer LN_MAX = IMG_H - 1;
    localparam integer BC_A   = (AX2 - AX1 + 1) / 8;   // 主框块列数 = 124
    localparam integer BC_B   = (BX2 - BX1 + 1) / 8;   // 小框块列数 = 9
    localparam integer NBLK   = BC_A + BC_B;           // 块列总数 = 133
    localparam [7:0]   CA_L   = AX1 / 8;               // 主框块列 7
    localparam [7:0]   CA_H   = AX2 / 8;               //          130
    localparam [7:0]   CB_L   = BX1 / 8;               // 小框块列 137
    localparam [7:0]   CB_H   = BX2 / 8;               //          145
    localparam [7:0]   CR_L   = CY1 / 8;               // 块行 30
    localparam [7:0]   CR_H   = CY2 / 8;               //      43
    localparam [7:0]   BBASE  = BC_A;                  // 小框在块列数组里的基址 = 124
    localparam [7:0]   NBLK8  = NBLK;                  // 133
    localparam [7:0]   NBLK_M = NBLK - 1;              // 132
    localparam [11:0]  THRA12 = THR_A;
    localparam [11:0]  THRB12 = THR_B;
    localparam [3:0]   HOLD4  = HOLD;

    // ------------------------------------------------------------------
    //  ① 行/拍计数（行末用 I_pix_last 归零拍计数）
    // ------------------------------------------------------------------
    reg  [9:0]  s_lb;
    reg  [10:0] s_line;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin s_lb <= 10'd0; s_line <= 11'd0; end
        else if (I_pix_valid) begin
            if (I_pix_user)      begin s_lb <= 10'd1; s_line <= 11'd0;          end
            else if (I_pix_last) begin s_lb <= 10'd0; s_line <= s_line + 11'd1; end
            else                      s_lb <= s_lb + 10'd1;
        end
    end
    wire        s_vend = I_pix_valid && I_pix_last && (s_line == LN_MAX[10:0]);
    // ★ 帧首那一拍要【组合性】当第 0 拍（寄存器要等时钟沿才清零，否则每帧漏一拍）
    wire [9:0]  s_lb_c = I_pix_user ? 10'd0 : s_lb;
    wire [10:0] s_ln_c = I_pix_user ? 11'd0 : s_line;
    wire [7:0]  s_bcol = s_lb_c[9:1];        // 块列
    wire [7:0]  s_brow = s_ln_c[10:3];       // 块行

    // ------------------------------------------------------------------
    //  ② 采样门控 + 块地址
    // ------------------------------------------------------------------
    wire s_row_ok = (s_brow >= CR_L) && (s_brow <= CR_H);
    wire s_in_a   = s_row_ok && (s_bcol >= CA_L) && (s_bcol <= CA_H);
    wire s_in_b   = s_row_ok && (s_bcol >= CB_L) && (s_bcol <= CB_H);
    wire s_in_any = s_in_a || s_in_b;
    wire [7:0] s_blkaddr = s_in_a ? (s_bcol - CA_L) : (BBASE + (s_bcol - CB_L));
    // ★ 位宽必须够：4 个像素 x 255 = 1020，要 10 位（写 9 位会被截断成 mod 512，
    //   实测亮场 200x4=800 被截成 288，平均就变成 72）
    wire [9:0] s_bsum    = I_y_now[7:0] + I_y_now[15:8] + I_y_now[23:16] + I_y_now[31:24];

    // ROI 最后一行的行末（★ 必须是块行的【最后一行】，否则一个块行会触发 8 次，
    // 每次结算都把只累了一半的 sum 读走又清零 —— 实测就是这个 bug 让平均只有 1/2~1/8）
    wire s_roi_end = I_pix_valid && I_pix_last && (s_brow == CR_H) && (s_ln_c[2:0] == 3'b111);

    // ------------------------------------------------------------------
    //  ③ 列条累加 / 背景 / 结算（同一个 always 块：数组只能有一个写口）
    //   ★ 一个"列条" = 8 像素宽 x ROI 高（真设计 8x112 = 896 像素）。
    //     之所以不按 8x8 块存：ROI 高 112 像素 = 14 个块行，按 (块行,块列) 存要
    //     14x133 = 1862 项、光地址译码就上千 LUT；按列条只存 133 项。
    //     代价：平均是"近似整除"（>>AVG_SH），全屏统一缩放，调 DIFF_THR 即可。
    //   ★ 累加与背景合成一个 26 位数组（[25:8]=累加 18 位，[7:0]=背景），
    //     这样地址译码只做一份（省约 350 LUT）。
    // ------------------------------------------------------------------
    reg  [25:0] s_mem [0:NBLK-1];
    reg         s_cst;                       // 结算中
    reg  [7:0]  s_cs;                        // 结算地址 0..NBLK-1
    reg  [11:0] s_ba, s_bb;                  // 本帧运动列条计数（主框 / 小框）

    // 结算用组合量
    wire [17:0] s_acc  = s_mem[s_cs][25:8];
    wire [7:0]  s_bgc  = s_mem[s_cs][7:0];
    wire [7:0]  s_avgq = s_acc[AVG_SH + 7 -: 8];
    wire [8:0]  s_diff = (s_avgq >= s_bgc) ? ({1'b0, s_avgq} - {1'b0, s_bgc})
                                           : ({1'b0, s_bgc} - {1'b0, s_avgq});
    wire        s_mot  = (s_diff > {1'b0, DIFF_THR});
    wire        s_is_b = (s_cs >= BBASE);

    integer k;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (k = 0; k < NBLK; k = k + 1) s_mem[k] <= 26'd0;
            s_cst <= 1'b0; s_cs <= 8'd0; s_ba <= 12'd0; s_bb <= 12'd0;
            O_a_count <= 12'd0; O_b_count <= 12'd0;
        end else begin
            // ---- 采样：本拍 4 像素的 Y 加进对应列条（背景字段原样保留）----
            if (I_pix_valid && s_in_any && !s_cst) begin
                for (k = 0; k < NBLK; k = k + 1)
                    if (k == s_blkaddr)
                        s_mem[k] <= { s_mem[k][25:8] + {8'd0, s_bsum}, s_mem[k][7:0] };
            end
            // ---- ROI 末行结束：清计数并从 0 号列条开始结算 ----
            if (s_roi_end) begin
                s_cst <= 1'b1; s_cs <= 8'd0; s_ba <= 12'd0; s_bb <= 12'd0;
            end
            // ---- 结算：一列条一拍（抓背景 / 比阈值 / 清累加）----
            else if (s_cst) begin
                // 抓背景：把当前平均写进背景字段（KEY4 拨上期间持续刷新）
                // 否则：背景冻结，|平均-背景| > DIFF_THR 记一个运动列条
                s_mem[s_cs] <= { 18'd0, I_bg_cap ? s_avgq : s_bgc };
                if (!I_bg_cap && O_bg_rdy && s_mot) begin
                    if (s_is_b) s_bb <= s_bb + 12'd1;
                    else        s_ba <= s_ba + 12'd1;
                end
                if (s_cs == NBLK_M) s_cst <= 1'b0;
                else                s_cs  <= s_cs + 8'd1;
            end
            // ---- 帧末锁存计数 ----
            if (s_vend) begin
                O_a_count <= s_ba;
                O_b_count <= s_bb;
            end
        end
    end

    // ------------------------------------------------------------------
    //  ④ 阈值判决 + 防抖
    // ------------------------------------------------------------------
    reg [3:0] s_ha, s_hb;                    // 连续不达标帧数
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            O_a_present <= 1'b0; O_b_present <= 1'b0;
            s_ha <= 4'd0; s_hb <= 4'd0; O_bg_rdy <= 1'b0;
        end else begin
            if (I_bg_cap) O_bg_rdy <= 1'b1;      // 抓过背景就置位
            if (s_vend) begin
                if (s_ba >= THRA12) begin O_a_present <= 1'b1; s_ha <= 4'd0; end
                else if (s_ha >= HOLD4) O_a_present <= 1'b0;
                else s_ha <= s_ha + 4'd1;

                if (s_bb >= THRB12) begin O_b_present <= 1'b1; s_hb <= 4'd0; end
                else if (s_hb >= HOLD4) O_b_present <= 1'b0;
                else s_hb <= s_hb + 4'd1;
            end
        end
    end
endmodule

// ===== END inline roi_motion_onchip.v ===== ===== =====

// ===== BEGIN inline tile_recog_onchip.v (由 sim/inline_tile_recog.py 生成，请勿手改) =====
// ============================================================================
//  tile_recog_onchip.v — 麻将牌识别【阶段 1 + 整排切分】
// ----------------------------------------------------------------------------
//  位置：ISP 输出域（与 frame_diff_onchip 同域、同像素流），**不碰 HDMI 时序域**
//
//  数据源：
//    · 像素流：I_y_now / I_col4（4 像素/拍，ISP 的 s1 级，与 valid/last/user 严格同拍）
//    · 合并框：frame_diff_onchip 的上一帧结果 ★【像素】坐标，不是块坐标 ★
//              （frame_diff 里 c_x1 = 块<<3、c_x2 = (块<<3)+7）
//
//  处理链：
//    ① 坐标重建：行末用 I_pix_last 归零拍计数（免疫"每行拍数"差异）
//    ② ★整排切分★：紧贴成一排的牌会被 frame_diff 的邻域投票（MARGIN=4 块 = 32 像素）
//       合并成【一个】大框，而牌间没有缝、投影谷值法也无效。解法是靠几何反推：
//       牌立着时 宽:高 ≈ 0.708 ≈ 12/17，框高就是一张牌的高，故一张牌的宽 ≈ 0.708*h，
//       张数 N ≈ w / (0.708*h)，再按 N 等宽切分（余数按 Bresenham 分给前几份）。
//       ★ 全程无乘法器、无除法器：定标全是移位加法；求 N 用重复减法；
//         求每份宽用一次 8 拍移位除法（w ≤ 240 块，8 位够）。
//    ③ ROI 门控（按切出来的子框，块粒度）
//    ④ 前景判定 + 4 桶色系分类（色系码由 ISP 用比较器算好，零乘法）
//    ⑤ 每槽累加 + 帧末锁存（像素数 / 颜色签名 / 空白率 / 宽高比）
//    ⑥ 按 I_sel 输出【选中槽】的特征与其矩形（供 OSD 显示 + 画黄框）
//       —— 只传一张牌的几十根线，而不是 16 槽的 800 多根
//
//  ★DSP★ 本模块一个算术 * 都没有（TD 会把 * 映射成 DSP，本器件上限 40、基线已用 33）。
// ============================================================================
module tile_recog_onchip #(
    parameter integer IMG_W     = 1920,
    parameter integer IMG_H     = 1080,
    parameter integer NBOX      = 14,       // 输入框数（★ 固定 14 个牌位：主框 13 张 + 右侧小框 1）
    parameter integer NSLOT     = 16,       // 切分后槽数（一排最多 9 张，留余量）
    parameter integer MAX_TILE   = 10,      // 一个框最多切成几张（防误切）
    parameter [7:0]   Y_DARK    = 8'd150,   // Y 低于它算"暗图案"
    parameter integer BLANK_PCT = 2,        // 空白率阈值（百分比）
    parameter integer MIN_BLK   = 3,        // 子框每边至少几个块（=24 像素）
    parameter integer GXOFF     = 0,        // 块坐标微调（与 frame_diff 相位对齐）
    parameter integer GYOFF     = 0,
    // ---- 阶段 2：细粒度数子图（筒数 / 条数）----
    //   4x4 像素 = 1 格；一张立牌 ≈ 17x23 格，一个筒子 ≈ 5 格
    parameter integer BMP_W     = 20,       // 局部位图宽（格）：20 格 = 80 像素
    parameter integer BMP_H     = 28,       // 局部位图高（格）：28 格 = 112 像素（= 牌位高）
    parameter [4:0]   CELL_THR  = 5'd4,     // 一格 16 像素里 >= 4 个前景像素 -> 算占据
    parameter integer DOT_MIN   = 2,        // 连通块 >= 这么多格才计入（滤孤立噪点）
    parameter [6:0]   BAR_MIN_V = 7'd4,     // 列投影峰的最小高度（像素）
    parameter integer BAR_MIN_W = 2,        // 峰宽下限（像素）
    parameter integer BAR_MIN_G = 2         // 两峰之间的间隙下限（像素）
) (
    input  wire        I_pix_clk,
    input  wire        I_rst_n,
    input  wire        I_en,                // 与 frame_diff 同一个使能（SW4）

    input  wire        I_pix_valid,
    input  wire        I_pix_last,          // ★ 行末标记（ISP 用它递增行计数器）
    input  wire        I_pix_user,          // ★ 帧首标记
    input  wire [31:0] I_y_now,
    input  wire [7:0]  I_col4,              // 4 像素色系码（2 位/像素）：0红1绿2蓝3无彩

    input  wire [NBOX-1:0]    I_bv,         // frame_diff 的合并框（★ 像素坐标）
    input  wire [12*NBOX-1:0] I_bx1,
    input  wire [12*NBOX-1:0] I_bx2,
    input  wire [12*NBOX-1:0] I_by1,
    input  wire [12*NBOX-1:0] I_by2,

    input  wire [3:0]  I_sel,               // 显示哪个槽（来自 mixer 的轮播，准静态）

    output reg         O_sel_v,
    output reg  [21:0] O_sel_pixcnt,
    output reg  [31:0] O_sel_colsig,        // [31:24]红 [23:16]绿 [15:8]蓝 [7:0]无彩
    output reg         O_sel_blank,
    output reg         O_sel_ar_ok,
    output reg  [11:0] O_sel_x1, O_sel_x2,  // ★ 像素坐标（与 frame_diff 同约定）
    output reg  [11:0] O_sel_y1, O_sel_y2,

    output reg  [3:0]  O_hit_slot,
    output reg  [4:0]  O_nslot,             // 诊断：本帧切出几个子框
    output reg         O_done,

    // ---- 阶段 2：细粒度计数结果（都属于【选中槽】那一张牌）----
    output reg  [4:0]  O_sel_dots,          // 连通块数（筒候选），饱和 15
    output reg  [4:0]  O_sel_bars,          // 列投影峰数（条候选），饱和 15
    output reg  [8:0]  O_sel_cells,         // 位图里被占据的格数（诊断）
    output reg  [8:0]  O_sel_maxc           // 最大连通块的格数（诊断）
);
    localparam integer LN_MAX   = IMG_H - 1;
    localparam integer CNT_W    = 22;
    localparam integer NUM_W    = 28;
    localparam [3:0]   SLOT_LAST= NSLOT - 1;
    localparam integer DENS_SH  = 3;        // 4 像素/拍 * BLANK_PCT(2) = 8 = 1<<3
    localparam integer BIDX_W   = 8;        // 块坐标位宽

    // ======================================================================
    //  ① 坐标重建
    //     ★ user 那一拍必须【组合性】当成本帧第 0 拍（寄存器要等时钟沿才清零，
    //       而这一拍就是要算的像素；否则每帧漏一拍）
    //     ★ 行末用 I_pix_last 归零拍计数：不要求"每行拍数恰好 = IMG_W/4"
    // ======================================================================
    reg  [9:0]  s_lb;
    reg  [10:0] s_line;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin s_lb <= 10'd0; s_line <= 11'd0; end
        else if (I_pix_valid) begin
            if (I_pix_user)      begin s_lb <= 10'd1; s_line <= 11'd0;          end
            else if (I_pix_last) begin s_lb <= 10'd0; s_line <= s_line + 11'd1; end
            else                      s_lb <= s_lb + 10'd1;
        end
    end
    wire s_vend = I_pix_valid && I_pix_last && (s_line == LN_MAX[10:0]);

    wire [9:0]  s_lb_c   = I_pix_user ? 10'd0 : s_lb;
    wire [10:0] s_line_c = I_pix_user ? 11'd0 : s_line;
    wire [7:0]  s_bc     = s_lb_c[9:1];             // 块列 0..239
    wire [7:0]  s_brow   = s_line_c[10:3];          // 块行 0..134
    wire [8:0]  s_bc_g   = {1'b0, s_bc}   + GXOFF[8:0];
    wire [8:0]  s_brow_g = {1'b0, s_brow} + GYOFF[8:0];

    // ======================================================================
    //  输入合并框：像素 -> 块（取 [10:3]；不能写成 I_bx1[..+:12][10:3]）
    // ======================================================================
    wire [BIDX_W*NBOX-1:0] b_x1, b_x2, b_y1, b_y2;
    genvar gi;
    generate
        for (gi = 0; gi < NBOX; gi = gi + 1) begin : BXC
            wire [11:0] t1 = I_bx1[12*gi +: 12];
            wire [11:0] t2 = I_bx2[12*gi +: 12];
            wire [11:0] t3 = I_by1[12*gi +: 12];
            wire [11:0] t4 = I_by2[12*gi +: 12];
            assign b_x1[BIDX_W*gi +: BIDX_W] = t1[10:3];
            assign b_x2[BIDX_W*gi +: BIDX_W] = t2[10:3];
            assign b_y1[BIDX_W*gi +: BIDX_W] = t3[10:3];
            assign b_y2[BIDX_W*gi +: BIDX_W] = t4[10:3];
        end
    endgenerate

    // ======================================================================
    //  ② 整排切分 FSM（帧末跑；产出 NSLOT 个子框，块坐标）
    // ======================================================================
    reg  [NSLOT-1:0]        s_sv;
    reg  [BIDX_W*NSLOT-1:0] s_sx1, s_sx2, s_sy1, s_sy2;

    localparam [2:0] SP_IDLE=3'd0, SP_CALC=3'd1, SP_NDIV=3'd2,
                     SP_DIVI=3'd3, SP_LAYO=3'd4, SP_DONE=3'd5;

    reg [2:0]  s_sp;
    reg [3:0]  s_j;                 // 当前输入框
    reg [4:0]  s_oc;                // 已输出子框数
    reg [7:0]  s_cx1, s_cx2, s_cy1, s_cy2;
    reg [7:0]  s_cur;               // 当前子框起点（块列）
    reg [7:0]  s_wblk;
    reg [15:0] s_rem;               // w*17 + h*6
    reg [13:0] s_h12;               // ((h<<3)+(h<<2))
    reg [4:0]  s_n;                 // N：切几张
    reg [3:0]  s_di;                // 除法迭代计数 0..8
    reg [7:0]  s_dq, s_drem;        // 商/余（wblk / N）
    reg [3:0]  s_lo;                // 分配子框计数
    reg [4:0]  s_err;               // 余数分配累加器
    reg [7:0]  s_len;               // 本份长度

    wire [7:0] c_x1 = b_x1[BIDX_W*s_j +: BIDX_W];
    wire [7:0] c_x2 = b_x2[BIDX_W*s_j +: BIDX_W];
    wire [7:0] c_y1 = b_y1[BIDX_W*s_j +: BIDX_W];
    wire [7:0] c_y2 = b_y2[BIDX_W*s_j +: BIDX_W];

    // ★ 本份长度/终点/进位数都用【组合】算，不能在同一个 always 里用刚赋值的
    //   非阻塞值（那样会晚一拍，边界全错）
    wire [4:0]  lay_err = ((s_err + {1'b0, s_drem}) >= {1'b0, s_n})
                        ? (s_err + {1'b0, s_drem} - {1'b0, s_n})
                        : (s_err + {1'b0, s_drem});
    wire [7:0]  lay_len = ((s_err + {1'b0, s_drem}) >= {1'b0, s_n}) ? (s_dq + 8'd1) : s_dq;
    wire [7:0]  lay_x2  = s_cur + lay_len - 8'd1;
    wire [7:0]  lay_nx  = s_cur + lay_len;

    integer wi;                      // ★ 切分 FSM 自己的循环变量
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_sp <= SP_IDLE; s_oc <= 5'd0; s_j <= 4'd0; s_n <= 5'd0;
            s_sv <= {NSLOT{1'b0}}; O_nslot <= 5'd0;
        end else begin
            case (s_sp)
                SP_IDLE: if (s_fst == F_DONE) begin
                    // ★ 切分必须排在"帧末锁存特征"【之后】：
                    //   本帧的像素是用【上一帧】切出的门控累加的，所以帧末要先把
                    //   特征和那个门控一起锁存（显示才对得上），然后再用本帧的合并框
                    //   重切门控、留给下一帧用。顺序颠倒会让"显示矩形"和"特征"差一帧。
                    s_sv <= {NSLOT{1'b0}};          // 每帧重新切
                    s_oc <= 5'd0;  s_j <= 4'd0;
                    s_sp <= SP_CALC;
                end
                // ---- 取下一个有效输入框，算 w*17 + h*6 与 ((h<<3)+(h<<2)) ----
                SP_CALC: begin
                    if (s_j >= NBOX[3:0]) s_sp <= SP_DONE;
                    else if (!I_bv[s_j])  s_j  <= s_j + 4'd1;
                    else begin
                        s_cx1 <= c_x1; s_cx2 <= c_x2; s_cy1 <= c_y1; s_cy2 <= c_y2;
                        s_wblk<= (c_x2 >= c_x1) ? (c_x2 - c_x1 + 8'd1) : 8'd0;
                        // w*17 = 16w + w ;  ((h<<3)+(h<<2)) = 8h + 4h ;  h*6 = 4h + 2h
                        s_rem <= ({4'd0, (c_x2 - c_x1 + 8'd1)} << 4) + {4'd0, (c_x2 - c_x1 + 8'd1)}
                               + ({4'd0, (c_y2 - c_y1 + 8'd1)} << 2) + ({4'd0, (c_y2 - c_y1 + 8'd1)} << 1);
                        s_h12 <= ({4'd0, (c_y2 - c_y1 + 8'd1)} << 3) + ({4'd0, (c_y2 - c_y1 + 8'd1)} << 2);
                        s_n   <= 5'd0;
                        s_sp  <= SP_NDIV;
                    end
                end
                // ---- 求张数 N：反复减 ((h<<3)+(h<<2))（等价于 (w*17 + h*6) / (((h<<3)+(h<<2))) 的四舍五入）----
                SP_NDIV: begin
                    if ((s_n < MAX_TILE[4:0]) && (s_rem >= {2'd0, s_h12})) begin
                        s_rem <= s_rem - {2'd0, s_h12};
                        s_n   <= s_n + 5'd1;
                    end else begin
                        if (s_n == 5'd0) s_n <= 5'd1;   // 至少一张
                        s_di  <= 4'd8;                  // 除法从最高位开始
                        s_dq  <= 8'd0;
                        s_drem<= 8'd0;
                        s_sp  <= SP_DIVI;
                    end
                end
                // ---- 每份宽 = wblk / N（8 拍移位减法除法，N 已确定）----
                SP_DIVI: begin
                    if (s_di == 4'd0) begin
                        s_lo  <= 4'd0;
                        s_err <= 5'd0;
                        s_cur <= s_cx1;
                        s_sp  <= SP_LAYO;
                    end else begin
                        // rem = (rem<<1) | wblk[di-1]
                        if ({(s_drem[6:0]), s_wblk[s_di - 4'd1]} >= {3'd0, s_n}) begin
                            s_drem <= {(s_drem[6:0]), s_wblk[s_di - 4'd1]} - {3'd0, s_n};
                            s_dq[s_di - 4'd1] <= 1'b1;
                        end else begin
                            s_drem <= {(s_drem[6:0]), s_wblk[s_di - 4'd1]};
                            s_dq[s_di - 4'd1] <= 1'b0;
                        end
                        s_di <= s_di - 4'd1;
                    end
                end
                // ---- 按 N 份分配：长度 = 商(+1)，余数用累加器摊给前几份 ----
                SP_LAYO: begin
                    if (s_lo >= s_n[3:0]) begin
                        s_sp <= SP_CALC;                // 本框切完，取下一个框
                        s_j  <= s_j + 4'd1;
                    end else if (s_oc > SLOT_LAST) begin
                        s_sp <= SP_DONE;                // 槽用完了
                    end else begin
                        // 写入第 s_oc 个子框（显式 for + 索引比较；用组合算出的长度）
                        for (wi = 0; wi < NSLOT; wi = wi + 1) begin
                            if (wi == s_oc[3:0]) begin
                                s_sv [wi]        <= 1'b1;
                                s_sx1[BIDX_W*wi +: BIDX_W] <= s_cur;
                                s_sx2[BIDX_W*wi +: BIDX_W] <= lay_x2;
                                s_sy1[BIDX_W*wi +: BIDX_W] <= s_cy1;
                                s_sy2[BIDX_W*wi +: BIDX_W] <= s_cy2;
                            end
                        end
                        s_cur <= lay_nx;
                        s_err <= lay_err;
                        s_oc  <= s_oc + 5'd1;
                        s_lo  <= s_lo + 4'd1;
                    end
                end
                SP_DONE: begin
                    O_nslot <= s_oc;
                    s_sp    <= SP_IDLE;
                end
                default: s_sp <= SP_IDLE;
            endcase
        end
    end

    // ======================================================================
    //  ③ ROI 门控（★ C1 瘦身：只对【选中槽】做，不再 16 槽并行比一遍）
    //     OSD 本来就只显示选中槽，阶段 2 也是这么做的 —— 省掉 16 组比较与
    //     16 槽累加器（16 x (5x22 + 20) = 2080 FF）+ 帧末 16 选 1 多路器。
    //     换槽后下一帧就出新数（轮播 1.5 秒，够）。
    // ======================================================================
    localparam [3:0] SLOTMAX = NSLOT - 1;                       // 合法槽号上限
    wire [3:0]  sel_i  = (I_sel <= SLOTMAX) ? I_sel : 4'd0;     // 越界保护
    //   ★ 不能写 NSLOT[3:0]：对 integer 参数做部分选择会得到 x（实测过，整个模块
    //     会一直按 0 号槽工作）
    wire [7:0]  sb_x1  = s_sx1[BIDX_W*sel_i +: BIDX_W];         // 选中槽子框（块坐标）
    wire [7:0]  sb_x2  = s_sx2[BIDX_W*sel_i +: BIDX_W];
    wire [7:0]  sb_y1  = s_sy1[BIDX_W*sel_i +: BIDX_W];
    wire [7:0]  sb_y2  = s_sy2[BIDX_W*sel_i +: BIDX_W];

    wire s_hit_sel = s_sv[sel_i]
                  && (s_bc_g   >= {1'b0, sb_x1})
                  && (s_bc_g   <= {1'b0, sb_x2})
                  && (s_brow_g >= {1'b0, sb_y1})
                  && (s_brow_g <= {1'b0, sb_y2});

    // 诊断输出：选中槽命中就给槽号，没命中给 15（原先是"最低命中槽"，
    // 单槽累加后这个量只对选中槽有意义）
    always @(*) O_hit_slot = s_hit_sel ? sel_i : 4'd15;

    // ======================================================================
    //  ④ 前景判定 + 4 桶色系分类
    // ======================================================================
    wire [3:0] s_fg, s_bin_r, s_bin_g, s_bin_b, s_bin_k;
    genvar li;
    generate
        for (li = 0; li < 4; li = li + 1) begin : LANE
            wire [7:0] y  = I_y_now[8*li +: 8];
            wire [1:0] cc = I_col4 [2*li +: 2];
            wire s_colored = (cc != 2'd3);
            assign s_fg[li]    = (y < Y_DARK) || s_colored;
            assign s_bin_k[li] = s_fg[li] && (cc == 2'd3);
            assign s_bin_r[li] = s_fg[li] && (cc == 2'd0);
            assign s_bin_g[li] = s_fg[li] && (cc == 2'd1);
            assign s_bin_b[li] = s_fg[li] && (cc == 2'd2);
        end
    endgenerate

    wire [2:0] s_n_fg = {2'd0,s_fg[0]}    + {2'd0,s_fg[1]}    + {2'd0,s_fg[2]}    + {2'd0,s_fg[3]};
    wire [2:0] s_n_r  = {2'd0,s_bin_r[0]} + {2'd0,s_bin_r[1]} + {2'd0,s_bin_r[2]} + {2'd0,s_bin_r[3]};
    wire [2:0] s_n_g  = {2'd0,s_bin_g[0]} + {2'd0,s_bin_g[1]} + {2'd0,s_bin_g[2]} + {2'd0,s_bin_g[3]};
    wire [2:0] s_n_b  = {2'd0,s_bin_b[0]} + {2'd0,s_bin_b[1]} + {2'd0,s_bin_b[2]} + {2'd0,s_bin_b[3]};
    wire [2:0] s_n_k  = {2'd0,s_bin_k[0]} + {2'd0,s_bin_k[1]} + {2'd0,s_bin_k[2]} + {2'd0,s_bin_k[3]};

    // ======================================================================
    //  ⑤ 累加（★ C1 瘦身：只有【选中槽】一份，帧末处理完后 s_clr 清零）
    // ======================================================================
    reg [CNT_W-1:0] s_cnt;                  // 前景像素数
    reg [CNT_W-1:0] s_cr_c, s_cg_c, s_cb_c, s_ck_c;   // 4 桶色系计数
    reg [19:0]      s_beat;                 // 命中拍数（算空白率用）
    reg s_clr;

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en || s_clr) begin
            s_cnt  <= {CNT_W{1'b0}};
            s_cr_c <= {CNT_W{1'b0}};
            s_cg_c <= {CNT_W{1'b0}};
            s_cb_c <= {CNT_W{1'b0}};
            s_ck_c <= {CNT_W{1'b0}};
            s_beat <= 20'd0;
        end else if (I_pix_valid && s_hit_sel) begin
            s_cnt  <= s_cnt  + {{(CNT_W-3){1'b0}}, s_n_fg};
            s_cr_c <= s_cr_c + {{(CNT_W-3){1'b0}}, s_n_r};
            s_cg_c <= s_cg_c + {{(CNT_W-3){1'b0}}, s_n_g};
            s_cb_c <= s_cb_c + {{(CNT_W-3){1'b0}}, s_n_b};
            s_ck_c <= s_ck_c + {{(CNT_W-3){1'b0}}, s_n_k};
            s_beat <= s_beat + 20'd1;
        end
    end

    // ======================================================================
    //  ⑥ 帧末锁存选中槽特征（★ C1 瘦身：不再逐槽扫 16 遍，直接算选中槽）
    //     顺序不变：锁存排在切分 FSM【之前】（那时子框还是上一帧的，与门控同源）
    // ======================================================================
    localparam [2:0] F_IDLE=3'd0, F_RUN=3'd1, F_DONE=3'd2, F_WAITG=3'd3;
    reg [2:0]  s_fst;
    reg [4:0]  s_step;
    reg [10:0] s_sw, s_sh;
    reg [4:0]  s_msb;
    reg [3:0]  s_shf;
    reg [NUM_W-1:0] s_num, s_den;
    reg        s_blank_f, s_ar_ok_f, s_vld_f;
    reg [7:0]  s_sig_r, s_sig_g, s_sig_b, s_sig_k;

    function [4:0] msb22;
        input [21:0] v;
        integer k;
        begin
            msb22 = 5'd0;
            for (k = 0; k < 22; k = k + 1)
                if (v[k]) msb22 = k[4:0];
        end
    endfunction

    function [7:0] norm8;
        input [21:0] v;
        input [3:0]  sh;
        reg [21:0] t;
        begin
            t = v >> sh;
            norm8 = (t > 22'd255) ? 8'd255 : t[7:0];
        end
    endfunction

    wire [10:0] q_wpx = {s_sw[7:0], 3'b000};
    wire [10:0] q_hpx = {s_sh[7:0], 3'b000};
    wire [15:0] q_w17 = ({5'd0, q_wpx} << 4) + {5'd0, q_wpx};
    wire [15:0] q_h10 = ({5'd0, q_hpx} << 3) + ({5'd0, q_hpx} << 1);
    wire [15:0] q_h14 = ({5'd0, q_hpx} << 4) - ({5'd0, q_hpx} << 1);

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_fst <= F_IDLE; s_step <= 5'd0; s_clr <= 1'b0;
            O_done <= 1'b0;
            O_sel_v <= 1'b0; O_sel_pixcnt <= 22'd0; O_sel_colsig <= 32'd0;
            O_sel_blank <= 1'b0; O_sel_ar_ok <= 1'b0;
            O_sel_x1 <= 12'd0; O_sel_x2 <= 12'd0; O_sel_y1 <= 12'd0; O_sel_y2 <= 12'd0;
        end else begin
            s_clr  <= 1'b0;
            O_done <= 1'b0;
            case (s_fst)
                F_IDLE: if (s_vend) begin               // 帧末立即锁存（切分排在它之后）
                    s_fst <= F_RUN; s_step <= 5'd0;
                end
                F_RUN: begin
                    case (s_step)
                        5'd0: begin
                            s_sw  <= (sb_x2 >= sb_x1) ? (sb_x2 - sb_x1 + 8'd1) : 8'd0;
                            s_sh  <= (sb_y2 >= sb_y1) ? (sb_y2 - sb_y1 + 8'd1) : 8'd0;
                            s_msb <= msb22(s_cnt);
                            O_sel_x1 <= {sb_x1, 3'b000};
                            O_sel_x2 <= {sb_x2, 3'b000} + 12'd7;
                            O_sel_y1 <= {sb_y1, 3'b000};
                            O_sel_y2 <= {sb_y2, 3'b000} + 12'd7;
                            s_step<= 5'd1;
                        end
                        5'd1: begin     // cnt*100 = cnt*64 + cnt*32 + cnt*4（移位加法）
                            s_num <= ({6'd0, s_cnt} << 6)
                                   + ({6'd0, s_cnt} << 5)
                                   + ({6'd0, s_cnt} << 2);
                            s_step<= 5'd2;
                        end
                        5'd2: begin     // 分母 = 命中拍数 * 4 像素/拍 * BLANK_PCT
                            s_den <= {8'd0, s_beat} << DENS_SH;
                            s_step<= 5'd3;
                        end
                        5'd3: begin
                            s_shf     <= (s_msb > 5'd7) ? (s_msb[3:0] - 4'd7) : 4'd0;
                            s_blank_f <= (s_num < s_den);
                            s_ar_ok_f <= (q_w17 >= q_h10) && (q_w17 <= q_h14);
                            s_vld_f   <= s_sv[sel_i] && (s_sw >= MIN_BLK[10:0])
                                                     && (s_sh >= MIN_BLK[10:0])
                                                     && (s_cnt != {CNT_W{1'b0}});
                            s_step    <= 5'd4;
                        end
                        5'd4: begin
                            s_sig_r <= norm8(s_cr_c, s_shf);
                            s_sig_g <= norm8(s_cg_c, s_shf);
                            s_sig_b <= norm8(s_cb_c, s_shf);
                            s_sig_k <= norm8(s_ck_c, s_shf);
                            s_step  <= 5'd5;
                        end
                        default: begin
                            O_sel_v      <= s_vld_f;
                            O_sel_pixcnt <= s_cnt;
                            O_sel_colsig <= {s_sig_r, s_sig_g, s_sig_b, s_sig_k};
                            O_sel_blank  <= s_blank_f;
                            O_sel_ar_ok  <= s_ar_ok_f;
                            s_fst        <= F_WAITG;    // ★ 交棒给阶段 2 计数
                            s_step       <= 5'd0;
                        end
                    endcase
                end
                F_WAITG: if (s_gdone) s_fst <= F_DONE;   // 等 ⑧ 的计数 FSM 跑完
                F_DONE: begin
                    O_done <= 1'b1;
                    s_clr  <= 1'b1;         // 下一拍清累加器（已进消隐期）
                    s_fst  <= F_IDLE;
                    s_step <= 5'd0;
                end
                default: s_fst <= F_IDLE;
            endcase
        end
    end

    // ======================================================================
    //  ⑦ 选中槽的输出已由帧末 FSM 直接写入（见 ⑥），这里不再需要 16 选 1 多路器
    // ======================================================================

    // ======================================================================
    //  ⑧ 阶段 2：细粒度数子图（4x4 格位图数"筒" + 逐像素列投影数"条"）
    // ----------------------------------------------------------------------
    //  ★ 为什么要在识别模块内部自己建位图：这里拿到的是【像素流】（4 像素/拍），
    //    不受 frame_diff 的 8x8 块粒度限制。4x4 像素 = 1 格 -> 一张立牌 ≈ 17x23 格，
    //    一个筒子 ≈ 5 格，数 1..9 够用；圆点是 2D 排布，必须用 2D 位图。
    //  ★ 竹条之间有缝：逐像素列投影的"峰数"就是列方向并排的竖直结构数（更省更稳）。
    //  ★ 只数【选中槽】那一张牌：位图和投影各只有一份（480 位 + 80x7 位），
    //    换槽后下一帧就出新数（轮播 1.5 秒，够）。
    //  ★ 全程没有一个算术 *：坐标全是移位/比较，计数全是累加（规则 4）。
    //  ★ C1 瘦身：帧末先算选中槽特征（6 拍，不再是 16 槽 x 6 拍），再跑这里。
    //  ★ 帧末顺序：计数排在 ⑥"帧末锁存特征"【之后】（那时切分几何才稳定），
    //    一次跑 6 + 1 + (480+24) + 16 + 40 + 41 + 1 ≈ 610 拍；
    //    720p60 的垂直消隐有 30 行 x 320 拍 = 9600 拍，余量充足。
    //  ★ 连通域用【即时重标号】而不是"合并表 + 收尾并查集"：
    //    发现一个格同时挨着两个不同标签时，当场把"输的"标签在上/本两行里
    //    就地改成"赢的"标签，尺寸并过去、输的清零。这样任何时刻标签都是唯一的，
    //    收尾只要数"尺寸 >= DOT_MIN 的标签个数"，不需要第二遍。
    // ======================================================================
    localparam integer PROJ_W = BMP_W * 4;          // ★ 逐像素列投影（C2 未采用：1 格 = 4 列）
    localparam [4:0]   CCL_W  = BMP_W;              // 位图宽（格）
    localparam [4:0]   CCL_H  = BMP_H;              // 位图高（格）
    localparam [4:0]   CCL_WM = CCL_W - 5'd1;       // 最右一列
    localparam [4:0]   CCL_HM = CCL_H - 5'd1;       // 最下一行
    localparam [3:0]   LAB_MAX= 4'd15;              // 标签上限（0 = 空）
    localparam [4:0]   LABMAX5= LAB_MAX;            // 同上，5 位版本（给计数器比较用）
    localparam [5:0]   NLAB   = 6'd16;              // 标签数组长度 = 0..15
    localparam [6:0]   PROJ_W7= PROJ_W;             // 投影列数（7 位）
    localparam [6:0]   PROJ_WM= PROJ_W - 1;
    localparam [6:0]   BARW7  = BAR_MIN_W;
    localparam [6:0]   BARG7  = BAR_MIN_G;
    localparam [8:0]   DOTMIN9= DOT_MIN;

    // ---------- 选中槽的子框（块坐标）-> 4x4 格坐标系 ----------
    //   （sel_i / sb_x1..sb_y2 已经在 ③ 声明，供 ROI 门控共用）
    // 绝对格坐标：一拍 = 一格列（4 像素）；4 行 = 一格行
    wire [10:0] s_ccell = {s_bc_g, 1'b0}   + {10'd0, s_lb_c[0]};
    wire [10:0] s_rcell = {s_brow_g, 1'b0} + {10'd0, s_line_c[2]};
    wire [10:0] s_kx1   = {3'd0, sb_x1, 1'b0};
    wire [10:0] s_ky1   = {3'd0, sb_y1, 1'b0};
    wire [10:0] s_kx2   = {3'd0, sb_x2, 1'b0} + 11'd1;   // 块 x2 覆盖到 2*x2+1 格列
    wire [10:0] s_ky2   = {3'd0, sb_y2, 1'b0} + 11'd1;

    wire s_in_box = s_sv[sel_i]
                 && (s_ccell >= s_kx1) && (s_ccell <= s_kx2)
                 && (s_rcell >= s_ky1) && (s_rcell <= s_ky2);
    wire [10:0] s_rel_cc = s_ccell - s_kx1;
    wire [10:0] s_rel_rr = s_rcell - s_ky1;
    // 位图窗口内的门控（子框比位图大时只数左/上的一块，其余裁掉）
    wire s_in_bmp = s_in_box && (s_rel_cc < CCL_W) && (s_rel_rr < CCL_H);

    // ---------- ① 格行累加器：本格行里每格的前景像素数（4 行 x 4 像素 = 最多 16）----------
    reg  [4:0] s_rcnt [0:BMP_W-1];
    reg  [4:0] s_rowidx;                    // 正在累加/刚结束的格行号（子框内）
    reg        s_row_used;                  // 本格行有没有采到样本
    reg        s_rowok;                     // 行号在位图范围内
    reg        s_commit_d;                  // 格行结束（行末且 line[1:0]==3）后一拍

    wire s_line_end4 = I_pix_valid && I_pix_last && (s_line_c[1:0] == 2'b11);

    integer ci;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (ci = 0; ci < BMP_W; ci = ci + 1) s_rcnt[ci] <= 5'd0;
        end else if (I_pix_valid) begin
            for (ci = 0; ci < BMP_W; ci = ci + 1) begin
                if (s_commit_d)
                    // ★ 本拍同时是"上一格行的收尾"和"新一行第 0 拍"：不能简单清零，
                    //   要写成"这一拍的新贡献"（否则子框从第 0 格列开始时丢一拍）
                    s_rcnt[ci] <= (s_in_bmp && (ci == s_rel_cc)) ? {2'd0, s_n_fg} : 5'd0;
                else if (s_in_bmp && (ci == s_rel_cc))
                    s_rcnt[ci] <= s_rcnt[ci] + {2'd0, s_n_fg};
            end
        end
    end

    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_commit_d <= 1'b0; s_row_used <= 1'b0; s_rowok <= 1'b0; s_rowidx <= 5'd0;
        end else begin
            s_commit_d <= s_line_end4;
            if (s_commit_d)                       s_row_used <= 1'b0;
            else if (I_pix_valid && s_in_bmp)     s_row_used <= 1'b1;
            if (I_pix_valid && s_in_box && !s_commit_d) begin
                s_rowidx <= s_rel_rr[4:0];
                s_rowok  <= (s_rel_rr < CCL_H);
            end
        end
    end

    // ---------- ② 位图（4x4 格）：格行结束时把本行阈值化后写进去 ----------
    reg  [BMP_W-1:0] s_bmp_row [0:BMP_H-1];
    reg              s_bmpclr;
    wire [BMP_W-1:0] s_occrow;

    genvar gw;
    generate
        for (gw = 0; gw < BMP_W; gw = gw + 1) begin : OCCROW
            assign s_occrow[gw] = (s_rcnt[gw] >= CELL_THR);
        end
    endgenerate

    integer bi;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (bi = 0; bi < BMP_H; bi = bi + 1) s_bmp_row[bi] <= {BMP_W{1'b0}};
        end else if (s_bmpclr) begin
            for (bi = 0; bi < BMP_H; bi = bi + 1) s_bmp_row[bi] <= {BMP_W{1'b0}};
        end else if (s_commit_d && s_row_used && s_rowok) begin
            for (bi = 0; bi < BMP_H; bi = bi + 1)
                if (bi == s_rowidx) s_bmp_row[bi] <= s_occrow;
        end
    end

    // ---------- ③ 逐像素列投影（索：竹条之间的缝在列方向形成峰谷）----------
    //   ★ 逐像素列（用户指定的粒度）：一格列 = 4 个像素列 = 4 个计数器。
    //     计数上限 = 4 像素/列/行 x BMP_H 行(4 行/格) <= 4*96 = 384 -> 用 9 位会翻倍；
    //     实际一行只贡献 1 个像素/列，故上限 = BMP_H*4 = 96 -> 7 位够。
    reg  [6:0] s_proj [0:PROJ_W-1];
    reg        s_projclr;

    integer pg;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            for (pg = 0; pg < PROJ_W; pg = pg + 1) s_proj[pg] <= 7'd0;
        end else if (s_projclr) begin
            for (pg = 0; pg < PROJ_W; pg = pg + 1) s_proj[pg] <= 7'd0;
        end else if (I_pix_valid && s_in_bmp) begin
            // 一拍 = 4 个连续像素列；格列 s_rel_cc 对应 4*cc .. 4*cc+3
            for (pg = 0; pg < BMP_W; pg = pg + 1)
                if (pg == s_rel_cc) begin
                    if (s_fg[0]) s_proj[4*pg    ] <= s_proj[4*pg    ] + 7'd1;
                    if (s_fg[1]) s_proj[4*pg + 1] <= s_proj[4*pg + 1] + 7'd1;
                    if (s_fg[2]) s_proj[4*pg + 2] <= s_proj[4*pg + 2] + 7'd1;
                    if (s_fg[3]) s_proj[4*pg + 3] <= s_proj[4*pg + 3] + 7'd1;
                end
        end
    end

    // ---------- ④ 帧末计数 FSM（清位图 -> 连通域 -> 统计 -> 列投影峰）----------
    localparam [3:0] G_IDLE =4'd0, G_CLR  =4'd1, G_SCAN =4'd2, G_ROWEND=4'd3,
                     G_STAT =4'd4, G_PMAX =4'd5, G_PCNT =4'd6, G_LAST  =4'd7;

    reg  [3:0]  s_gst;
    reg  [4:0]  s_cc, s_cr;                 // 连通域扫描的列 / 行
    reg  [4:0]  s_nlab;                     // 已分配标签数
    reg  [4:0]  s_gl;                       // 统计遍历
    reg  [7:0]  s_pcol;                     // 投影扫描列
    reg  [6:0]  s_pmax, s_thr, s_run, s_gap;    // 投影值 7 位
    reg  [4:0]  s_bars;
    reg         s_inhi, s_cand;
    reg  [8:0]  s_dots, s_cells, s_maxc;
    reg  [8:0]  s_sz  [0:15];               // 每个标签的格数（合并时就地累加）
    reg  [3:0]  s_clab [0:BMP_W-1];         // 本行标签（0 = 空）
    reg  [3:0]  s_plab [0:BMP_W-1];         // 上一行标签
    reg  [BMP_W-1:0] s_pocc;                // 上一行占据位
    reg         s_gdone;

    wire [BMP_W-1:0] g_clab_nz;
    generate
        for (gw = 0; gw < BMP_W; gw = gw + 1) begin : CLABNZ
            assign g_clab_nz[gw] = (s_clab[gw] != 4'd0);
        end
    endgenerate

    // 8 邻域：上一行的上左/上/上右 + 本行的左
    wire [4:0] cc_p1 = (s_cc == CCL_WM) ? CCL_WM : (s_cc + 5'd1);
    wire [4:0] cc_m1 = (s_cc == 5'd0)   ? 5'd0   : (s_cc - 5'd1);
    wire       g_occ = s_bmp_row[s_cr][s_cc];
    wire [3:0] g_ul  = ((s_cc != 5'd0)   && s_pocc[cc_m1]) ? s_plab[cc_m1] : 4'd0;
    wire [3:0] g_uc  = (s_pocc[s_cc])                      ? s_plab[s_cc]  : 4'd0;
    wire [3:0] g_ur  = ((s_cc != CCL_WM) && s_pocc[cc_p1]) ? s_plab[cc_p1] : 4'd0;
    wire [3:0] g_lf  = (s_cc != 5'd0) ? s_clab[cc_m1] : 4'd0;

    function [3:0] min_nz4;                 // 取非零候选里的最小标签（全 0 时返回 15）
        input [3:0] a, b, c, d;
        reg   [3:0] m;
        begin
            m = 4'd15;
            if ((a != 4'd0) && (a < m)) m = a;
            if ((b != 4'd0) && (b < m)) m = b;
            if ((c != 4'd0) && (c < m)) m = c;
            if ((d != 4'd0) && (d < m)) m = d;
            min_nz4 = m;
        end
    endfunction

    wire [3:0] g_surv = min_nz4(g_ul, g_uc, g_ur, g_lf);
    wire       g_any  = ((g_ul | g_uc | g_ur | g_lf) != 4'd0);
    // 需要合并的候选：★ 只有"本格被占据"时才谈合并 —— 空格子两侧的邻居
    // 在 8 邻域判定里会同时出现（左上/上右），不 gate 就会把两根竹条并成一根
    wire g_rl_ul = g_occ && (g_ul != 4'd0) && (g_ul != g_surv);
    wire g_rl_uc = g_occ && (g_uc != 4'd0) && (g_uc != g_surv) && (g_uc != g_ul);
    wire g_rl_ur = g_occ && (g_ur != 4'd0) && (g_ur != g_surv) && (g_ur != g_ul) && (g_uc != g_ur);
    wire g_rl_lf = g_occ && (g_lf != 4'd0) && (g_lf != g_surv) && (g_lf != g_ul) && (g_lf != g_uc) && (g_lf != g_ur);

    wire [8:0] g_szadd = (g_rl_ul ? s_sz[g_ul] : 9'd0) + (g_rl_uc ? s_sz[g_uc] : 9'd0)
                       + (g_rl_ur ? s_sz[g_ur] : 9'd0) + (g_rl_lf ? s_sz[g_lf] : 9'd0);
    wire [3:0] g_newlab = !g_occ ? 4'd0
                        : (!g_any ? ((s_nlab < LAB_MAX) ? (s_nlab[3:0] + 4'd1) : LAB_MAX)
                                  : g_surv);
    // 投影单元号夹紧（多跑的那一列不是真列，用 0 号列顶替，值由外层选 0）
    wire [6:0] g_pidx = (s_pcol[6:0] <= PROJ_WM) ? s_pcol[6:0] : 7'd0;
    wire [6:0] g_pval = (s_pcol[6:0] <= PROJ_WM) ? s_proj[g_pidx] : 7'd0;

    integer si2;
    always @(posedge I_pix_clk or negedge I_rst_n) begin
        if (!I_rst_n || !I_en) begin
            s_gst <= G_IDLE; s_cc <= 5'd0; s_cr <= 5'd0; s_nlab <= 5'd0; s_gl <= 5'd0;
            s_pcol <= 8'd0; s_pmax <= 7'd0; s_thr <= 7'd0; s_run <= 7'd0; s_gap <= 7'd0;
            s_bars <= 5'd0; s_inhi <= 1'b0; s_cand <= 1'b0;
            s_dots <= 9'd0; s_cells <= 9'd0; s_maxc <= 9'd0;
            s_pocc <= {BMP_W{1'b0}}; s_gdone <= 1'b0;
            s_bmpclr <= 1'b0; s_projclr <= 1'b0;
            O_sel_dots <= 5'd0; O_sel_bars <= 5'd0; O_sel_cells <= 9'd0; O_sel_maxc <= 9'd0;
            for (si2 = 0; si2 < NLAB; si2 = si2 + 1) s_sz[si2] <= 9'd0;
            for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                s_clab[si2] <= 4'd0; s_plab[si2] <= 4'd0;
            end
        end else begin
            s_bmpclr  <= 1'b0;
            s_projclr <= 1'b0;
            case (s_gst)
                // ---- 等帧末特征锁存完（F_WAITG），开始清场 ----
                G_IDLE: begin
                    // ★ s_gdone 是"跑完"脉冲：只保持一拍，否则下一帧 F_WAITG 立刻放行，
                    //   计数 FSM 就再也不会启动了
                    if (s_gdone) s_gdone <= 1'b0;
                    if ((s_fst == F_WAITG) && !s_gdone) begin
                    s_cc <= 5'd0; s_cr <= 5'd0; s_gl <= 5'd0; s_nlab <= 5'd0;
                    s_cells <= 9'd0; s_dots <= 9'd0; s_maxc <= 9'd0;
                    s_bars <= 5'd0; s_inhi <= 1'b0; s_run <= 7'd0; s_gap <= 7'd0; s_cand <= 1'b0;
                    s_pcol <= 8'd0; s_pmax <= 7'd0; s_thr <= 7'd0;
                    s_pocc <= {BMP_W{1'b0}};
                    for (si2 = 0; si2 < NLAB; si2 = si2 + 1) s_sz[si2] <= 9'd0;
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                        s_clab[si2] <= 4'd0; s_plab[si2] <= 4'd0;
                    end
                    s_gst <= G_CLR;
                end
                end
                G_CLR: s_gst <= G_SCAN;                     // 清位图/投影（上面已经打了一拍脉冲）
                // ---- 8 邻域连通域标记（一格一拍）----
                G_SCAN: begin
                    // ① 合并：把输掉的标签在上/本两行里就地改成存活标签
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                        if (g_rl_ul && (s_plab[si2] == g_ul)) s_plab[si2] <= g_surv;
                        if (g_rl_uc && (s_plab[si2] == g_uc)) s_plab[si2] <= g_surv;
                        if (g_rl_ur && (s_plab[si2] == g_ur)) s_plab[si2] <= g_surv;
                        if (g_rl_lf && (s_clab[si2] == g_lf)) s_clab[si2] <= g_surv;
                    end
                    // ② 本格标签（写在合并之后，同拍时以后者为准）
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1)
                        if (si2 == s_cc) s_clab[si2] <= g_newlab;
                    // ③ 尺寸：本格计入它的最终标签（新块 = g_newlab，否则 = 存活标签）
                    //    ★ 新块时 g_surv 是 min_nz4 的"全 0 缺省值 15"，不能拿它当标签用
                    for (si2 = 0; si2 < NLAB; si2 = si2 + 1) begin
                        if (g_occ && (si2 == g_newlab))
                            s_sz[si2] <= s_sz[si2] + g_szadd + 9'd1;
                        if (g_rl_ul && (si2 == g_ul)) s_sz[si2] <= 9'd0;
                        if (g_rl_uc && (si2 == g_uc)) s_sz[si2] <= 9'd0;
                        if (g_rl_ur && (si2 == g_ur)) s_sz[si2] <= 9'd0;
                        if (g_rl_lf && (si2 == g_lf)) s_sz[si2] <= 9'd0;
                    end
                    // ④ 新标签 / 占据格数
                    if (g_occ && !g_any && (s_nlab < LAB_MAX)) s_nlab <= s_nlab + 5'd1;
                    if (g_occ) s_cells <= s_cells + 9'd1;
                    if (s_cc == CCL_WM) s_gst <= G_ROWEND;
                    else                s_cc  <= s_cc + 5'd1;
                end
                // ---- 行末：本行标签 -> 上一行 ----
                G_ROWEND: begin
                    s_pocc <= g_clab_nz;
                    for (si2 = 0; si2 < BMP_W; si2 = si2 + 1) begin
                        s_plab[si2] <= s_clab[si2];
                        s_clab[si2] <= 4'd0;
                    end
                    s_cc <= 5'd0;
                    if (s_cr == CCL_HM) begin s_gl <= 5'd0; s_gst <= G_STAT; end
                    else begin s_cr <= s_cr + 5'd1; s_gst <= G_SCAN; end
                end
                // ---- 统计：连通块数（>= DOT_MIN 格）与最大块 ----
                G_STAT: begin
                    if (s_sz[s_gl] >= DOTMIN9) s_dots <= s_dots + 9'd1;
                    if (s_sz[s_gl] > s_maxc)   s_maxc <= s_sz[s_gl];
                    if (s_gl == LABMAX5) begin
                        s_pcol <= 8'd0; s_pmax <= 7'd0; s_gst <= G_PMAX;
                    end else s_gl <= s_gl + 5'd1;
                end
                // ---- 列投影：先找最大峰 ----
                G_PMAX: begin
                    if (s_proj[s_pcol[6:0]] > s_pmax) s_pmax <= s_proj[s_pcol[6:0]];
                    if (s_pcol[6:0] == PROJ_WM) begin
                        // 门限 = max(固定底噪, 最大峰的一半)
                        s_thr  <= (s_pmax > (BAR_MIN_V << 1)) ? {1'b0, s_pmax[6:1]} : BAR_MIN_V;
                        s_pcol <= 8'd0;
                        s_bars <= 5'd0; s_inhi <= 1'b0; s_run <= 7'd0; s_gap <= 7'd0; s_cand <= 1'b0;
                        s_gst  <= G_PCNT;
                    end else s_pcol <= s_pcol + 8'd1;
                end
                // ---- 列投影：数峰（峰宽 >= BAR_MIN_W，峰距 < BAR_MIN_G 的两个峰算一个）----
                G_PCNT: begin
                    // 多跑一个单元（s_pcol > PROJ_WM 时当成 0），保证最后一根竹条也能收尾
                    if (g_pval >= s_thr) begin
                        if (!s_inhi) begin
                            s_inhi <= 1'b1;
                            s_run  <= 7'd1;
                            s_cand <= (s_bars == 5'd0) || (s_gap >= BARG7);
                        end else s_run <= s_run + 7'd1;
                        s_gap <= 7'd0;
                    end else begin
                        if (s_inhi) begin
                            s_inhi <= 1'b0;
                            if (s_cand && (s_run >= BARW7) && (s_bars != 5'd15))
                                s_bars <= s_bars + 5'd1;
                        end else if (s_gap != 7'd127) s_gap <= s_gap + 7'd1;
                    end
                    if (s_pcol[6:0] == PROJ_W7) s_gst <= G_LAST;
                    else                   s_pcol <= s_pcol + 8'd1;
                end
                // ---- 锁存结果（属于本帧选中槽那一张牌），并清掉位图/投影给下一帧 ----
                G_LAST: begin
                    O_sel_dots  <= (s_dots > 9'd15) ? 5'd15 : s_dots[4:0];
                    O_sel_bars  <= s_bars;
                    O_sel_cells <= s_cells;
                    O_sel_maxc  <= s_maxc;
                    // ★ 清场必须放在【扫描之后】：位图里装的就是本帧的结果，
                    //   清早了（放 G_IDLE）CCL 只能看到一张空图
                    s_bmpclr    <= 1'b1;
                    s_projclr   <= 1'b1;
                    s_gdone     <= 1'b1;
                    s_gst       <= G_IDLE;
                end
                default: s_gst <= G_IDLE;
            endcase
        end
    end
endmodule
// ===== END inline tile_recog_onchip.v =====

// ===================== 内联：安路 logo ROM（ERAM 版，RGB565，半分辨率）=====================
module anlogic_logo_rom (
    input  wire        I_clk,
    input  wire [13:0] I_addr,
    output reg  [15:0] O_pixel
);
    reg [15:0] rom [0:4095];
    integer ir;
    initial begin
        for (ir = 0; ir < 4096; ir = ir + 1) rom[ir] = 16'h0000;
        rom[12'd525] = 16'hb5d8;
        rom[12'd526] = 16'hfeba;
        rom[12'd527] = 16'hf2ab;
        rom[12'd528] = 16'hf26a;
        rom[12'd529] = 16'hf26a;
        rom[12'd530] = 16'hf26a;
        rom[12'd531] = 16'hf26a;
        rom[12'd532] = 16'hf26a;
        rom[12'd533] = 16'hf26a;
        rom[12'd534] = 16'hf26a;
        rom[12'd535] = 16'hf26a;
        rom[12'd536] = 16'hf26a;
        rom[12'd537] = 16'hf26a;
        rom[12'd538] = 16'hf26a;
        rom[12'd539] = 16'hf26a;
        rom[12'd540] = 16'hf26a;
        rom[12'd541] = 16'hf26a;
        rom[12'd542] = 16'hf26a;
        rom[12'd543] = 16'hf26a;
        rom[12'd544] = 16'hf26a;
        rom[12'd545] = 16'hf26a;
        rom[12'd546] = 16'hf26a;
        rom[12'd547] = 16'hf26a;
        rom[12'd548] = 16'hf26a;
        rom[12'd549] = 16'hf26a;
        rom[12'd550] = 16'hfd55;
        rom[12'd588] = 16'hce7a;
        rom[12'd589] = 16'h428d;
        rom[12'd590] = 16'h9d16;
        rom[12'd591] = 16'hf24a;
        rom[12'd592] = 16'hf187;
        rom[12'd593] = 16'hf187;
        rom[12'd594] = 16'hf187;
        rom[12'd595] = 16'hf187;
        rom[12'd596] = 16'hf187;
        rom[12'd597] = 16'hf187;
        rom[12'd598] = 16'hf187;
        rom[12'd599] = 16'hf187;
        rom[12'd600] = 16'hf187;
        rom[12'd601] = 16'hf187;
        rom[12'd602] = 16'hf187;
        rom[12'd603] = 16'hf187;
        rom[12'd604] = 16'hf187;
        rom[12'd605] = 16'hf187;
        rom[12'd606] = 16'hf187;
        rom[12'd607] = 16'hf187;
        rom[12'd608] = 16'hf187;
        rom[12'd609] = 16'hf187;
        rom[12'd610] = 16'hf187;
        rom[12'd611] = 16'hf187;
        rom[12'd612] = 16'hf187;
        rom[12'd613] = 16'hf24a;
        rom[12'd652] = 16'h6bb1;
        rom[12'd653] = 16'h1969;
        rom[12'd654] = 16'h52ee;
        rom[12'd715] = 16'hce7a;
        rom[12'd716] = 16'h3a4c;
        rom[12'd717] = 16'h1969;
        rom[12'd718] = 16'h21aa;
        rom[12'd719] = 16'h94d4;
        rom[12'd720] = 16'hf34d;
        rom[12'd721] = 16'hf1c8;
        rom[12'd722] = 16'hf1c8;
        rom[12'd723] = 16'hf1c8;
        rom[12'd724] = 16'hf1c8;
        rom[12'd725] = 16'hf1c8;
        rom[12'd726] = 16'hf1c8;
        rom[12'd727] = 16'hf1c8;
        rom[12'd728] = 16'hf1c8;
        rom[12'd729] = 16'hf1c8;
        rom[12'd730] = 16'hf1c8;
        rom[12'd731] = 16'hf1c8;
        rom[12'd732] = 16'hf1c8;
        rom[12'd733] = 16'hf1c8;
        rom[12'd734] = 16'hf1c8;
        rom[12'd735] = 16'hf1c8;
        rom[12'd736] = 16'hf1c8;
        rom[12'd737] = 16'hf1c8;
        rom[12'd738] = 16'hf1c8;
        rom[12'd739] = 16'hf1c8;
        rom[12'd740] = 16'hf30c;
        rom[12'd779] = 16'h6b90;
        rom[12'd780] = 16'h1969;
        rom[12'd781] = 16'h1969;
        rom[12'd782] = 16'h1969;
        rom[12'd783] = 16'h4ace;
        rom[12'd784] = 16'hf431;
        rom[12'd785] = 16'hf1a7;
        rom[12'd786] = 16'hf1a7;
        rom[12'd787] = 16'hf1a7;
        rom[12'd788] = 16'hf1a7;
        rom[12'd789] = 16'hf1a7;
        rom[12'd790] = 16'hf1a7;
        rom[12'd791] = 16'hf1a7;
        rom[12'd792] = 16'hf1a7;
        rom[12'd793] = 16'hf1a7;
        rom[12'd794] = 16'hf1a7;
        rom[12'd795] = 16'hf1a7;
        rom[12'd796] = 16'hf1a7;
        rom[12'd797] = 16'hf1a7;
        rom[12'd798] = 16'hf1a7;
        rom[12'd799] = 16'hf1a7;
        rom[12'd800] = 16'hf1a7;
        rom[12'd801] = 16'hf1a7;
        rom[12'd802] = 16'hf1a7;
        rom[12'd803] = 16'hf1a7;
        rom[12'd804] = 16'hf30c;
        rom[12'd842] = 16'hce7a;
        rom[12'd843] = 16'h3a2c;
        rom[12'd844] = 16'h1969;
        rom[12'd845] = 16'h1969;
        rom[12'd846] = 16'h1969;
        rom[12'd847] = 16'h218a;
        rom[12'd848] = 16'h8473;
        rom[12'd906] = 16'h6390;
        rom[12'd907] = 16'h1969;
        rom[12'd908] = 16'h1969;
        rom[12'd909] = 16'h1969;
        rom[12'd910] = 16'h1969;
        rom[12'd911] = 16'h1969;
        rom[12'd912] = 16'h42ad;
        rom[12'd913] = 16'hfd14;
        rom[12'd914] = 16'hf1a7;
        rom[12'd915] = 16'hf1a7;
        rom[12'd916] = 16'hf1a7;
        rom[12'd917] = 16'hf1a7;
        rom[12'd918] = 16'hf1a7;
        rom[12'd919] = 16'hf1a7;
        rom[12'd920] = 16'hf1a7;
        rom[12'd921] = 16'hf1a7;
        rom[12'd922] = 16'hf1a7;
        rom[12'd923] = 16'hf1a7;
        rom[12'd924] = 16'hf1a7;
        rom[12'd925] = 16'hf1a7;
        rom[12'd926] = 16'hf1a7;
        rom[12'd927] = 16'hf1a7;
        rom[12'd928] = 16'hf1a7;
        rom[12'd929] = 16'hf1a7;
        rom[12'd930] = 16'hf1a7;
        rom[12'd931] = 16'hfc51;
        rom[12'd969] = 16'hce5a;
        rom[12'd970] = 16'h3a2c;
        rom[12'd971] = 16'h1969;
        rom[12'd972] = 16'h1969;
        rom[12'd973] = 16'h1969;
        rom[12'd974] = 16'h1969;
        rom[12'd975] = 16'h1969;
        rom[12'd976] = 16'h218a;
        rom[12'd977] = 16'h7c33;
        rom[12'd978] = 16'hf32d;
        rom[12'd979] = 16'hf249;
        rom[12'd980] = 16'hf249;
        rom[12'd981] = 16'hf249;
        rom[12'd982] = 16'hf249;
        rom[12'd983] = 16'hf249;
        rom[12'd984] = 16'hf249;
        rom[12'd985] = 16'hf249;
        rom[12'd986] = 16'hf249;
        rom[12'd987] = 16'hf249;
        rom[12'd988] = 16'hf249;
        rom[12'd989] = 16'hf249;
        rom[12'd990] = 16'hf249;
        rom[12'd991] = 16'hf249;
        rom[12'd992] = 16'hf249;
        rom[12'd993] = 16'hf249;
        rom[12'd994] = 16'hf2ec;
        rom[12'd1033] = 16'h6390;
        rom[12'd1034] = 16'h1969;
        rom[12'd1035] = 16'h1969;
        rom[12'd1036] = 16'h1969;
        rom[12'd1037] = 16'h1969;
        rom[12'd1038] = 16'h1969;
        rom[12'd1039] = 16'h1969;
        rom[12'd1040] = 16'h1969;
        rom[12'd1041] = 16'h52ee;
        rom[12'd1043] = 16'hfc92;
        rom[12'd1044] = 16'hf451;
        rom[12'd1045] = 16'hf451;
        rom[12'd1046] = 16'hf451;
        rom[12'd1047] = 16'hf451;
        rom[12'd1048] = 16'hf451;
        rom[12'd1049] = 16'hf451;
        rom[12'd1050] = 16'hf451;
        rom[12'd1051] = 16'hf451;
        rom[12'd1052] = 16'hf451;
        rom[12'd1053] = 16'hf451;
        rom[12'd1054] = 16'hf451;
        rom[12'd1055] = 16'hf451;
        rom[12'd1056] = 16'hf451;
        rom[12'd1057] = 16'hf471;
        rom[12'd1096] = 16'hce5a;
        rom[12'd1097] = 16'h3a2c;
        rom[12'd1098] = 16'h1969;
        rom[12'd1099] = 16'h1969;
        rom[12'd1100] = 16'h1969;
        rom[12'd1101] = 16'h1969;
        rom[12'd1102] = 16'h1969;
        rom[12'd1103] = 16'h1969;
        rom[12'd1104] = 16'h1969;
        rom[12'd1105] = 16'h218a;
        rom[12'd1106] = 16'h9d15;
        rom[12'd1107] = 16'hf26a;
        rom[12'd1108] = 16'he8e4;
        rom[12'd1109] = 16'he8e4;
        rom[12'd1110] = 16'he8e4;
        rom[12'd1111] = 16'he8e4;
        rom[12'd1112] = 16'he8e4;
        rom[12'd1113] = 16'he8e4;
        rom[12'd1114] = 16'he8e4;
        rom[12'd1115] = 16'he8e4;
        rom[12'd1116] = 16'he8e4;
        rom[12'd1117] = 16'he8e4;
        rom[12'd1118] = 16'he8e4;
        rom[12'd1119] = 16'he8e4;
        rom[12'd1120] = 16'he8e4;
        rom[12'd1121] = 16'hf229;
        rom[12'd1160] = 16'h6390;
        rom[12'd1161] = 16'h1969;
        rom[12'd1162] = 16'h1969;
        rom[12'd1163] = 16'h1969;
        rom[12'd1164] = 16'h1969;
        rom[12'd1165] = 16'h1969;
        rom[12'd1166] = 16'h1969;
        rom[12'd1167] = 16'h1969;
        rom[12'd1168] = 16'h1969;
        rom[12'd1169] = 16'h1969;
        rom[12'd1170] = 16'h530f;
        rom[12'd1171] = 16'hfc31;
        rom[12'd1172] = 16'hfb4e;
        rom[12'd1173] = 16'hfb4e;
        rom[12'd1174] = 16'hfb4e;
        rom[12'd1175] = 16'hfb4e;
        rom[12'd1176] = 16'hfb4e;
        rom[12'd1177] = 16'hfb4e;
        rom[12'd1178] = 16'hfb4e;
        rom[12'd1179] = 16'hfb4e;
        rom[12'd1180] = 16'hfb4e;
        rom[12'd1181] = 16'hfb4e;
        rom[12'd1182] = 16'hfb4e;
        rom[12'd1183] = 16'hfb4e;
        rom[12'd1184] = 16'hfb4e;
        rom[12'd1185] = 16'hfbf0;
        rom[12'd1223] = 16'hc65a;
        rom[12'd1224] = 16'h322c;
        rom[12'd1225] = 16'h1969;
        rom[12'd1226] = 16'h1969;
        rom[12'd1227] = 16'h1969;
        rom[12'd1228] = 16'h1969;
        rom[12'd1229] = 16'h6bb1;
        rom[12'd1230] = 16'h1969;
        rom[12'd1231] = 16'h1969;
        rom[12'd1232] = 16'h1969;
        rom[12'd1233] = 16'h1969;
        rom[12'd1234] = 16'h218a;
        rom[12'd1235] = 16'h94d5;
        rom[12'd1236] = 16'hf3f0;
        rom[12'd1237] = 16'hf28a;
        rom[12'd1238] = 16'hf28a;
        rom[12'd1239] = 16'hf28a;
        rom[12'd1240] = 16'hf28a;
        rom[12'd1241] = 16'hf28a;
        rom[12'd1242] = 16'hf28a;
        rom[12'd1243] = 16'hf28a;
        rom[12'd1244] = 16'hf28a;
        rom[12'd1245] = 16'hf28a;
        rom[12'd1246] = 16'hf28a;
        rom[12'd1247] = 16'hf28a;
        rom[12'd1248] = 16'hf3f0;
        rom[12'd1287] = 16'h6370;
        rom[12'd1288] = 16'h1969;
        rom[12'd1289] = 16'h1969;
        rom[12'd1290] = 16'h1969;
        rom[12'd1291] = 16'h1969;
        rom[12'd1292] = 16'h428d;
        rom[12'd1293] = 16'he73d;
        rom[12'd1294] = 16'h426d;
        rom[12'd1295] = 16'h1969;
        rom[12'd1296] = 16'h1969;
        rom[12'd1297] = 16'h1969;
        rom[12'd1298] = 16'h1969;
        rom[12'd1299] = 16'h52ee;
        rom[12'd1300] = 16'hf3f0;
        rom[12'd1301] = 16'he966;
        rom[12'd1302] = 16'he946;
        rom[12'd1303] = 16'he946;
        rom[12'd1304] = 16'he946;
        rom[12'd1305] = 16'he946;
        rom[12'd1306] = 16'he946;
        rom[12'd1307] = 16'he946;
        rom[12'd1308] = 16'he946;
        rom[12'd1309] = 16'he946;
        rom[12'd1310] = 16'he946;
        rom[12'd1311] = 16'he966;
        rom[12'd1312] = 16'hf3cf;
        rom[12'd1350] = 16'hc65a;
        rom[12'd1351] = 16'h322c;
        rom[12'd1352] = 16'h1969;
        rom[12'd1353] = 16'h1969;
        rom[12'd1354] = 16'h1969;
        rom[12'd1355] = 16'h1969;
        rom[12'd1356] = 16'h7c12;
        rom[12'd1358] = 16'h7c12;
        rom[12'd1359] = 16'h1969;
        rom[12'd1360] = 16'h1969;
        rom[12'd1361] = 16'h1969;
        rom[12'd1362] = 16'h1969;
        rom[12'd1363] = 16'h196a;
        rom[12'd1364] = 16'h8c94;
        rom[12'd1414] = 16'h6370;
        rom[12'd1415] = 16'h1969;
        rom[12'd1416] = 16'h1969;
        rom[12'd1417] = 16'h1969;
        rom[12'd1418] = 16'h1969;
        rom[12'd1419] = 16'h426d;
        rom[12'd1420] = 16'hdf1c;
        rom[12'd1423] = 16'h4aee;
        rom[12'd1424] = 16'h1969;
        rom[12'd1425] = 16'h1969;
        rom[12'd1426] = 16'h1969;
        rom[12'd1427] = 16'h1969;
        rom[12'd1428] = 16'h4ace;
        rom[12'd1429] = 16'hfcd3;
        rom[12'd1430] = 16'hf229;
        rom[12'd1431] = 16'hf209;
        rom[12'd1432] = 16'hf209;
        rom[12'd1433] = 16'hf209;
        rom[12'd1434] = 16'hf209;
        rom[12'd1435] = 16'hf209;
        rom[12'd1436] = 16'hf209;
        rom[12'd1437] = 16'hf209;
        rom[12'd1438] = 16'hf229;
        rom[12'd1439] = 16'hfd35;
        rom[12'd1477] = 16'hc639;
        rom[12'd1478] = 16'h322c;
        rom[12'd1479] = 16'h1969;
        rom[12'd1480] = 16'h1969;
        rom[12'd1481] = 16'h1969;
        rom[12'd1482] = 16'h1969;
        rom[12'd1483] = 16'h7412;
        rom[12'd1487] = 16'h9cf5;
        rom[12'd1488] = 16'h218a;
        rom[12'd1489] = 16'h1969;
        rom[12'd1490] = 16'h1969;
        rom[12'd1491] = 16'h1969;
        rom[12'd1492] = 16'h1969;
        rom[12'd1493] = 16'h8453;
        rom[12'd1494] = 16'hf26a;
        rom[12'd1495] = 16'hf1a7;
        rom[12'd1496] = 16'hf1a7;
        rom[12'd1497] = 16'hf1a7;
        rom[12'd1498] = 16'hf1a7;
        rom[12'd1499] = 16'hf1a7;
        rom[12'd1500] = 16'hf1a7;
        rom[12'd1501] = 16'hf1a7;
        rom[12'd1502] = 16'hf28a;
        rom[12'd1541] = 16'h6370;
        rom[12'd1542] = 16'h1969;
        rom[12'd1543] = 16'h1969;
        rom[12'd1544] = 16'h1969;
        rom[12'd1545] = 16'h1969;
        rom[12'd1546] = 16'h3a6d;
        rom[12'd1547] = 16'hdefc;
        rom[12'd1552] = 16'h4aae;
        rom[12'd1553] = 16'h1969;
        rom[12'd1554] = 16'h1969;
        rom[12'd1555] = 16'h1969;
        rom[12'd1556] = 16'h1969;
        rom[12'd1557] = 16'h4aae;
        rom[12'd1604] = 16'hc639;
        rom[12'd1605] = 16'h322c;
        rom[12'd1606] = 16'h1969;
        rom[12'd1607] = 16'h1969;
        rom[12'd1608] = 16'h1969;
        rom[12'd1609] = 16'h1969;
        rom[12'd1610] = 16'h73f1;
        rom[12'd1616] = 16'h94d5;
        rom[12'd1617] = 16'h21aa;
        rom[12'd1618] = 16'h1969;
        rom[12'd1619] = 16'h1969;
        rom[12'd1620] = 16'h1969;
        rom[12'd1621] = 16'h1969;
        rom[12'd1622] = 16'h7c12;
        rom[12'd1623] = 16'hf30c;
        rom[12'd1624] = 16'hf1e8;
        rom[12'd1625] = 16'hf1e8;
        rom[12'd1626] = 16'hf1e8;
        rom[12'd1627] = 16'hf1e8;
        rom[12'd1628] = 16'hf1e8;
        rom[12'd1629] = 16'hf36e;
        rom[12'd1668] = 16'h6370;
        rom[12'd1669] = 16'h1969;
        rom[12'd1670] = 16'h1969;
        rom[12'd1671] = 16'h1969;
        rom[12'd1672] = 16'h1969;
        rom[12'd1673] = 16'h3a4c;
        rom[12'd1674] = 16'hd6db;
        rom[12'd1681] = 16'h5b2f;
        rom[12'd1682] = 16'h1969;
        rom[12'd1683] = 16'h1969;
        rom[12'd1684] = 16'h1969;
        rom[12'd1685] = 16'h1969;
        rom[12'd1686] = 16'h428d;
        rom[12'd1687] = 16'hf34e;
        rom[12'd1688] = 16'hf1e8;
        rom[12'd1689] = 16'hf1e8;
        rom[12'd1690] = 16'hf1e8;
        rom[12'd1691] = 16'hf1e8;
        rom[12'd1692] = 16'hf1e8;
        rom[12'd1693] = 16'hfbf0;
        rom[12'd1731] = 16'hc639;
        rom[12'd1732] = 16'h322c;
        rom[12'd1733] = 16'h1969;
        rom[12'd1734] = 16'h1969;
        rom[12'd1735] = 16'h1969;
        rom[12'd1736] = 16'h1969;
        rom[12'd1737] = 16'h4ace;
        rom[12'd1738] = 16'h52ee;
        rom[12'd1739] = 16'h52ee;
        rom[12'd1740] = 16'h52ee;
        rom[12'd1741] = 16'h52ee;
        rom[12'd1742] = 16'h6370;
        rom[12'd1743] = 16'hef5d;
        rom[12'd1745] = 16'ha556;
        rom[12'd1746] = 16'h29eb;
        rom[12'd1747] = 16'h1969;
        rom[12'd1748] = 16'h1969;
        rom[12'd1749] = 16'h1969;
        rom[12'd1750] = 16'h1969;
        rom[12'd1751] = 16'h73d1;
        rom[12'd1752] = 16'hfcb3;
        rom[12'd1753] = 16'hf492;
        rom[12'd1754] = 16'hf492;
        rom[12'd1755] = 16'hf492;
        rom[12'd1756] = 16'hfcf4;
        rom[12'd1795] = 16'h6370;
        rom[12'd1796] = 16'h1969;
        rom[12'd1797] = 16'h1969;
        rom[12'd1798] = 16'h1969;
        rom[12'd1799] = 16'h1969;
        rom[12'd1800] = 16'h1969;
        rom[12'd1801] = 16'h1969;
        rom[12'd1802] = 16'h1969;
        rom[12'd1803] = 16'h1969;
        rom[12'd1804] = 16'h1969;
        rom[12'd1805] = 16'h1969;
        rom[12'd1806] = 16'h1969;
        rom[12'd1807] = 16'h7c53;
        rom[12'd1810] = 16'h530f;
        rom[12'd1811] = 16'h1969;
        rom[12'd1812] = 16'h1969;
        rom[12'd1813] = 16'h1969;
        rom[12'd1814] = 16'h1969;
        rom[12'd1815] = 16'h3a6d;
        rom[12'd1816] = 16'hf34d;
        rom[12'd1817] = 16'he8e4;
        rom[12'd1818] = 16'he8e4;
        rom[12'd1819] = 16'he905;
        rom[12'd1820] = 16'hfc51;
        rom[12'd1858] = 16'hc639;
        rom[12'd1859] = 16'h322c;
        rom[12'd1860] = 16'h1969;
        rom[12'd1861] = 16'h1969;
        rom[12'd1862] = 16'h1969;
        rom[12'd1863] = 16'h1969;
        rom[12'd1864] = 16'h1969;
        rom[12'd1865] = 16'h1969;
        rom[12'd1866] = 16'h1969;
        rom[12'd1867] = 16'h1969;
        rom[12'd1868] = 16'h1969;
        rom[12'd1869] = 16'h1969;
        rom[12'd1870] = 16'h1969;
        rom[12'd1871] = 16'h426d;
        rom[12'd1872] = 16'hdedb;
        rom[12'd1874] = 16'had97;
        rom[12'd1875] = 16'h322c;
        rom[12'd1876] = 16'h1969;
        rom[12'd1877] = 16'h1969;
        rom[12'd1878] = 16'h1969;
        rom[12'd1879] = 16'h1969;
        rom[12'd1880] = 16'h7c12;
        rom[12'd1881] = 16'hf2ab;
        rom[12'd1882] = 16'hf26a;
        rom[12'd1883] = 16'hf30c;
        rom[12'd1922] = 16'h6350;
        rom[12'd1923] = 16'h1969;
        rom[12'd1924] = 16'h1969;
        rom[12'd1925] = 16'h1969;
        rom[12'd1926] = 16'h1969;
        rom[12'd1927] = 16'h1969;
        rom[12'd1928] = 16'h1969;
        rom[12'd1929] = 16'h1969;
        rom[12'd1930] = 16'h1969;
        rom[12'd1931] = 16'h1969;
        rom[12'd1932] = 16'h1969;
        rom[12'd1933] = 16'h1969;
        rom[12'd1934] = 16'h1969;
        rom[12'd1935] = 16'h1969;
        rom[12'd1936] = 16'h6bb1;
        rom[12'd1939] = 16'h6bb1;
        rom[12'd1940] = 16'h1969;
        rom[12'd1941] = 16'h1969;
        rom[12'd1942] = 16'h1969;
        rom[12'd1943] = 16'h1969;
        rom[12'd1944] = 16'h42ad;
        rom[12'd1945] = 16'hf638;
        rom[12'd1946] = 16'hf32d;
        rom[12'd1947] = 16'hfedb;
        rom[12'd1985] = 16'hdedb;
        rom[12'd1986] = 16'h428d;
        rom[12'd1987] = 16'h29cb;
        rom[12'd1988] = 16'h29cb;
        rom[12'd1989] = 16'h29cb;
        rom[12'd1990] = 16'h29cb;
        rom[12'd1991] = 16'h29cb;
        rom[12'd1992] = 16'h29cb;
        rom[12'd1993] = 16'h29cb;
        rom[12'd1994] = 16'h29cb;
        rom[12'd1995] = 16'h29cb;
        rom[12'd1996] = 16'h29cb;
        rom[12'd1997] = 16'h29cb;
        rom[12'd1998] = 16'h29cb;
        rom[12'd1999] = 16'h29cb;
        rom[12'd2000] = 16'h42ad;
        rom[12'd2001] = 16'hdedb;
        rom[12'd2003] = 16'hadb7;
        rom[12'd2004] = 16'h3a4c;
        rom[12'd2005] = 16'h29cb;
        rom[12'd2006] = 16'h29cb;
        rom[12'd2007] = 16'h29cb;
        rom[12'd2008] = 16'h29cb;
        rom[12'd2009] = 16'h8453;
        rom[12'd2010] = 16'hf3cf;
    end
    always @(posedge I_clk) O_pixel <= rom[I_addr[11:0]];
endmodule
