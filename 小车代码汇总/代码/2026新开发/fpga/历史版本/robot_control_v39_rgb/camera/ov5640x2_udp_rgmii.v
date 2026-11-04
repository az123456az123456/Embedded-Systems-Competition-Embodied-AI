/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : ov5640x2_udp_gmii
// Description   : 摄像头采集，以太网UDP协议发送
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module camera_dual_gmii(
 rgb_session_valid,rgb_session_epoch,tx_allow,tx_busy,tx_clk,tx_data,tx_valid,pll_ready,diag_clk,
 af_status1,af_status2,
	camera_status,
	clk,
	rst_n,
	
	eth_gtxc,
	eth_txd,
	eth_txen,

	eth_mdc,
	eth_mdio,
	eth_rst_n,

    led,
	
	camera1_sclk,
	camera1_sdat,
	camera1_vsync,
	camera1_href,
	camera1_pclk,
	camera1_xclk,
	camera1_data,
	camera1_rst_n,
	camera1_pwdn,
	
	
	camera2_sclk,
	camera2_sdat,
	camera2_vsync,
	camera2_href,
	camera2_pclk,
	camera2_xclk,
	camera2_data,
	camera2_rst_n,
	camera2_pwdn
);
 input tx_allow;
 input rgb_session_valid;
 input [15:0] rgb_session_epoch;
 output tx_busy,tx_clk,tx_valid,pll_ready,diag_clk;
 wire clk_50m;
 wire Init_Done1,Init_Done2,Init_Done;
 assign diag_clk=clk_50m;
 output [7:0] tx_data;
 output [15:0] af_status1,af_status2;
	output [2:0] camera_status;
 wire pair_fault;
	assign camera_status = {pair_fault,Init_Done2,Init_Done1};
	input clk;
	input rst_n;
	
	output eth_gtxc;
	output [3:0] eth_txd;
	output eth_txen;
	inout eth_mdc;
	inout eth_mdio;
	output eth_rst_n;
    output led;
	assign led = Init_Done;
	//camera1 interface
	output		camera1_sclk;
	inout		camera1_sdat;
	input		camera1_vsync;
	input		camera1_href;
	input		camera1_pclk;
	output		camera1_xclk;
	input [7:0]	camera1_data;
	output		camera1_rst_n;
	output		camera1_pwdn;
	
	//camera2 interface
	output		camera2_sclk;
	inout		camera2_sdat;
	input		camera2_vsync;
	input		camera2_href;
	input		camera2_pclk;
	output		camera2_xclk;
	input [7:0]	camera2_data;
	output		camera2_rst_n;
	output		camera2_pwdn;
	
	wire clk_125m;
	wire clk_24m;
	wire locked;
	wire camera_async_reset_n = locked && rst_n;
    wire global_rst_n,tx_reset_n,pclk1_reset_n,pclk2_reset_n;
    camera_reset_release cfg_reset(.clk(clk_50m),.async_reset_n(camera_async_reset_n),.reset_n(global_rst_n));
    camera_reset_release tx_reset(.clk(clk_125m),.async_reset_n(camera_async_reset_n),.reset_n(tx_reset_n));
    camera_reset_release pclk1_reset(.clk(camera1_pclk),.async_reset_n(camera_async_reset_n),.reset_n(pclk1_reset_n));
    camera_reset_release pclk2_reset(.clk(camera2_pclk),.async_reset_n(camera_async_reset_n),.reset_n(pclk2_reset_n));
    (* syn_async_reg=1 *) reg [1:0] pclk1_init_sync=0,pclk2_init_sync=0;
    always @(posedge camera1_pclk or negedge pclk1_reset_n)
        if(!pclk1_reset_n)pclk1_init_sync<=0;else pclk1_init_sync<={pclk1_init_sync[0],Init_Done};
    always @(posedge camera2_pclk or negedge pclk2_reset_n)
        if(!pclk2_reset_n)pclk2_init_sync<=0;else pclk2_init_sync<={pclk2_init_sync[0],Init_Done};
	assign camera1_xclk = clk_24m;
	assign camera2_xclk = clk_24m;

    Gowin_PLL your_instance_name(
        .clkin(clk), //input  clkin
        .clkout0(clk_125m), //output  clkout0
        .clkout1(clk_50m), //output  clkout1
        .clkout2(clk_24m), //output  clkout2
        .lock(locked), //output  lock
        .mdclk(clk), //input  mdclk
        .reset(~rst_n) //input  reset
    );
	
	wire phy_init;	//phy initial done
	wire Rd_Data;
	phy_reg_config phy_reg_config(
		.Clk       			(clk_50m       	),
		.Rst_n     			(global_rst_n  	),
		.Phy_rst_n 			(eth_rst_n    	),
		.mdc       			(eth_mdc    	), 
		.mdio      			(eth_mdio   	),
		.Rd_Data  			(Rd_Data      	),
		.Phy_init_done		(phy_init      	)
	);

	assign camera1_pwdn = 1'b0;
	assign camera2_pwdn = 1'b0;
	
	localparam RGB = 0;
	localparam JPEG = 1;
	
	parameter IMAGE_WIDTH  = 960; //图片宽度
	parameter IMAGE_HEIGHT = 1080; //图片高度(≤720)
	parameter IMAGE_FLIP   = 0;   //0：不翻转，1：上下翻转
	parameter IMAGE_MIRROR = 0;   //0：不镜像，1：左右镜像
	//-----------------------------------------------
	//摄像头1,摄像头2初始化配置
	assign Init_Done = (Init_Done1 && Init_Done2);
	camera_init
	#(
		.CAMERA_TYPE		("ov5640"		),
		.IMAGE_TYPE         (0              ),
		.IMAGE_WIDTH		(IMAGE_WIDTH	),
		.IMAGE_HEIGHT		(IMAGE_HEIGHT	),
		.IMAGE_FLIP_EN		(1	            ),
		.IMAGE_MIRROR_EN	(0             	)
	)
	camera_init1(
 .af_status(af_status1),
		.Clk				(clk_50m		),
		.Rst_n				(global_rst_n	),
		.Init_Done			(Init_Done1		),
		.camera_rst_n 		(camera1_rst_n  ),
		.i2c_sclk			(camera1_sclk	),
		.i2c_sdat			(camera1_sdat	)
	),
	camera_init2(
 .af_status(af_status2),
		.Clk				(clk_50m		),
		.Rst_n				(global_rst_n	),
		.Init_Done			(Init_Done2		),
		.camera_rst_n   	(camera2_rst_n  ),
		.i2c_sclk			(camera2_sclk	),
		.i2c_sdat			(camera2_sdat	)
	);
	
    // Model-input transport replaces the raw preview/UDP row path.
    wire GMII_GTXC=clk_125m;
    wire [7:0] GMII_TXD;
    wire GMII_TXEN;
    reg [15:0] pending_epoch,epoch;
    reg commit;
    reg [2:0] fifo_reset_count;
    wire cdc1_ready,cdc2_ready;
    wire session1_valid,session2_valid;
    wire [511:0] session1_data,session2_data;
    wire fifo_reset=!tx_reset_n || fifo_reset_count!=0;
    always @(posedge clk_125m or negedge tx_reset_n)begin
        if(!tx_reset_n)begin pending_epoch<=0;epoch<=0;commit<=0;fifo_reset_count<=4;end
        else begin
            commit<=0;
            if(fifo_reset_count!=0)fifo_reset_count<=fifo_reset_count-1'b1;
            if(rgb_session_valid && rgb_session_epoch!=0)pending_epoch<=rgb_session_epoch;
            // Finish an Ethernet frame before applying a different session.
            // A repeated START for the same epoch does not reset the stream.
            if(pending_epoch!=0 && pending_epoch!=epoch && !tx_busy &&
               cdc1_ready && cdc2_ready && fifo_reset_count==0)begin
                epoch<=pending_epoch;commit<=1;fifo_reset_count<=4;
            end
        end
    end
    imu_snapshot_cdc #(.CAPTURE_WHEN_IDLE(1)) rgb_session1_mailbox(
        .src_clk(clk_125m),.src_rst_n(tx_reset_n),.src_valid(commit),
        .src_data({496'd0,epoch}),.src_ready(cdc1_ready),
        .dst_clk(camera1_pclk),.dst_rst_n(pclk1_reset_n),.dst_ready(1'b1),
        .dst_valid(session1_valid),.dst_data(session1_data));
    imu_snapshot_cdc #(.CAPTURE_WHEN_IDLE(1)) rgb_session2_mailbox(
        .src_clk(clk_125m),.src_rst_n(tx_reset_n),.src_valid(commit),
        .src_data({496'd0,epoch}),.src_ready(cdc2_ready),
        .dst_clk(camera2_pclk),.dst_rst_n(pclk2_reset_n),.dst_ready(1'b1),
        .dst_valid(session2_valid),.dst_data(session2_data));
    reg [16:0] millisecond_div;
    reg [31:0] uptime_ms,uptime_gray;
    always @(posedge clk_125m or negedge tx_reset_n)begin
        if(!tx_reset_n)begin millisecond_div<=0;uptime_ms<=0;uptime_gray<=0;end
        else if(millisecond_div==124999)begin
            millisecond_div<=0;uptime_ms<=uptime_ms+1'b1;
            uptime_gray<=((uptime_ms+1'b1)>>1)^(uptime_ms+1'b1);
        end else millisecond_div<=millisecond_div+1'b1;
    end
    wire [35:0] word1,word2,read1,read2;
    wire write1,write2,full1,full2,empty1,empty2,pop1,pop2;
    wire [10:0] used1,used2;
    wire bridge_fault1,bridge_fault2,packet_fault;
    assign pair_fault=bridge_fault1 || bridge_fault2 || packet_fault;
    rgb_capture_bridge bridge1(
        .clk(camera1_pclk),.rst_n(pclk1_reset_n),.init_done(pclk1_init_sync[1]),
        .href(camera1_href),.vsync(camera1_vsync),.raw(camera1_data),
        .session_valid(session1_valid),.session_epoch(session1_data[15:0]),.uptime_gray(uptime_gray),
        .fifo_full(full1 || used1>=2044),.fifo_write(write1),.fifo_word(word1),
        .fault(bridge_fault1),.completed_frames(),.dropped_frames());
    rgb_capture_bridge bridge2(
        .clk(camera2_pclk),.rst_n(pclk2_reset_n),.init_done(pclk2_init_sync[1]),
        .href(camera2_href),.vsync(camera2_vsync),.raw(camera2_data),
        .session_valid(session2_valid),.session_epoch(session2_data[15:0]),.uptime_gray(uptime_gray),
        .fifo_full(full2 || used2>=2044),.fifo_write(write2),.fifo_word(word2),
        .fault(bridge_fault2),.completed_frames(),.dropped_frames());
    rgb_stream_fifo fifo1(.wrclk(camera1_pclk),.rdclk(clk_125m),.aclr(fifo_reset),
        .wrreq(write1),.rdreq(pop1),.wrdata(word1),.rddata(read1),
        .rdusedw(),.wrusedw(used1),.empty(empty1),.full(full1));
    rgb_stream_fifo fifo2(.wrclk(camera2_pclk),.rdclk(clk_125m),.aclr(fifo_reset),
        .wrreq(write2),.rdreq(pop2),.wrdata(word2),.rddata(read2),
        .rdusedw(),.wrusedw(used2),.empty(empty2),.full(full2));
    rgb_packet_tx sender(.clk(clk_125m),.rst_n(tx_reset_n),.epoch(epoch),
        .tx_allow(tx_allow),.tx_busy(tx_busy),.gmii_data(GMII_TXD),.gmii_valid(GMII_TXEN),
        .fifo0_data(read1),.fifo0_empty(empty1),.fifo0_pop(pop1),
        .fifo1_data(read2),.fifo1_empty(empty2),.fifo1_pop(pop2),
        .tx_pending(),.pair_fault(packet_fault),.pair_fault_count(),.packets_sent(),.pairs_started());

    assign tx_clk=GMII_GTXC;
    assign tx_data=GMII_TXD;
    assign tx_valid=GMII_TXEN;
    assign pll_ready=locked;
    // Legacy external PHY outputs are unused for board-internal direct RGMII.
    assign eth_gtxc=1'b0;
    assign eth_txd=4'b0;
    assign eth_txen=1'b0;
endmodule

// Dedicated reset boundary: assert asynchronously, release after two local edges.
module camera_reset_release(input wire clk,async_reset_n,output wire reset_n);
    (* syn_async_reg=1 *) reg [1:0] release_sync=0;
    always @(posedge clk or negedge async_reset_n)
        if(!async_reset_n)release_sync<=0;else release_sync<={release_sync[0],1'b1};
    assign reset_n=release_sync[1];
endmodule
