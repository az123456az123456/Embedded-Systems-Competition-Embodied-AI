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
 tx_allow,tx_busy,tx_clk,tx_data,tx_valid,pll_ready,
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
 output tx_busy,tx_clk,tx_valid,pll_ready;
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
	wire clk_50m;
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
	wire Init_Done1;
	wire Init_Done2;
	wire Init_Done = (Init_Done1 && Init_Done2);
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
	
	//-----------------------------------------------
	//摄像头1,摄像头2图像输出
	wire fifo1_aclr;
	wire fifo1_wrreq;
	wire [7:0] fifo1_wrdata;
	wire fifo2_aclr;
	wire fifo2_wrreq;
	wire [7:0] fifo2_wrdata;
	Camera_ETH_Formator Camera_ETH_Formator1(
		.PCLK		    	(camera1_pclk   ),
		.Rst_n(pclk1_reset_n),
		.Init_Done(pclk1_init_sync[1]),
		.HREF	     		(camera1_href   ),
		.VSYNC		    	(camera1_vsync	),
		.DATA			    (camera1_data	),
		.fifo_aclr			(fifo1_aclr		),
		.wrdata	            (fifo1_wrdata	),
		.wrreq		     	(fifo1_wrreq	)
	),
	Camera_ETH_Formator2(
		.PCLK			    (camera2_pclk   ),
		.Rst_n(pclk2_reset_n),
		.Init_Done(pclk2_init_sync[1]),
		.HREF       		(camera2_href	),
		.VSYNC      		(camera2_vsync	),
		.DATA       		(camera2_data	),
		.fifo_aclr			(fifo2_aclr		),
		.wrdata	        	(fifo2_wrdata	),
		.wrreq	    		(fifo2_wrreq	)
	);
	


	wire fifo_aclr;
	wire fifo_wrreq;
	wire [7:0] fifo_wrdata;

	wire wrclk;//ethernet fifo wrclk
	wire GMII_GTXC;
	assign GMII_GTXC = clk_125m;

	
	controller_top controller_top(
        .pair_fault(pair_fault),
        .clk(GMII_GTXC),
        .rst_n(tx_reset_n),
        
        .wrclk1(camera1_pclk),
        .fifo1_aclr(fifo1_aclr),
        .wrdata1(fifo1_wrdata),
        .wrreq1(fifo1_wrreq),
        
        .wrclk2(camera2_pclk),
        .fifo2_aclr(fifo2_aclr),
        .wrdata2(fifo2_wrdata),
        .wrreq2(fifo2_wrreq),
        
        .wrclk(wrclk),
        .fifo_aclr(fifo_aclr),
        .fifo_wrreq(fifo_wrreq),
        .fifo_wrdata(fifo_wrdata)
    );
	
	
	wire [7:0] GMII_TXD;
	wire GMII_TXEN;
    // Synchronize camera init completion into the shared 125MHz scheduler.
    (* syn_async_reg=1 *) reg [1:0] init_done_tx=0;
    always @(posedge GMII_GTXC or negedge tx_reset_n)
        if(!tx_reset_n) init_done_tx<=0;
        else init_done_tx<={init_done_tx[0],Init_Done};

	UDP_Send UDP_Send(
        .tx_allow(tx_allow),.tx_busy(tx_busy),
		.Clk(1'b0),
		.GMII_GTXC			(GMII_GTXC		),
		.GMII_TXD			(GMII_TXD		),
		.GMII_TXEN			(GMII_TXEN		),
		.Rst_n(init_done_tx[1]),
		.Tx_Done			(				),
		.data_length		(IMAGE_WIDTH*2+2		),
		.des_ip				(32'hc0_a8_00_03),//hc0_a8_00_03; 192.168.0.3
		.des_mac			(48'hFF_FF_FF_FF_FF_FF),
		.des_port			(16'd6102		),
		.src_ip				(32'hc0_a8_00_02),//hc0_a8_65_02; 192.168.0.2
		.src_mac			(48'h00_0a_35_01_fe_c0),
		.src_port			(16'd5000		),
		//.wrclk			(camera_pclk		),
		.wrclk				(wrclk			),
		.wrdata				(fifo_wrdata	),
		.wrreq				(fifo_wrreq		),
		.aclr				(fifo_aclr		),
		.wrusedw			( )
	);
	
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
