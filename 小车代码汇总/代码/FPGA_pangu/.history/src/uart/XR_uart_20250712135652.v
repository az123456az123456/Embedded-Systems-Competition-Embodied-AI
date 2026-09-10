`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    13:57:05 06/23/2016 
// Design Name: 
// Module Name:    XR_uart 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 	串口通信程序，包括接收、发送、发送控制
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_uart(
	 input				clk_sys
	,input 				rx
	,output 			tx
	,input 				rx0
	,output 			tx0
	,input 				rx1
	,output 			tx1
	,input 				rst_n
	,output	[7:0]		rxdata
	,output	[7:0]		cnt
	,output				clk_uart
	,input	[3:0]		send_flag
	,input	[23:0]		send_data	
	,input				init_flag
);
	 
	 
/******************************************************
   net 
	以USB-TTL为主调试口，优先解析USB接口指令，其次解析uart0
	
******************************************************/  
	 
wire 		[7:0] 		txdata;
wire		[7:0]		rxdata_u;
wire		[7:0]		rxdata_0;
wire		[7:0]		rxdata_1;
wire		[7:0]		cnt_u;
wire		[7:0]		cnt_0;
wire		[7:0]		cnt_1;
wire 					rdsig;
wire					rdsig_u;
wire					rdsig_0;
wire					rdsig_1;
wire					receive;

assign	rxdata 	=	receive	?	rxdata_0	:		rxdata_u;
assign	cnt 	=	receive	?	cnt_0		:		cnt_u;
assign	rdsig	=	receive	?	rdsig_0		:		rdsig_u;

/******************************************************
INSTANCE ./uart/XR_uart_clkdiv.v	 分频模块
******************************************************/  
XR_uart_clkdiv 	XR_uart_clkdiv_inst 
(
	.clk_sys	(clk_sys				),
	.rst_n		(rst_n					),
	.clkout		(clk_uart				)
);

/******************************************************
INSTANCE ./uart/XR_uart_RX.v	接收端
******************************************************/   
XR_uart_RX 	XR_uart_RX_inst 
(
	.clk		(clk_uart				),				
	.rx			(rx						),
	.rst_n		(rst_n					),
	.rxdata		(rxdata_u				),
	.rdsig		(rdsig_u				),
	.cnt		(cnt_u					)		
);
 
/******************************************************
INSTANCE ./uart/XR_uart_TX.v	发送端
******************************************************/ 
XR_uart_TX 	XR_uart_TX_inst 
(
	.clk		(clk_uart				),			
	.txdata		(txdata					),
	.wrsig		(wrsig					),
	.tx			(tx						)
);
 
/******************************************************
INSTANCE ./uart/XR_uart_RX0.v	
******************************************************/   
XR_uart_RX0 XR_uart_RX0_inst 
(
	.clk				(clk_uart				),				
	.rx					(rx0					),
	.rst_n				(rst_n					),
	.rxdata				(rxdata_0				),
	.rdsig				(rdsig_0				),
	.cnt				(cnt_0					),
	.init_flag			(init_flag				),
	.receive			(receive				)
 );
 
/******************************************************
INSTANCE ./uart/XR_uart_TX0.v
******************************************************/ 
XR_uart_TX0 XR_uart_TX0_inst 
(
	.clk				(clk_uart				),			
	.txdata				(txdata					),
	.wrsig				(wrsig					),
	.tx					(tx0					),
	.send_flag			(send_flag				)
);

/******************************************************
INSTANCE ./uart/XR_uart_RX1.v	
******************************************************/   
XR_uart_RX1 XR_uart_RX1_inst 
(
	.clk				(clk_uart				),				
	.rx					(rx1					),
	.rst_n				(rst_n					),
	.rxdata				(rxdata_1				),
	.rdsig				(rdsig_1				),
	.cnt				(cnt_1					)
);
  
/******************************************************
INSTANCE ./uart/XR_uart_TX1.v
******************************************************/ 
XR_uart_TX1 XR_uart_TX1_inst 
(
	.clk				(clk_uart				),			
	.txdata				(txdata					),
	.wrsig				(wrsig					),
	.tx					(tx1					)
);
 
/******************************************************
INSTANCE ./uart/XR_uart_ctrl.v
******************************************************/  
XR_uart_ctrl 			XR_uart_ctrl_inst 
(
			.clk        		(clk_uart				),                           
			.rdsig     			(rdsig					),              
			.rxdata        	(rxdata					), 		      
			.wrsig  				(wrsig					),              
			.dataout   			(txdata					),
			.send_flag			(send_flag				),
			.send_data			(send_data				)
 );
 
endmodule
