`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    13:57:31 06/23/2016 
// Design Name: 
// Module Name:    XR_iic 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 		iic通信，包括iic读写、读写控制
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_iic
(	 
	 input				clk_sys
	,input 				rst_n
	,input	[1:0]		i2c_rw
	,output				scl
	,inout				sda
	//,output	[7:0]		RdData
	,input	[63:0]	servo_temp
	,output	[63:0]	servo_rd

);



/******************************************************
  net 
******************************************************/  
wire 	[1:0]		Start_Sig;
wire 	[7:0]		Addr_Sig;
wire 	[7:0]		WrData;
wire	[7:0]		RdData;





/******************************************************
INSTANCE ./iic/XR_iic_ctrl.v	
******************************************************/   
XR_iic_ctrl XR_iic_ctrl_inst (
		.clk_sys						(clk_sys			),
		.rst_n						(rst_n			),
		.Start_Sig            	(Start_Sig		),//read or write command
		.Addr_Sig              	(Addr_Sig		),//eeprom words address
		.WrData                	(WrData			),//eeprom write data
		.RdData               	(RdData			),//eeprom read data
		.Done_Sig               (Done_Sig		),   //eeprom read/write finish
		.i2c_rw						(i2c_rw			),
		.servo_temp					(servo_temp		),
		.servo_rd					(servo_rd		)

 );
 
 
 
 
/******************************************************
INSTANCE ./iic/XR_iic_com.v	
******************************************************/   
XR_iic_com 		XR_iic_com_inst (
	   .clk_sys						(clk_sys			),
		.rst_n						(rst_n			),
		.Start_Sig            	(Start_Sig		),//read or write command
		.Addr_Sig              	(Addr_Sig		),//eeprom words address
		.WrData                	(WrData			),//eeprom write data
		.RdData               	(RdData			),//eeprom read data
		.Done_Sig               (Done_Sig		),   //eeprom read/write finish
		.scl							(scl				),
		.sda							(sda				)
 );

















endmodule
