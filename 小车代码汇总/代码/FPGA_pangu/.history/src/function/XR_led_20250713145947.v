`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    15:03:43 06/29/2016 
// Design Name: 
// Module Name:    XR_led 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_led
(
		 input							clk_sys
		,input							rst_n
		,input			[3:0]			ctrl_led
		,input			[3:0]			init_led
		,input							init_flag
		,input			[3:0]			motor_led
		,output	reg	[3:0]				led
  );
  
/*********************************************************************
register
**********************************************************************/ 
wire 	[3:0] 	led_out;



//assign led_out 				= 			(!init_flag) ? init_led : ctrl_led;        //LED数据输出选择
assign led_out 				= 			(!init_flag) ? init_led : motor_led;        //LED数据输出选择


always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)
		led				<=				4'h0;			
	else 
		led				<=				led_out;	
end
endmodule
