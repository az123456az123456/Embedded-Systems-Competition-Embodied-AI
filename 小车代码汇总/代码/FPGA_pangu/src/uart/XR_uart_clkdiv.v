`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    	14:10:21 06/23/2016 
// Design Name: 	 	XR_FPGA
// Module Name:    	XR_uart_clkdiv 
// Project Name: 
// Target Devices: 	XR_FPGA
// Tool versions: 	ISE 14.7
// Description: 		产生一个波特率9600的16倍频的时钟，9600*16= 153600
// 						相当于50MHz的326分频，50000000/153600=326
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_uart_clkdiv
(
	 input 						clk_sys			//系统时钟	
	,input						rst_n
	,output	reg				clkout			//采样时钟输出
);




/******************************************************
   register 
******************************************************/  

reg [8:0] cnt;







/******************************************************
分频进程,对50Mhz的时钟326分频
******************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		clkout 		<= 		1'b0;
		cnt			<=			9'd0;
	end
	else 
	begin
		case(cnt)
		9'd162: 
		begin
			clkout 	<= 		1'b1;
			cnt 		<= 		cnt + 9'd1;
		end
		9'd325: 		
		begin
			clkout 	<= 		1'b0;
			cnt 		<= 		9'd0;
		end
		default:
		begin
			cnt 		<= 		cnt + 9'd1;
		end
		endcase
	end//rst_n
end//always






















endmodule
