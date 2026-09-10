`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:28:13 08/02/2016 
// Design Name: 
// Module Name:    XR_IR 
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
module XR_IR
(
		 input							clk_sys
		,input							rst_n
		,output	reg	[3:0]			motor_Follow_Line			//巡线模式
		,output	reg	[3:0]			motor_Avoiding				//红外壁障模式
		,input			[2:0]			IR


);
/********************************************************************

	,input		[2:0]		IR		//0红外中，1红外左，2红外右  高电平为有反射
	红外壁障模式，检测到IR[0]为高电平，停止，否则前进，也可用于正常模式中的壁障
	巡线模式，正常地面有反射，为高，检测到黑线时，输出为低。哪边输出低，就往该方向转动。
	
	
	红外壁障模式
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)
		motor_Avoiding				<=				4'b0000;			
	else if (!IR[0])
	begin
		motor_Avoiding				<=				4'b0000;						//停止
	end
	else begin
		motor_Avoiding				<=				4'b1010;						//前进
	end
	
end







always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)
		motor_Follow_Line				<=		4'h0000;	
	else begin
		case(IR[2:1])
		2'b00:begin	motor_Follow_Line	<=		4'b0101;	end
		2'b01:begin	motor_Follow_Line	<=		4'b0110;	end
		2'b10:begin	motor_Follow_Line	<=		4'b1001;	end
		2'b11:begin	motor_Follow_Line	<=		4'b0000;	end
		endcase
	end
end




endmodule
