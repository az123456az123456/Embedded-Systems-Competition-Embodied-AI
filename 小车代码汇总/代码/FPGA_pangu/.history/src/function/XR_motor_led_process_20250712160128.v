`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:49:25 07/31/2016 
// Design Name: 
// Module Name:    XR_motor_led_process 
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
module XR_motor_led_process
(
	 input							clk_sys
	,input							rst_n
	,output			[3:0]			motor_led					//电机及LED的输出寄存器	
	,input			[3:0]			ctrl_mode					//小车控制模式				
	,input			[3:0]			motor_ctrl					//手动控制电机
	,input			[3:0]			motor_Follow_Track			//跟随模式
	,input			[3:0]			motor_Follow_Line			//巡线模式
	,input			[3:0]			motor_Avoiding				//红外壁障模式
	,input			[3:0]			motor_AvoidByRadar			//超声波壁障模式
	,input			[3:0]			motor_Wave_Avoidance		//超声波摇头壁障模式，走迷宫
);
/******************************************************************************
register
******************************************************************************/
reg	[3:0]		motor_led_out;

/******************************************************************************

******************************************************************************/
assign motor_led = motor_led_out;       

/******************************************************************************

******************************************************************************/
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n )
		motor_led_out				<=					4'b0000;			
	else begin
	case	(ctrl_mode)
			4'b0000:begin		motor_led_out			<=				motor_ctrl;					end
			4'b0001:begin		motor_led_out			<=				motor_Follow_Track;		end
			4'b0010:begin		motor_led_out			<=				motor_Follow_Line;		end
			4'b0011:begin		motor_led_out			<=				motor_Avoiding;			end
			4'b0100:begin		motor_led_out			<=				motor_AvoidByRadar;		end
			4'b0111:begin		motor_led_out			<=				motor_Wave_Avoidance;	end
	endcase
	end
		
end




















endmodule
