`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    18:00:23 06/23/2016 
// Design Name: 
// Module Name:    XR_motor 
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
module XR_motor
(		 
	 input							  clk_sys
	,input							  rst_n
	,input			[10:0]		speed_a
	,input			[10:0]		speed_b
	,output	reg					  ena
	,output	reg					  enb
	,input			[3:0]			motor_led
	,output	reg	[3:0]			motor
)
 
/*********************************************************************
  register
**********************************************************************/ 
reg 			[10:0] 	ena_cnt;
reg 			[10:0] 	enb_cnt;
wire			[3:0]		motor_out;

assign motor_out = motor_led;        

/*********************************************************************
将motor_led处理的电机控制数据赋值给电机端口
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)
		motor				<=				4'h0;			
	else 
		motor				<=				motor_out;	
end






/*********************************************************************
产生enapwm  23kHz=2170
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
if(!rst_n)begin
	ena					<=				1'b0;
	ena_cnt				<=				11'd0;
end
else begin
  if(speed_a==11'd0)begin
	ena					<=				1'b0;
	ena_cnt				<=				11'd0;
	end
  else if(ena_cnt == speed_a)
  begin
    ena 					<= 			1'b0;
    ena_cnt 			<= 			ena_cnt + 11'd1;
  end
  else if(ena_cnt == 11'd2047)
  begin
    ena 					<= 			1'b1;
    ena_cnt 			<= 			11'd0;
  end
  else
  begin
    ena_cnt 			<= 			ena_cnt + 11'd1;
  end
  end//rst_n
end//always

///产生enbpwm  23kHz=2170


always @(posedge clk_sys or negedge rst_n)   
begin
if(!rst_n)begin
	enb					<=				1'b0;
	enb_cnt				<=				11'd0;
end
else begin
  if(speed_b==11'd0)begin
	enb					<=				1'b0;
	enb_cnt				<=				11'd0;
	end
  else if(enb_cnt == speed_b)
  begin
    enb 					<= 			1'b0;
    enb_cnt 			<= 			enb_cnt + 11'd1;
  end
  else if(enb_cnt == 11'd2047)
  begin
    enb 					<= 			1'b1;
    enb_cnt 			<= 			11'd0;
  end
  else
  begin
    enb_cnt 			<= 			enb_cnt + 11'd1;
  end
  end//rst_n
end//always














endmodule
