`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:27:05 08/02/2016 
// Design Name: 
// Module Name:    XR_ultrasonic 
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
module XR_ultrasonic
(
	 input						clk_sys
	,input						rst_n
	,output	reg	[3:0]			motor_Wave_Avoidance			//超声波摇头壁障模式，走迷宫
	,output	reg	[3:0]			motor_AvoidByRadar				//超声波壁障模式
	,input						USE
	,output						UST
	,input		[3:0]			motor_ctrl
	,input  	[7:0]		    servo7_angle
	//,output	[3:0]			send_flag
	,output		[23:0]			send_data
 );
 
/**********************************************************************************
register
***********************************************************************************/
reg	[2:0]		US_TE;			//00 发射低电平2us；01发射高电平10us；10持续发射低电平直到计算出回传时间。
reg	[23:0]		cnt;			//计数器，49为1us，
reg	[12:0]		Dis_cnt;
reg	[12:0]		Dis_mm;
reg	[23:0]		Distance;
reg				Trig_Done;
reg				Echo_Done;

reg				trig;
wire			echo;

assign			UST		=	trig;
assign			echo	=	USE;

//assign		send_flag	=	4'b1001;
assign			send_data	=	Distance;

/******************************************************
分频进程,对50Mhz的时钟326分频
******************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		Dis_cnt		<=			13'd0;
		Dis_mm		<=			13'd0;
	end
	else if(!Trig_Done)
	begin
		Dis_cnt		<=			13'd0;
		Dis_mm		<=			13'd0;
	end
	else if(Trig_Done)	
	begin
		if(Dis_cnt == 13'd294)
		begin
			Dis_mm		<=		Dis_mm + 1'b1;
			Dis_cnt 	<= 		1'b0;
		end
		else
		begin
			Dis_cnt 	<= 		Dis_cnt + 1'b1;
		end
	end//rst_n
end//always

/**********************************************************************************
超声波发射，等待回声计算距离完毕之后延时一段时间继续发射
发射2us低电平
发射10us高电平（至少10us）
持续发射低电平
计算echo高电平到来时间差，并计算出距离
***********************************************************************************/
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)
	begin
		Distance			<=			8'd0;
		Trig_Done			<=			1'b0;
		Echo_Done			<=			1'b0;
		cnt					<=			1'b0;
		US_TE				<=			3'b000;
		trig				<= 			1'b0;
	end
	else if(US_TE[2])
	begin
		if(cnt == 24'd650000)
		begin
			cnt				<=			1'b0;
			US_TE			<=			3'b000;
		end
		else
		begin
			cnt				<=			cnt+1'b1;	
		end
	end
	else
	begin
		if (US_TE == 3'b000)
		begin
			if(cnt == 18'd100)
			begin
				trig 		<= 			1'b1;
				cnt			<=			1'b0;
				US_TE		<=			2'b01;
			end
			else
			begin
				cnt			<=			cnt+1'b1;	
			end
		end
		else if(US_TE == 3'b001)
		begin
			if(cnt == 18'd500)
			begin
				trig 		<= 			1'b0;
				cnt			<=			1'b0;
				US_TE		<=			2'b10;
			end
			else
			begin
				cnt			<=			cnt+1'b1;	
			end
		end
		else if(US_TE == 3'b010)
		begin
			if(cnt == 18'd28800)
			begin
				cnt			<=				1'b0;
				US_TE		<=				2'b11;
				Trig_Done	<=				1'b1;
			end
			else
			begin
				cnt			<=				cnt+1'b1;	
			end
		end
		else if(US_TE == 3'b011)
		begin
			
			if(Dis_mm == 13'd500)
			begin
				US_TE		<=				3'b100;
				Distance	<=				24'hEEEEEE;
				Trig_Done	<=				1'b0;
			end
			else if(!echo)
			begin
				US_TE		<=				3'b100;
				Distance	<=				Dis_mm + 40;
				Trig_Done	<=				1'b0;
			end
		end	
	end//if rst	
end//always

/***************************************************************************

***************************************************************************/
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)
		motor_AvoidByRadar	<=	4'b0000;			
	else if (Distance < 200)
	begin
		motor_AvoidByRadar	<=	4'b0000;	//停止
	end
	else begin
		motor_AvoidByRadar	<=	4'b1010;	//前进
	end	
end

if (ctrl_mode == 4'b1000) begin//雷达模式
	
end



endmodule
