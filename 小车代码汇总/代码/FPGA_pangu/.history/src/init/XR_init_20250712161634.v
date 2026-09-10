`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    13:23:56 06/29/2016 
// Design Name: 
// Module Name:    XR_init 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 上电等待wifi初始化完成，大概35-40s，初始化期间跑流水灯。
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_init
(
	 input 					clk_sys				//系统时钟	
	,input					rst_n
	,output	reg				init_flag			//初始化完成标志
	,output	reg	[3:0]		init_led
);

/******************************************************
register 
******************************************************/  
reg [11:0] 		cnt;
reg [14:0] 		timer;
reg	[5:0]		init_cnt;
reg				init_clk;

/******************************************************
分频进程,对50Mhz的时钟FFF分频,输出init_clk
******************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		init_clk 		<= 		1'b0;
		cnt				<=		12'h0;
	end
	else 
	begin
		case(cnt)
		12'h7FF: 
		begin
			init_clk 	<= 		1'b1;
			cnt 		<= 		cnt + 12'h1;
		end
		12'hFFF:		
		begin
			init_clk 	<= 		1'b0;
			cnt 		<= 		12'h0;
		end
		default:
		begin
			cnt 		<= 		cnt + 12'h1;
		end
		endcase
	end//rst_n
end//always



/******************************************************
		1s的计数器为12207
******************************************************/ 
always @(posedge init_clk or negedge rst_n)   
begin
	if(!rst_n)
		timer			<=		15'd0;
	else if (timer == 15'd12207)
		timer			<=		15'd0;
	else 
		timer			<=		timer+15'd1;
end//always


/******************************************************
		流水灯
******************************************************/ 
always @(posedge init_clk or negedge rst_n)
begin
	if (~rst_n)
	begin
		init_led 		<= 	4'b0000;
		init_flag		<=		1'b0;
		init_cnt			<=		5'b00000;
		end
	else if(!init_flag)
	begin
		if	(init_cnt==40)
		init_flag		<=		1'b1;
		else 
		begin
			case(timer)
			15'd2034:
				init_led 	<= 	4'b0001;
			15'd4068:
				init_led 	<= 	4'b0010;
			15'd6102:
				init_led 	<= 	4'b0100;	
			15'd8137:
				init_led	 	<= 	4'b1000;
			15'd10172:
				init_led 	<= 	4'b0100;
			15'd12207:
			begin
				init_led 	<= 	4'b0010;
				init_cnt		<=		init_cnt+1'b1;	
			end		
			endcase 
		end
	end
end

endmodule
