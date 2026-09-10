`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    18:00:40 06/23/2016 
// Design Name: 
// Module Name:    XR_servo 
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
module XR_servo
(
	 input					clk_sys
	,input					rst_n
	,input		[63:0]		servo_temp
	,output	reg	[7:0]		servo
	,input		[1:0]		i2c_rw
	,input		[63:0]		servo_rd
	
);

/*********************************************************************
register
**********************************************************************/ 
reg 	[19:0] 	ser_cnt;
wire	[63:0]	servo_out;

assign servo_out 				= 			i2c_rw[1] ? servo_rd : servo_temp;        //SDA数据输出选择

/*********************************************************************
舵机0-200度为0.5ms-2.5ms，生成周期20ms(20'd999_999)，
2.5%-12.5%占空比对应0.5-2.5ms,对应计数是24_999+555*servo_out
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)
		ser_cnt				<=				20'd0;		
	else if(ser_cnt == 20'd999_999)
		ser_cnt				<=				20'd0;
	else
		ser_cnt				<=				ser_cnt+20'd1;	
end

/*********************************************************************
产生ser1的pwm  servo[0] servo_out[7:0] 
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[0] <=	8'b0;
	end
	else begin
		if(ser_cnt == servo_out[7:0]*555+20'd24_999)
		begin
			servo[0] <= 1'b0;
		end	
		else if(ser_cnt == 20'd999_991)
		begin
			servo[0] <= 1'b1;
		end
	end//rst_n
end//always

/*********************************************************************
产生ser2的pwm  servo[1] servo_out[15:8] 
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[1] <=	8'b0;
	end
	else begin
		if(ser_cnt == servo_out[15:8]*555+20'd24_999)
		begin
			servo[1] <= 1'b0;
		end
		else if(ser_cnt == 20'd999_992)
		begin
			servo[1] <= 1'b1;
		end
	end//rst_n
end//always

/*********************************************************************
产生ser3的pwm  servo[2] servo_out[23:16]  
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[2] <=	8'b0;
	end
	else begin
		if(ser_cnt == servo_out[23:16]*555+20'd24_999)
		begin
			servo[2] <= 1'b0;
		end
		else if(ser_cnt == 20'd999_993)
		begin
			servo[2] <= 1'b1;
		end
	end//rst_n
end//always




/*********************************************************************
产生ser4的pwm  servo[3] servo_out[31:24]
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[3]				<=				8'b0;
	end
	else begin
		if(ser_cnt == servo_out[31:24]*555+20'd24_999)
		begin
			servo[3] 		<= 			1'b0;
		end
		else if(ser_cnt == 20'd999_994)
		begin
			servo[3] 		<= 			1'b1;
		end
	end//rst_n
end//always



/*********************************************************************
产生ser5的pwm  servo[4] servo_out[39:32]
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[4]				<=				8'b0;
	end
	else begin
		if(ser_cnt == servo_out[39:32]*555+20'd24_999)
		begin
			servo[4] 		<= 			1'b0;
		end
		else if(ser_cnt == 20'd999_995)
		begin
			servo[4] 		<= 			1'b1;
		end
	end//rst_n
end//always




/*********************************************************************
产生ser6的pwm  servo[5] servo_out[47:40]  
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[5]				<=				8'b0;
	end
	else begin
		if(ser_cnt == servo_out[47:40]*555+20'd24_999)
		begin
			servo[5] 		<= 			1'b0;
		end
		else if(ser_cnt == 20'd999_996)
		begin
			servo[5] 		<= 			1'b1;
		end
	end//rst_n
end//always






/*********************************************************************
产生ser7的pwm  servo[6] servo_out[55:48] 
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[6]				<=				8'b0;
	end
	else begin
		if(ser_cnt == servo_out[55:48]*555+20'd24_999)
		begin
			servo[6] 		<= 			1'b0;
		end
		else if(ser_cnt == 20'd999_997)
		begin
			servo[6] 		<= 			1'b1;
		end
	end//rst_n
end//always





/*********************************************************************
产生ser8的pwm  servo[7] servo_out[63:56]
**********************************************************************/ 
always @(posedge clk_sys or negedge rst_n)   
begin
	if(!rst_n)begin
		servo[7]				<=				8'b0;
	end
	else begin
		if(ser_cnt == servo_out[63:56]*555+20'd24_999)
		begin
			servo[7] 		<= 			1'b0;
		end
		else if(ser_cnt == 20'd999_998)
		begin
			servo[7] 		<= 			1'b1;
		end
	end//rst_n
end//always


endmodule
