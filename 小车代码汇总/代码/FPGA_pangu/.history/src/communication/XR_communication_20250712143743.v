`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    14:02:00 06/23/2016 
// Design Name: 
// Module Name:    XR_communication 
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
module XR_communication
(
	 input				clk
	,input 		[7:0] 	rxdata//接收来自wifi uart的数据
	,output	reg	[3:0]	ctrl_led
	,input		[7:0]	cnt
	,input				rst_n
	,output	reg	[3:0]	ctrl_mode
	,output	reg	[3:0]	motor_ctrl
	,output	reg	[3:0]	send_flag
	,output	reg	[10:0]	speed_a
	,output	reg	[10:0]	speed_b
	,output	reg	[63:0]	servo_temp
	,output	reg	[1:0]	i2c_rw
	,output	reg			HLED
	,input		[63:0]	servo_rd
);

/*********************************************************************
  register
**********************************************************************/ 
reg	[1:0] 				i;
reg [7:0] 	rxbuffer[2:0];

/*********************************************************************
 解析串口通信数据，FF-XX-XX-XX-FF,去掉包头、包尾，取出数据包
**********************************************************************/ 
always @( posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		rxbuffer[0]				<=				8'hFF;
		rxbuffer[1]				<=				8'hFF;
		rxbuffer[2]				<=				8'hFF;
		i						<=				1'b0;	
	end
	else 
	begin
		if (cnt == 8'd152)
		begin
			if(rxdata == 8'hFF)
				i				<=				4'b0000;			
			else begin 
				if(i==2'b10)begin
					rxbuffer[i]		<=				rxdata;
					i				<=				1'b0;	
				end			
				else begin
					rxbuffer[i]		<=				rxdata;
					i				<=				i+1'b1;
				end
			end
		end//cnt
	end//rst_n
end//always

/*********************************************************************
指令处理
**********************************************************************/ 
 always @( posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		ctrl_led					<=				1'b0;
		motor_ctrl				<=				4'b0000;
		speed_a					<=				11'd2041;
		speed_b					<=				11'd2041;
		i2c_rw					<=				2'b10;			//10上电初始化读角度
		ctrl_mode				<=				4'b0000;
		send_flag				<=				4'b0001;
	end
	else begin
	if(i2c_rw[1])
	begin
		servo_temp				<=				servo_rd;
	end
	
	if(rxbuffer[0]==8'h00)//手动控制小车运动
	begin
		i2c_rw					<=				2'b00;
		send_flag				<=				4'b0001;
		//ctrl_mode				<=				4'b0000;
		case(rxbuffer[1])
			8'h01: begin		ctrl_led<=4'b0101;motor_ctrl<=4'b0101;speed_a<=11'd2040;speed_b<=11'd2040;end//前进,速度满速2150
			8'h02: begin		ctrl_led<=4'b1010;motor_ctrl<=4'b1010;speed_a<=11'd2040;speed_b<=11'd2040;end//后退
			8'h03: begin		ctrl_led<=4'b0110;motor_ctrl<=4'b0110;speed_a<=11'd2040;speed_b<=11'd2040;end//左转
			8'h04: begin		ctrl_led<=4'b1001;motor_ctrl<=4'b1001;speed_a<=11'd2040;speed_b<=11'd2040;end//右转			
			8'h00: begin		ctrl_led<=4'b0000;motor_ctrl<=4'b0000;speed_a<=11'd0;speed_b<=11'd0;end//停止			
			8'h05: begin		ctrl_led<=4'b0101;motor_ctrl<=4'b0101;speed_a<=11'd1670;speed_b<=11'd2040;end//左前,速度满速2150
			8'h06: begin		ctrl_led<=4'b1010;motor_ctrl<=4'b1010;speed_a<=11'd1670;speed_b<=11'd2040;end//左后
			8'h07: begin		ctrl_led<=4'b0110;motor_ctrl<=4'b0101;speed_a<=11'd2040;speed_b<=11'd1670;end//右前
			8'h08: begin		ctrl_led<=4'b1001;motor_ctrl<=4'b1010;speed_a<=11'd2040;speed_b<=11'd1670;end//右后			
			default:begin		ctrl_led<=4'b0000;motor_ctrl<=4'b0000;speed_a<=11'd0;speed_b<=11'd0;end//停止		
		endcase
	end
	else if(rxbuffer[0]==8'h01)//手动控制八路舵机
	begin
		i2c_rw					<=				2'b00;
		send_flag				<=				4'b0001;
		if(rxbuffer[2]<8'd180)
		begin
			case(rxbuffer[1])
			8'h01: begin		ctrl_led<=4'b0001;servo_temp[7:0]<=rxbuffer[2];end//舵机1
			8'h02: begin		ctrl_led<=4'b0010;servo_temp[15:8]<=rxbuffer[2];end//舵机2
			8'h03: begin		ctrl_led<=4'b0011;servo_temp[23:16]<=rxbuffer[2];end//舵机3
			8'h04: begin		ctrl_led<=4'b0100;servo_temp[31:24]<=rxbuffer[2];end//舵机4
			8'h05: begin		ctrl_led<=4'b0101;servo_temp[39:32]<=rxbuffer[2];end//舵机5
			8'h06: begin		ctrl_led<=4'b0110;servo_temp[47:40]<=rxbuffer[2];end//舵机6
			8'h07: begin		ctrl_led<=4'b0111;servo_temp[55:48]<=rxbuffer[2];end//云台1
			8'h08: begin		ctrl_led<=4'b1000;servo_temp[63:56]<=rxbuffer[2];end//云台2
			default:begin ctrl_led<=4'b0000;end
			endcase
		end
	end
	else if(rxbuffer[0]==8'h32)//存储角度
	begin
		i2c_rw					<=				2'b01;			//01写角度
		send_flag				<=				4'b0001;
	end
	else if(rxbuffer[0]==8'h33)//读取并归位角度
	begin
		i2c_rw					<=				2'b10;			//10读角度
		send_flag				<=				4'b0001;
	end
	else if(rxbuffer[0]==8'h13)//模式切换开关
	begin
		i2c_rw					<=				2'b00;
		speed_a					<=				11'd2040;
		speed_b					<=				11'd2040;
		case(rxbuffer[1])
//	0000 手动模式；		0001跟随模式；		0010红外巡线模式；
//	0011红外壁障模式；	0100雷达模式；		0101机械臂演示；
//	0110摇头演示；		0111超声波摇头壁障；
			8'h01: begin		ctrl_mode	<=		4'b0001;send_flag	<=	4'b0001;			end//跟随模式
			8'h02: begin		ctrl_mode	<=		4'b0010;send_flag	<=	4'b0001;			end//红外巡线模式
			8'h03: begin		ctrl_mode	<=		4'b0011;send_flag	<=	4'b0001;			end//红外壁障模式
			8'h04: begin		ctrl_mode	<=		4'b0100;send_flag	<=	4'b1001;	end//雷达模式
			8'h05: begin		ctrl_mode	<=		4'b0101;send_flag	<=	4'b0001;			end//机械臂演示
			8'h06: begin		ctrl_mode	<=		4'b0110;send_flag	<=	4'b0001;			end//摇头演示
			8'h07: begin		ctrl_mode	<=		4'b0111;send_flag	<=	4'b1001;			end//超声波摇头壁障
			8'h08: begin		ctrl_mode	<=		4'b1000;send_flag	<=	4'b0001;			end//待定
			default:begin 		ctrl_mode <= 4'b0000;motor_ctrl<=4'b0000;speed_a<=11'd0;speed_b<=11'd0;	end//手动模式
		 endcase
	end	
	else if(rxbuffer[0]==8'h05)//大灯开关
	begin
		i2c_rw					<=				2'b00;
		send_flag				<=				4'b0001;
		case(rxbuffer[1])
//	00 开灯；		01关灯

			8'h00: begin		HLED						<=				1'b1;			end
			8'h01: begin		HLED						<=				1'b0;			end
		 endcase
	end
 end//rst
end//always















endmodule
