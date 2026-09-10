`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    10:55:51 06/23/2016 
// Design Name: 
// Module Name:    XR_function 
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
module XR_function
(
	 input						clk_sys
	,input						rst_n
	,input			[10:0]		speed_a
	,input			[10:0]		speed_b
	,output						ena
	,output						enb
	,input			[63:0]		servo_temp
	,output			[7:0]		servo
	,input			[1:0]		i2c_rw
	,input			[63:0]		servo_rd
	,input			[3:0]		ctrl_mode
	,input			[3:0]		motor_ctrl
	,output			[3:0]		motor
	,input			[3:0]		ctrl_led
	,input			[3:0]		init_led
	,input						init_flag
	,output			[3:0]		led	
	//,output		[3:0]		send_flag
	,output			[23:0]		send_data
	//sensor port
	,input			[2:0]		IR					//0红外中，1红外左，2红外右
	,input						USE					//超声波echo
	,output						UST					//超声波TRIG		
);

wire		[3:0]			motor_led;				//电机及LED的输出寄存器	
//wire		[3:0]			motor_ctrl;				//手动控制电机
wire		[3:0]			motor_Follow_Track;		//跟随模式
wire		[3:0]			motor_Follow_Line;		//巡线模式
wire		[3:0]			motor_Avoiding;			//红外壁障模式
wire		[3:0]			motor_AvoidByRadar;		//超声波壁障模式
wire		[3:0]			motor_Wave_Avoidance;	//超声波摇头壁障模式，走迷宫
wire 		[7:0]			servo7_angle;  // 新增输出端口，表示7号舵机角度
reg	[23:0]			Distance;
/******************************************************
小车运动模式led灯参数切换模块
******************************************************/  
XR_motor_led_process 	XR_motor_led_process_inst 
(
	.clk_sys				(clk_sys				),
	.rst_n					(rst_n					),
	.motor_led				(motor_led				),
	.ctrl_mode				(ctrl_mode				),
	.motor_ctrl				(motor_ctrl				),
	.motor_Follow_Track		(motor_Follow_Track		),
	.motor_Follow_Line		(motor_Follow_Line		),
	.motor_Avoiding			(motor_Avoiding			),
	.motor_AvoidByRadar		(motor_AvoidByRadar		),
	.motor_Wave_Avoidance	(motor_Wave_Avoidance	)
);

/******************************************************
电机驱动模块
******************************************************/  
XR_motor XR_motor_inst 
(
	.clk_sys				(clk_sys			),
	.rst_n					(rst_n				),
	.speed_a				(speed_a			),
	.speed_b				(speed_b			),
	.ena					(ena				),
	.enb					(enb				),
	.motor_led				(motor_led			),
	.motor					(motor				)
 );
 
/******************************************************
舵机控制模块
******************************************************/   
XR_servo XR_servo_inst 
(
	.clk_sys				(clk_sys		),
	.rst_n					(rst_n			),
	.servo_temp				(servo_temp		),
	.servo					(servo			),
	.i2c_rw					(i2c_rw			),
	.servo_rd				(servo_rd		),
	.servo7_angle			(servo7_angle	) // 新增输出端口，表示7号舵机角度
 );
 
/******************************************************
led灯模块
******************************************************/  
XR_led XR_led_inst 
(
	.clk_sys				(clk_sys		),
	.rst_n					(rst_n			),
	.ctrl_led				(ctrl_led		),
	.init_flag				(init_flag		),
	.init_led				(init_led		),
	.motor_led				(motor_led		),
	.led					(led			)
);
 
/******************************************************
传感器预留模块
******************************************************/  
XR_sensor XR_sensor_inst 
(


 );
 
/******************************************************
INSTANCE ./function/XR_IR.v
******************************************************/  
XR_IR XR_IR_inst 
(
	.clk_sys				(clk_sys			),
	.rst_n					(rst_n				),
	.motor_Follow_Line		(motor_Follow_Line	),
	.motor_Avoiding			(motor_Avoiding		),
	.IR						(IR					)
);
 
/******************************************************
超声波测距模块
******************************************************/  
XR_ultrasonic XR_ultrasonic_inst 
(
	.clk_sys				(clk_sys				),
	.rst_n					(rst_n					),
	.motor_AvoidByRadar		(motor_AvoidByRadar		),
	.motor_Wave_Avoidance	(motor_Wave_Avoidance	),
	.USE					(USE					),
	.UST					(UST					),
	.ctrl_mode				(ctrl_mode				),
	.servo7_angle  			(servo7_angle			),// 新增输入端口，表示7号舵机角度
	.Distance				(Distance				),
	//.send_flag			(send_flag				),
	.send_data				(send_data				)
); 

/******************************************************
空模块
******************************************************/  
XR_send_data 	XR_send_data_inst 
(
	//.send_flag				(send_flag		),
	//.send_data				(send_data		)
);
 
endmodule
