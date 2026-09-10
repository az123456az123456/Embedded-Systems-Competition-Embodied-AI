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
	//,output			[3:0]			send_flag
	,output			[23:0]		send_data
	//sensor port
	,input			[2:0]		IR		//0红外中，1红外左，2红外右
	,input						USE		//超声波echo
	,output						UST		//超声波TRIG		
);

wire		[3:0]			motor_led;				//电机及LED的输出寄存器	
//wire		[3:0]			motor_ctrl;				//手动控制电机
wire		[3:0]			motor_Follow_Track;	//跟随模式
wire		[3:0]			motor_Follow_Line;	//巡线模式
wire		[3:0]			motor_Avoiding;		//红外壁障模式
wire		[3:0]			motor_AvoidByRadar;	//超声波壁障模式
wire		[3:0]			motor_Wave_Avoidance;//超声波摇头壁障模式，走迷宫






/******************************************************
INSTANCE ./function/XR_motor.v
******************************************************/  
XR_motor_led_process 	XR_motor_led_process_inst 
(
		.clk_sys						(clk_sys					),
		.rst_n						(rst_n					),
		.motor_led					(motor_led				),
		.ctrl_mode					(ctrl_mode				),
		.motor_ctrl					(motor_ctrl				),
		.motor_Follow_Track		(motor_Follow_Track	),
		.motor_Follow_Line		(motor_Follow_Line	),
		.motor_Avoiding			(motor_Avoiding		),
		.motor_AvoidByRadar		(motor_AvoidByRadar	),
		.motor_Wave_Avoidance	(motor_Wave_Avoidance)

 );





/******************************************************
INSTANCE ./function/XR_motor.v
******************************************************/  
XR_motor XR_motor_inst 
(
		.clk_sys						(clk_sys			),
		.rst_n						(rst_n			),
		.speed_a						(speed_a			),
		.speed_b						(speed_b			),
		.ena							(ena				),
		.enb							(enb				),
		.motor_led					(motor_led		),
		.motor						(motor			)
 );
 
 
 
/******************************************************
INSTANCE ./function/XR_servo.v
******************************************************/   
XR_servo XR_servo_inst 
(
		.clk_sys						(clk_sys			),
		.rst_n						(rst_n			),
		.servo_temp					(servo_temp		),
		.servo						(servo			),
		.i2c_rw						(i2c_rw			),
		.servo_rd					(servo_rd		)
 );
 
 
/******************************************************
INSTANCE ./function/XR_led.v
******************************************************/  
XR_led XR_led_inst 
(
		.clk_sys						(clk_sys			),
		.rst_n						(rst_n			),
		.ctrl_led					(ctrl_led		),
		.init_flag					(init_flag		),
		.init_led					(init_led		),
		.motor_led					(motor_led		),
		.led							(led				)
 );


 
/******************************************************
INSTANCE ./function/XR_sensor.v
******************************************************/  
XR_sensor XR_sensor_inst 
(


 );
 
 
/******************************************************
INSTANCE ./function/XR_IR.v
******************************************************/  
XR_IR XR_IR_inst 
(
		.clk_sys						(clk_sys					),
		.rst_n						(rst_n					),
		.motor_Follow_Line		(motor_Follow_Line	),
		.motor_Avoiding			(motor_Avoiding		),
		.IR							(IR						)


 ); 
 
 
/******************************************************
INSTANCE ./function/ultrasonic/XR_ultrasonic.v
******************************************************/  
XR_ultrasonic  XR_ultrasonic_inst 
(
		.clk_sys						(clk_sys						),
		.rst_n						(rst_n						),
		.motor_AvoidByRadar		(motor_AvoidByRadar		),
		.motor_Wave_Avoidance	(motor_Wave_Avoidance	),
		.USE							(USE							),
		.UST							(UST							),
		//.send_flag					(send_flag					),
		.send_data					(send_data					)
 ); 

 
/******************************************************
INSTANCE ./function/XR_send_data.v
******************************************************/  
XR_send_data 	XR_send_data_inst 
(
		//.send_flag				(send_flag		),
		//.send_data				(send_data		)

 );


 
 
endmodule
