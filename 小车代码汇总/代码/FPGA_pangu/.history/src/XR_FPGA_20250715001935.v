`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    10:33:50 06/23/2016 
// Design Name: 
// Module Name:    XR_FPGA 
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
module XR_FPGA(
	input					clk_sys, //系统时钟，50MHz
	input 					rst_n,

	//UART port
	input 					rx,
	output 					tx,
	input 					rx0,
	output 					tx0,

	//I2C port
	output					scl,
	inout					sda,
	output					scl_hand,
	inout					sda_hand,
	//function port
	output		[3:0]		led,
	output		[3:0]		motor,  //电机控制端口
	output					ena,
	output					enb,
	output		[7:0]		servo,  //舵机接口

	//sensor port
	input		[2:0]		IR,		//00红外中，01红外左，10红外右
	input					USE,	//超声波echo
	output					UST,	//超声波TRIG
	output					HLED,	//大灯
	
	//SMG
	output 	    [7:0]		SMG_Data,
	output 		[5:0]		Scan_Sig
);
	 

/******************************************************
   net 
******************************************************/  	 
wire						clk_uart;	//串口时钟，9600*16Hz
wire		[7:0]			cnt;		//RX接收计数器，作为每字节的判定标准
wire 		[7:0] 			txdata;		//串口准备发送的数据（7位）
wire		[7:0]			rxdata;     //串口发送数据和串口接收数据
wire		[10:0]			speed_a;	//左侧速度
wire		[10:0]			speed_b;	//右侧速度
wire 		[63:0]			servo_temp;	//串口接收到的舵机角度值
wire 		[63:0]			servo_rd;	//eeprom读取到的舵机角度值
//wire 		[7:0]			servo7_angle; // 新增输出端口，表示7号舵机角度
wire 		[1:0]			i2c_rw;		//iic读写标志，2'b10读，2'b01写
wire 						init_flag;	//上电等待wifi初始化40s标志位，高代表完成

wire		[3:0]			ctrl_led;	//串口接收的led状态
wire		[3:0]			init_led;	//初始化时流水灯状态

wire		[3:0]			ctrl_mode;	//小车模式标志位：
wire		[3:0]			motor_ctrl;	//小车模式标志位：

//0000 手动模式；		0001跟随模式；		0010红外巡线模式；
//0011红外壁障模式；	0100雷达模式；		0101机械臂演示；
//0110摇头演示；		0111超声波摇头壁障；

/****************************************************************
数据上传，send_flag为上传标志位，1为上传sensor数据
send_data为3个字节的上传数据，指令协议：FF-XX-XX-XX-FF。去掉包头包尾的FF。
****************************************************************/
wire		[3:0]			send_flag;	//数据上传标志位 3:上传标志, 2:uart2, 1:uart1, 0:uart0 
wire		[23:0]			send_data;	//24位上传数据

/******************************************************
INSTANCE ./init/XR_init.v   上电初始化程序
******************************************************/  
XR_init XR_init_inst 
(
	.clk_sys				(clk_sys		),
	.rst_n					(rst_n			),
	.init_flag				(init_flag		),
	.init_led				(init_led		)
);

/******************************************************
串口通信，接收uart数据，传递给communicate模块
******************************************************/  
XR_uart XR_uart_inst 
(
	.clk_sys				(clk_sys		),
	.rx						(rx				),
	.tx						(tx				),
	.rx0					(rx0			),
	.tx0					(tx0			),
	.rx1					(				),
	.tx1					(				),
	.rst_n					(rst_n			),
	.rxdata					(rxdata			),
	.cnt					(cnt			),
	.clk_uart				(clk_uart		),
	.send_flag				(send_flag		),
	.send_data				(send_data		),
	.init_flag				(init_flag		)		
);
 
/******************************************************
INSTANCE ./iic/XR_iic.v	    I2C通信
******************************************************/   
XR_iic XR_iic_inst 
(
	.clk_sys				(clk_sys		),
	.rst_n					(rst_n			),
	.scl					(scl			),
	.sda					(sda			),
	.i2c_rw					(i2c_rw			),
	.servo_temp				(servo_temp		),
	.servo_rd				(servo_rd		)
);
 
/******************************************************
INSTANCE ./communication/XR_communication.v	
******************************************************/  
XR_communication XR_communication_inst 
(
	.clk					(clk_uart		),
	.rxdata					(rxdata			),
	.ctrl_led				(ctrl_led		),
	.cnt					(cnt			),
	.rst_n					(rst_n			),
	.ctrl_mode				(ctrl_mode		),
	.send_flag				(send_flag		),
	.motor_ctrl				(motor_ctrl		),
	.speed_a				(speed_a		),
	.speed_b				(speed_b		),
	.servo_temp				(servo_temp		),
	.i2c_rw					(i2c_rw			),
	.HLED					(HLED			),
	.servo_rd				(servo_rd		)
);
 
/******************************************************
INSTANCE ./smg_interface/XR_smg_interface.v	数码管
******************************************************/  
wire 		[7:0]			servo7_angle;  // 新增输出端口，表示7号舵机角度
wire		[23:0]			Distance;
XR_smg_interface XR_smg_interface_inst 
(
	.CLK					(clk_sys		),
	.RSTn					(rst_n			),		
	.SMG_Data				(SMG_Data		),
	.Scan_Sig				(Scan_Sig		),
	.servo7_angle			(servo7_angle	), // 新增输出端口，表示7号舵机角度
	.Distance				(Distance		), // 新增输出端口，表示距离
	.send_data				(send_data		)
); 
 
/******************************************************
INSTANCE ./function/XR_function.v	
******************************************************/ 
XR_function XR_function_inst 
(
	.clk_sys				(clk_sys		),
	.rst_n					(rst_n			),
	.speed_a				(speed_a		),
	.speed_b				(speed_b		),
	.ena					(ena			),
	.enb					(enb			),
	.servo_temp				(servo_temp		),
	.servo					(servo			),
	.i2c_rw					(i2c_rw			),
	.servo_rd				(servo_rd		),
	//.servo7_angle			(servo7_angle	), // 新增输出端口，表示7号舵机角度
	.ctrl_mode				(ctrl_mode		),
	.motor_ctrl				(motor_ctrl		),
	.motor					(motor			),
	.ctrl_led				(ctrl_led		),
	.init_flag				(init_flag		),
	.init_led				(init_led		),
	.led					(led			),
	//.send_flag			(send_flag		),
	.send_data				(send_data		),
	.servo7_angle			(servo7_angle	), // 新增输出端口，表示7号舵机角度
	.Distance				(Distance		), 
	.IR						(IR				),
	.USE					(USE			),
	.UST					(UST			),
	.hand_data				(hand_data		) // 手势控制数据
);

    // input wire clk ,
    // input wire rst_n ,
    // output wire scl ,
    // output reg [3:0] led ,
    // inout wire sda ,
    // output wire [5:0] sel , // 片选
    // output reg [7:0] dig , // 段选
    // output wire buzzer
wire [3:0] hand_data;
// 上：0001
// 下：0010
// 左：0100
// 右：1000

prj7620_top u_prj7620_top
(
    .clk(clk_sys),
    .rst_n(rst_n),
    .scl(scl_hand),
    .led(hand_data),
    .sda(sda_hand),
    .sel(), // 片选
    .dig(), // 段选
    .buzzer()
);
 
endmodule
