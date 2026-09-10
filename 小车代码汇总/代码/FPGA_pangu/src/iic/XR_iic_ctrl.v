`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    22:12:58 06/24/2016 
// Design Name: 
// Module Name:    XR_iic_ctrl 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 		读写控制，	01写，写下8路舵机数据
//										10读，将eeprom数据读出
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_iic_ctrl(
	    input 							clk_sys
		,input 							rst_n
		,output	reg	[1:0] 		Start_Sig            	//read or write command
		,output	reg	[7:0] 		Addr_Sig              	//eeprom words address
		,output	reg	[7:0] 		WrData                	//eeprom write data
		,input 		 	[7:0] 		RdData               //eeprom read data
		,input 							Done_Sig                   //eeprom read/write finish
		,input			[1:0]			i2c_rw
		,input			[63:0]		servo_temp
		,output	reg	[63:0]		servo_rd
		//,output	reg					servo_rflag				//读取完成标志位
		

    );


/******************************************************
  register
******************************************************/ 
reg 		[3:0] 		i;
reg						write_flag;
reg						read_flag;





/******************************************************
   EEPROM write and read 
******************************************************/	  
always @ ( posedge clk_sys or negedge rst_n )	
	 if( !rst_n ) begin
			i 							<= 			4'd0;
			Addr_Sig 				<= 			8'd0;
			WrData 					<= 			8'd0;
			Start_Sig 				<= 			2'b00;
			//servo_rflag				<=				1'b0;
			write_flag				<=				1'b1;
			read_flag				<=				1'b1;
			
	 end
	 else begin
	 
	 if (i2c_rw[0]==0)write_flag<=1'b1;
	 if (i2c_rw[1]==0)read_flag <=1'b1;
	 
/**************************************************************************
		写取舵机角度
***************************************************************************/	
	 if(i2c_rw[0]& write_flag )//写角度
	 begin
		case( i )
		0:
			if( Done_Sig ) 
			begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[7:0]; 
				Addr_Sig 			<= 			8'h01; 
			end              //eeprom 写舵机1的当前角度servo_temp[7:0]到addr 00,
					 
		  1:
			if( Done_Sig ) 
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[15:8]; 
				Addr_Sig 			<= 			8'h02;
			end              //eeprom 写舵机2的当前角度servo_temp[15:8]到addr 01,
		  2:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[23:16]; 
				Addr_Sig 			<= 			8'h03;
			end              //eeprom 写舵机3的当前角度servo_temp[23:16]到addr 02,		  
		  3:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[31:24]; 
				Addr_Sig 			<= 			8'h04;
			end              //eeprom 写舵机4的当前角度servo_temp[31:24]到addr 03,			  
		  4:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[39:32]; 
				Addr_Sig 			<= 			8'h05;
			end              //eeprom 写舵机5的当前角度servo_temp[39:32]到addr 04,			  
		  5:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[47:40]; 
				Addr_Sig 			<= 			8'h06;
			end              //eeprom 写舵机6的当前角度servo_temp[47:40]到addr 05,			  
		  6:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[55:48]; 
				Addr_Sig 			<= 			8'h07;
			end              //eeprom 写舵机7的当前角度servo_temp[47:40]到addr 06,			 		  
		  7:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			1'd1;
				write_flag			<=				1'b0;	
			end                    //等待IIC写操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b01; 
				WrData 				<= 			servo_temp[63:56]; 
				Addr_Sig 			<= 			8'h08;
			end              //eeprom 写舵机8的当前角度servo_temp[47:40]到addr 07,			  
			endcase	
		end
/**************************************************************************
		读取舵机角度
***************************************************************************/		
		if(i2c_rw[1] & read_flag)//读取角度
		begin
		case( i )
		0:
			if( Done_Sig ) 
			begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
				servo_rd[7:0]		<=				RdData;	
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10; 
				Addr_Sig 			<= 			8'h01; 
			end              //eeprom 读舵机1的当前角度addr servo_rd[7:0],
					 
		  1:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
				servo_rd[15:8]		<=				RdData;	
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10; 
				Addr_Sig 			<= 			8'h02;
			end              //eeprom 读舵机2的当前角度addr 01到servo_rd[15:8]
		  2:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
				servo_rd[23:16]	<=				RdData;	
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10;  
				Addr_Sig 			<= 			8'h03;
			end              //eeprom 读舵机3的当前角度addr 02到servo_rd[23:16],		  
		  3:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
				servo_rd[31:24]	<=				RdData;		
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10;  
				Addr_Sig 			<= 			8'h04;
			end              //eeprom 读舵机4的当前角度addr 03到servo_rd[31:24],			  
		  4:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1;
				servo_rd[39:32]	<=				RdData;				
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10; 
				Addr_Sig 			<= 			8'h05;
			end              //eeprom 读舵机5的当前角度addr 04到servo_rd[39:32],			  
		  5:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1; 
				servo_rd[47:40]	<=				RdData;
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10; 
				Addr_Sig 			<= 			8'h06;
			end              //eeprom 读舵机6的当前角度addr 05到servo_rd[47:40],			  
		  6:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				i 						<= 			i + 1'd1;
				servo_rd[55:48]	<=				RdData;
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10; 
				Addr_Sig 			<= 			8'h07;
			end              //eeprom 读舵机7的当前角度addr 06到servo_rd[47:40],			 		  
		  7:
			if( Done_Sig )
		  	begin 
				Start_Sig 			<= 			2'b00; 
				servo_rd[63:56]	<=				RdData;
				//servo_rflag			<=				1'b1;
				i 						<= 			1'd1;
				read_flag			<=				1'b0;
			end                    //等待IIC读操作完成, 完成i状态变为1
			else 
			begin 
				Start_Sig 			<= 			2'b10; 
				Addr_Sig 			<= 			8'h08;
			end              //eeprom 读舵机8的当前角度addr 07到servo_rd[47:40],			  
			endcase	
		end
	end	  	
endmodule
