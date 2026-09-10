`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    14:12:40 06/23/2016 
// Design Name: 
// Module Name:    XR_uart_ctrl 
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
module XR_uart_ctrl
(
    input       			clk
	,input       			rdsig                //串口接收数据有效信号
	,input   		[7:0]   rxdata               //串口接收数据
	,output   	     		wrsig                //串口发送指示信号
	,output			[7:0]	dataout              //串口发送数据
	,input			[3:0]	send_flag
	,input			[23:0]	send_data	
);

/******************************************************
   register 
******************************************************/  
reg 	[19:0] 		uart_wait;
reg 	[7:0] 		uart_cnt;
reg 				rx_data_valid;
reg 	[7:0] 		store [20:0];                        //存储发送字符
reg 	[2:0] 		uart_stat;                           //uart状态机
reg 	[14:0] 		k;                                   //用于指示发送的第几个数据
reg 	[7:0] 		dataout_reg;
reg 				data_sel;
reg 				wrsig_reg;
reg		[7:0]		send_reg [4:0];						//上传数据缓存
  
/******************************************************
  发送数据选择及使能
******************************************************/  
assign dataout = data_sel ?  dataout_reg : rxdata ;            //发送数据选择：data_sel高，选择存储的字符串，data_sel：低，选择接收的数据
assign wrsig = data_sel ?  wrsig_reg : rdsig;                  //发送请求选择：data_sel高，发送存储的字符串，data_sel：低，发送接收的数据

/******************************************************
  存储串口发送的默认字符串
******************************************************/  
always @(*)
begin     //定义发送的字符
	 store[0]			<=			8'h53;          		//存储字符S
	 store[1]			<=			8'h5a;          		//存储字符Z
	 store[2]			<=			8'h54;          		//存储字符T
	 store[3]			<=			8'h55;          		//存储字符U
	 store[4]			<=			8'h2d;          		//存储字符-
	 store[5]			<=			8'h49;          		//存储字符I
	 store[6]			<=			8'h43;          		//存储字符C
	 store[7]			<=			8'h4f;          		//存储字符O
	 store[8]			<=			8'h43;          		//存储字符C
	 store[9]			<=			8'h2d;          		//存储字符-
	 store[10]			<=			8'h77;          		//存储字符w
	 store[11]			<=			8'h69;          		//存储字符i
	 store[12]			<=			8'h66;          		//存储字符f
	 store[13]			<=			8'h69;          		//存储字符i
	 store[14]			<=			8'h2d;          		//存储字符-
	 store[15]			<=			8'h46;          		//存储字符F
	 store[16]			<=			8'h50;          		//存储字符P
	 store[17]			<=			8'h47;          		//存储字符G
	 store[18]			<=			8'h41;          		//存储字符A
	 store[19]			<=			8'h0d;          		//存储字符回车
	 store[20]			<=			8'h0a;          		//存储字符回车	 
	
	 send_reg[0]		<=			8'hFF;                  //上传字符FF
	 send_reg[1]		<=			send_data[23:16];       //上传字符01
	 send_reg[2]		<=			send_data[15:8];        //上传字符02
	 send_reg[3]		<=			send_data[7:0];         //上传字符03
	 send_reg[4]		<=			8'hFF;                  //上传字符FF 	
end 

/******************************************************
  串口发送控制，每隔一段时间产生一个发送字符串的命令
  可修改为控制标志位开通上传功能
******************************************************/ 
always @(negedge clk)
begin
  if(rdsig == 1'b1) begin   //如果串口有接收到数据，停止发送字符串                        
		uart_wait 			<= 			0;
		rx_data_valid 		<=			1'b0;
  end
  else begin
    if (uart_wait ==20'h7ffff) begin              			//等待一段时间(每隔一段时间发送字符串）,调整这参数可以改变发送字符串之间的时间间隔。
		uart_wait 			<= 			0;					//此处可改为上传标志位，用于数据上传
		rx_data_valid 		<=			1'b1;	            //等待时间结束，产生一个发送字符串有效信号脉冲
    end		
	 else begin
		uart_wait 			<= 			uart_wait+1'b1;
		rx_data_valid 		<=			1'b0;
	 end
  end
end 

/******************************************************
  串口发送字符串控制程序，依次发送存储的字符串
******************************************************/
always @(posedge clk)
begin
	if (!send_flag[3])
	
//******************************************************************
//发送存储字节，将store[]按顺序赋值给dataout_reg[],
//并提供data_sel信号，wrsig_reg
//********************************************************************	
	begin
		if(rdsig == 1'b1) 
		begin   
			uart_cnt 			<= 			0;
			uart_stat 			<= 			3'b000; 
			data_sel			<=			1'b0;
			k					<=			0;
		end
		else begin
			case(uart_stat)
			3'b000: 
			begin               
				if (rx_data_valid == 1'b1) 		//发送字符串有效信号为高，开始发送字符串
				begin  
					uart_stat 			<= 			3'b001; 
					data_sel			<=			1'b1; 
				end
			end//3'b000	
			3'b001: 									
			begin                       
				if (k == 20 ) 							//发送第20个字符   
				begin              		 
					if(uart_cnt ==0) 
					begin
						dataout_reg 			<= 			store[20]; 		//包尾的FF
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b1;      		//发送字符使能脉冲             			
					end	
					else if(uart_cnt ==254) 								//等待一个字符发送完成，发送一个字符的时间为168个时钟，所以这里等待的时间需要大于168
					begin    
						uart_cnt 				<= 			0;
						wrsig_reg 				<= 			1'b0; 				
						uart_stat 				<= 			3'b010; 
						k 						<= 			0;
					end
					else	begin			
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b0;  
					end
				end//if(k==4)
				else begin                      //发送前4个字符   
					if(uart_cnt ==0) 
					begin      
						dataout_reg 			<= 			store[k]; 
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b1;           //发送使能           			
					end	
					else if(uart_cnt ==254) 								//等待一个数据发送完成，发送一个字符的时间为168个时钟，所以这里等待的时间需要大于168
					begin    
						uart_cnt 				<= 			0;
						wrsig_reg 				<= 			1'b0; 
						k 						<= 			k + 1'b1;	    //k加1，发送下一个字符        			
					end
					else	begin			
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b0;  
					end
				end//if(k==4) else	 
			end//3'b001
			3'b010: 					 //发送finish	 
			begin      
				uart_stat 				<= 			3'b000;
				data_sel				<=			1'b0;	
			end
			default:uart_stat 			<= 			3'b000;
			endcase 
		end 	//if (rdsig) else
	end		//  if (!send_flag)

//******************************************************************
//发送上传字节，将send_data[]按顺序赋值给dataout_reg[],FF-01-02-03-FF
//FF								//包头
//send_data[23:16]  				//类型位
//send_data[15:8]					//命令位
//send_data[7:0]					//数据位
//FF								//包尾
//并提供data_sel信号，wrsig_reg
//********************************************************************
	else begin
		if(rdsig == 1'b1) 
		begin   
			uart_cnt 			<= 			0;
			uart_stat 			<= 			3'b000; 
			data_sel			<=			1'b0;
			k					<=			0;
		end
		else begin
			case(uart_stat)
			3'b000: 
			begin               
				if (rx_data_valid == 1'b1) 		//发送字符串有效信号为高，开始发送字符串
				begin  
					uart_stat 			<= 			3'b001; 
					data_sel			<=			1'b1; 
				end
			end//3'b000	
			3'b001: 									
			begin                       
				if (k == 4 ) 							//发送第5个字符   
				begin              		 
					if(uart_cnt ==0) 
					begin
						dataout_reg 			<= 			send_reg[4]; 
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b1;      	//发送字符使能脉冲             			
					end	
					else if(uart_cnt ==254) 								//等待一个字符发送完成，发送一个字符的时间为168个时钟，所以这里等待的时间需要大于168
					begin    
						uart_cnt 				<= 			0;
						wrsig_reg 				<= 			1'b0; 				
						uart_stat 				<= 			3'b010; 
						k 						<= 			0;
					end
					else	begin			
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b0;  
					end
				end//if(k==4)
				else begin                      //发送前18个字符   
					if(uart_cnt ==0) 
					begin      
						dataout_reg 			<= 			send_reg[k]; 
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b1;           //发送使能           			
					end	
					else if(uart_cnt ==254) 									//等待一个数据发送完成，发送一个字符的时间为168个时钟，所以这里等待的时间需要大于168
					begin    
						uart_cnt 				<= 			0;
						wrsig_reg 				<= 			1'b0; 
						k 						<= 			k + 1'b1;	               //k加1，发送下一个字符        			
					end
					else	begin			
						uart_cnt 				<= 			uart_cnt + 1'b1;
						wrsig_reg 				<= 			1'b0;  
					end
				end//if(k==4) else	 
			end//3'b001
			3'b010: 					 //发送finish	 
			begin      
				uart_stat 				<= 			3'b000;
				data_sel					<=				1'b0;	
			end
			default:uart_stat 			<= 			3'b000;
			endcase 
		end 	//if (rdsig) else	

	end// if(send_flag) else

 
end	//always


endmodule
