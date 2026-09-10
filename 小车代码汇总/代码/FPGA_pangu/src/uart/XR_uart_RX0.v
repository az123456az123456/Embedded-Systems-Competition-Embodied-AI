`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    	14:10:21 06/23/2016 
// Design Name: 	 	XR_FPGA
// Module Name:    	XR_uart_RX 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 		16个clock接收一个bit，16个时钟采样，取中间的采样值，
//							起始位、8位数据、停止位 共10位，160个时钟完成一帧数据接收
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_uart_RX0
(
			 input						clk			//uart采样时钟
			,input						rx				//rx信号线输入电平
			,input						rst_n			//复位脚电平，低复位
			,output	reg	[7:0]		rxdata		//一个字节接受数据输出
			,output	reg				rdsig			//接受数据有效，高代表接受一个字节数据
			,output	reg	[7:0]		cnt
			,input						init_flag
			,output  reg				receive
);



/********************************************************
 parameter
********************************************************/
parameter paritymode = 1'b0;




/******************************************************
   register 
******************************************************/  

reg 						rxbuf;
reg						rxfall;
//reg						receive;
reg 						presult;
reg						idle;




/******************************************************
   检测RX下降沿(空闲时为高电平) 
******************************************************/ 
always @(posedge clk)   
begin
  rxbuf 			<=				rx;
  rxfall 		<= 			rxbuf & (~rx);
end





/******************************************************
   接收状态标志--receive 1：开始接收 0：停止接收
******************************************************/ 
always @(posedge clk)
begin
  if (rxfall && (~idle)&& init_flag)  //初始化完成之后，检测到线路的下降沿并且原先线路为空闲，启动接收数据进程
		receive			<=			1'b1;      //开始接收数据
		
  else if(cnt == 8'd152)
		receive 			<= 		1'b0;
end





/******************************************************
	使用160个时钟接收一个数据（起始位、8位数据、停止位)
	上位机不带校验位,	每位占用16个时钟
******************************************************/ 
always @(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		rxdata		<=			8'hFF;
		rdsig 		<= 		1'b0;
		cnt 			<= 		8'd0;
		idle 			<= 		1'b0;
	end
	else 
	begin
		if(receive == 1'b1)
		begin
			case (cnt)
			8'd0:                   //0~15个时钟为接收第一个比特，起始位
			begin
				idle 			<= 		1'b1;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;
			end
			8'd24:                  //16~31个时钟为第1个bit数据，取中间第24个时钟的采样值
			begin
				idle 			<= 		1'b1;
				rxdata[0] 	<= 		rx;
				presult 		<= 		paritymode^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;
			 end
			8'd40:                 //47~32个时钟为第2个bit数据，取中间第40个时钟的采样值 
			begin
				idle 			<= 		1'b1;
				rxdata[1] 	<= 		rx;
				presult 		<= 		presult^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;
			 end
			8'd56:                 //63~48个时钟为第3个bit数据，取中间第56个时钟的采样值   
			begin
				idle 			<= 		1'b1;
				rxdata[2] 	<= 		rx;
				presult 		<= 		presult^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig	 		<= 		1'b0;
			 end
			8'd72:                //79~64个时钟为第4个bit数据，取中间第72个时钟的采样值   
			begin
				idle 			<= 		1'b1;
				rxdata[3] 	<= 		rx;
				presult 		<= 		presult^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;
			 end
			8'd88:               //95~80个时钟为第5个bit数据，取中间第88个时钟的采样值    
			begin
				idle 			<= 		1'b1;
				rxdata[4] 	<= 		rx;
				presult 		<= 		presult^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;
			 end
			8'd104:             //111~96个时钟为第6个bit数据，取中间第104个时钟的采样值    
			begin
				idle 			<= 		1'b1;
				rxdata[5] 	<= 		rx;
				presult 		<= 		presult^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;
			 end
			8'd120:             //127~112个时钟为第7个bit数据，取中间第120个时钟的采样值     
			begin
				idle 			<= 		1'b1;
				rxdata[6] 	<= 		rx;
				presult 		<= 		presult^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;
			 end
			8'd136:            //143~128个时钟为第8个bit数据，取中间第136个时钟的采样值   
			begin
				idle 			<= 		1'b1;
				rxdata[7] 	<= 		rx;
				presult 		<= 		presult^rx;
				cnt 			<= 		cnt + 8'd1;
				rdsig 		<= 		1'b0;      //接收数据有效
			 end
			8'd144:            //159~144停止  
			begin
			  idle 			<= 		1'b1;
			  rdsig 			<= 		1'b1;
			  cnt 			<= 		cnt + 8'd1;  
			end
			default:
			  begin
				cnt 			<= 		cnt + 8'd1;
			 end
			endcase
		end
		else
		begin
			cnt 			<= 		8'd0;
			idle 			<= 		1'b0;
			rdsig 		<= 		1'b0;
		end
	end//rst_n
end//always






endmodule
