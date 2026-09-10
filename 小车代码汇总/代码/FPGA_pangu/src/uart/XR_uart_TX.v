`timescale 1ns / 1ps
// Company: 			Shenzhen Xiao-R Geek Technology Co.,Ltd
// Engineer: 			Ray_yi
// 
// Create Date:    	14:10:21 06/23/2016 
// Design Name: 	 	XR_FPGA 
// Module Name:    	XR_uart_TX 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 		16个clock发送一个bit,起始位、8位数据、停止位 共10位，160个时钟完成一帧数据发送
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module XR_uart_TX
(
			 input						clk
			,input 			[7:0]		txdata
			,input						wrsig		
			,output	reg				tx
);



/********************************************************
 parameter
********************************************************/
parameter paritymode = 1'b0;




/******************************************************
   register 
******************************************************/  
reg 						idle;
reg 						send;
reg 						wrsigbuf;
reg		 				wrsigrise;
reg 						presult;
reg			[7:0]		cnt;




/******************************************************
   检测发送命令是否有效，判断wrsig的上升沿 
******************************************************/ 
always @(posedge clk)
begin
   wrsigbuf 			<= 			wrsig;
   wrsigrise 			<= 			(~wrsigbuf) & wrsig;
end


always @(posedge clk)
begin
  if (wrsigrise &&  (~idle))  //当发送命令有效且线路为空闲时，启动新的数据发送进程
  begin
     send 			<= 		1'b1;
  end
  else if(cnt == 8'd152)      //一帧资料发送结束
  begin
     send 			<= 		1'b0;
  end
end



/*********************************************************************
使用152个时钟发送一个数据（起始位、8位数据、停止位），每位占用16个时钟
**********************************************************************/ 
always @(posedge clk)
begin
  if(send == 1'b1)  begin
    case(cnt)                 //tx变低电平产生起始位，0~15个时钟为发送起始位
    8'd0: begin
         tx 			<= 			1'b0;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd16: begin
         tx 			<= 			txdata[0];    //发送数据位的低位bit0,占用第16~31个时钟
         presult 		<= 			txdata[0]^paritymode;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd32: begin
         tx 			<= 			txdata[1];    //发送数据位的第2位bit1,占用第47~32个时钟
         presult 		<= 			txdata[1]^presult;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd48: begin
         tx 			<= 			txdata[2];    //发送数据位的第3位bit2,占用第63~48个时钟
         presult 		<= 			txdata[2]^presult;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd64: begin
         tx 			<= 			txdata[3];    //发送数据位的第4位bit3,占用第79~64个时钟
         presult		<= 			txdata[3]^presult;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd80: begin 
         tx 			<= 			txdata[4];   //发送数据位的第5位bit4,占用第95~80个时钟
         presult 		<= 			txdata[4]^presult;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd96: begin
         tx 			<= 			txdata[5];    //发送数据位的第6位bit5,占用第111~96个时钟
         presult 		<= 			txdata[5]^presult;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd112: begin
         tx 			<= 			txdata[6];    //发送数据位的第7位bit6,占用第127~112个时钟
         presult 		<= 			txdata[6]^presult;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd128: begin 
         tx 			<= 			txdata[7];    //发送数据位的第8位bit7,占用第143~128个时钟
         presult 		<= 			txdata[7]^presult;
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd144: begin
			tx 			<= 			1'b1;         //发送停止位，占用第151~144个时钟            
         idle 			<= 			1'b1;
         cnt 			<= 			cnt + 8'd1;
    end
    8'd152: begin
         tx 			<= 			1'b1;             
         idle 			<= 			1'b0;       //一帧资料发送结束
         cnt 			<= 			cnt + 8'd1;
    end
    default: begin
          cnt 			<= 			cnt + 8'd1;
    end
   endcase
  end
  else  begin
    tx 			<= 		1'b1;
    cnt 			<= 		8'd0;
    idle 		<= 		1'b0;
  end
end







endmodule
