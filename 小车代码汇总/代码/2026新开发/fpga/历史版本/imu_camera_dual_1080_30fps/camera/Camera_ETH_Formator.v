/*============================================================================
*
*  LOGIC CORE:          对摄像头行数据进行编号，以便于以太网上位机正确接收图像
*  MODULE NAME:         Camera_ETH_Formator()
*  COMPANY:             武汉芯路恒科技有限公司
*                       http://xiaomeige.taobao.com
*	author:					小梅哥
*	Website:					www.corecourse.cn
*  REVISION HISTORY:  
*
*    Revision 1.0  04/10/2019     Description: Initial Release.
*
*  FUNCTIONAL DESCRIPTION:
===========================================================================*/

module Camera_ETH_Formator(
	PCLK,
	Rst_n,
    Init_Done,
	HREF,
	VSYNC,
	DATA,

	fifo_aclr,
	wrdata,
	wrreq
);

	input PCLK;
	input Rst_n;
	input Init_Done;
	input HREF;
	input VSYNC;
	input [7:0]DATA;
	output reg fifo_aclr;	
	output [7:0]wrdata;
	output reg wrreq;
	
	reg [23:0]data_tmp;
	
	reg [15:0]Vcnt;
		
	reg href_r1;
	reg href_r2;
	
	assign wrdata = data_tmp[23:16];
	
	always@(posedge PCLK)begin
		href_r1 <= HREF;
		href_r2 <= href_r1;	
	end
	
    always @ (posedge PCLK or negedge Rst_n)
	if (!Rst_n)
		fifo_aclr <= #1 1'b1;
	else if (Init_Done && VSYNC ) //等到初始化摄像完成且头场同步信号出现，释放清零信号，开始写入数据
		fifo_aclr <= #1 1'b0;
	else
		fifo_aclr <= #1 fifo_aclr;
	
	always@(posedge PCLK or negedge Rst_n)
	if(!Rst_n)
		data_tmp <= 0;
	else if({href_r1,HREF} == 2'b01)
		data_tmp <= {Vcnt[7:0],Vcnt[15:8],DATA};
	else 
		data_tmp <= {data_tmp[15:0],DATA};
	
	always@(posedge PCLK)
	if({href_r1,HREF} == 2'b01)
		wrreq <= 1;
	else if(href_r2 | href_r1) 
		wrreq <= 1;
	else
		wrreq <= 0;

	always@(posedge PCLK or negedge Rst_n)
	if(!Rst_n)
		Vcnt <= 0;
	else if(VSYNC)
		Vcnt <= 0;
	else if({href_r1,HREF} == 2'b10)
		Vcnt <= Vcnt + 1'd1;

endmodule
