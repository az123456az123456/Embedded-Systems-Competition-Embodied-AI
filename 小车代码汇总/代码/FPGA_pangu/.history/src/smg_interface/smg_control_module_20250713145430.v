`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:44:06 08/22/2016 
// Design Name: 
// Module Name:    smg_control_module 
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
module smg_control_module
(
    input CLK,
	input RSTn,
	output [23:0]Number_Sig,
	input [7:0]servo7_angle,  // 新增输出端口，表示7号舵机角度
	input [23:0]Distance
);
   
	 /******************************/
	 
	 parameter T100MS = 23'd4_999_999;          //100ms的计数
	 
	 /******************************/
	 
	 reg [22:0]C1;
	 
	 always @ ( posedge CLK or negedge RSTn )
	     if( !RSTn )
		      C1 <= 23'd0;
		  else if( C1 == T100MS )
		      C1 <= 23'd0;
		  else 
		      C1 <= C1 + 1'b1;
	
	  /*******************************************************/
	 
	 reg [3:0]i;
	 reg [23:0]rNum;
	 reg [23:0]rNumber;
	 
	 always @ ( posedge CLK or negedge RSTn )
	      if( !RSTn )
			    begin
			        i <= 4'd0;
					rNum <= 24'd0;
					rNumber <= 24'd0;
				 end
		   else begin
			    // case( i )
				 
				//      0:
				// 	  if( C1 == T100MS ) begin rNum[3:0] <= rNum[3:0] + 1'b1; i <= i + 1'b1; end                 //个位的计数
					  
				// 	  1:
				// 	  if( rNum[3:0] > 4'd9 ) begin rNum[7:4] <= rNum[7:4] + 1'b1; rNum[3:0] <= 4'd0; i <= i + 1'b1; end //个位>9, 进位给十位
				// 	  else i <= i + 1'b1; 
					  
				// 	  2:
				// 	  if( rNum[7:4] > 4'd9 ) begin rNum[11:8] <= rNum[11:8] + 1'b1; rNum[7:4] <= 4'd0; i <= i + 1'b1; end//十位>9, 进位给百位
				// 	  else i <= i + 1'b1;
					  
				// 	  3:
				// 	  if( rNum[11:8] > 4'd9 ) begin rNum[15:12] <= rNum[15:12] + 1'b1; rNum[11:8] <= 4'd0; i <= i + 1'b1; end//百位>9, 进位给千位
				// 	  else i <= i + 1'b1;
					  
				// 	  4:
				// 	  if( rNum[15:12] > 4'd9 ) begin rNum[19:16] <= rNum[19:16] + 1'b1; rNum[15:12] <= 4'd0; i <= i + 1'b1; end//千位>9, 进位给万位
			    //     else i <= i + 1'b1;
					  
				// 	  5:
				// 	  if( rNum[19:16] > 4'd9 ) begin rNum[23:20] <= rNum[23:20] + 1'b1; rNum[19:16] <= 4'd0; end //万位>9, 进位给十万位
				// 	  else i <= i + 1'b1;
					  
				// 	  6:
				// 	  if( rNum[23:20] > 4'd9 ) begin rNum <= 24'd0; i <= i + 1'b1; end       //十万位>9, 全部清0
				// 	  else i <= i + 1'b1; 
					  
				// 	  7:
				// 	  begin rNumber <= rNum; i <= 4'd0; end
				 
				//  endcase
				rNumber<={8'h03,servo7_angle,Distance[7:0]};
		   end
    /*******************************************************/
	 
	 assign Number_Sig = rNumber;
	 
	 /*******************************************************/
	 
endmodule
