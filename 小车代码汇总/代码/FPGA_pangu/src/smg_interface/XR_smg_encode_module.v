`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:47:00 08/22/2016 
// Design Name: 
// Module Name:    XR_smg_encode_module 
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
module XR_smg_encode_module
(
	 input CLK,
	 input RSTn,
	 input [3:0]Number_Data,
	 output [7:0]SMG_Data
);

	 /***************************************/
	 
	 parameter _0 = 8'b1000_0001, _1 = 8'b1100_1111, _2 = 8'b1001_0010, 
	           _3 = 8'b1000_0110, _4 = 8'b1100_1100, _5 = 8'b1010_0100, 
				  _6 = 8'b1010_0000, _7 = 8'b1000_1111, _8 = 8'b1000_0000,
				  _9 = 8'b1000_0100;
	 
	 /***************************************/
	 	// 4'd0:smg = 8'b1000_0001;//"0" 
		// 4'd1:smg = 8'b1100_1111;//"1"  
		// 4'd2:smg = 8'b1001_0010;//"2"  
		// 4'd3:smg = 8'b1000_0110;//"3"  
		// 4'd4:smg = 8'b1100_1100;//"4"  
		// 4'd5:smg = 8'b1010_0100;//"5"  
		// 4'd6:smg = 8'b1010_0000;//"6"  
		// 4'd7:smg = 8'b1000_1111;//"7"  
		// 4'd8:smg = 8'b1000_0000;//"8"  
		// 4'd9:smg = 8'b1000_0100;//"9"  
	 reg [7:0]rSMG;

	 always @ ( posedge CLK or negedge RSTn )
	     if( !RSTn )
		      begin
				    rSMG <= 8'b1111_1111;
				end
        else
				//rSMG <= 8'b0000_0000;
			
		       case( Number_Data )
		           
					  4'd0 : rSMG <= _0;
					  4'd1 : rSMG <= _1;
					  4'd2 : rSMG <= _2;
					  4'd3 : rSMG <= _3;
					  4'd4 : rSMG <= _4;
					  4'd5 : rSMG <= _5;
					  4'd6 : rSMG <= _6;
					  4'd7 : rSMG <= _7;
					  4'd8 : rSMG <= _8;
					  4'd9 : rSMG <= _9;
					  
				 endcase
		      
	 /***************************************/
	 
	 assign SMG_Data = rSMG;
	 
	 /***************************************/
			  
endmodule
