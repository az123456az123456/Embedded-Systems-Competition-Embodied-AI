`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:44:21 08/22/2016 
// Design Name: 
// Module Name:    smg_interface 
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
module smg_interface
(
    input CLK,
	 input RSTn,
	 input [23:0]Number_Sig,
	 output [7:0]SMG_Data,
	 output [5:0]Scan_Sig
);

    /******************************************/
	 
	 wire [3:0]Number_Data;
	 
	 XR_smg_control_module U1
	 (
	    .CLK( CLK ),
		 .RSTn( RSTn ),
		 .Number_Sig( Number_Sig ),    // input - from top
		 .Number_Data( Number_Data )   // output - to U2
	 );
	 
	 /******************************************/
	 
    XR_smg_encode_module U2
	 (
	     .CLK( CLK ),
		  .RSTn( RSTn ),
		  .Number_Data( Number_Data ),   // input - from U2
		  .SMG_Data( SMG_Data )          // output - to top
	 );
	 
	 /*******************************************/
	 
	 XR_smg_scan_module U3
	 (
	     .CLK( CLK ),
		  .RSTn( RSTn ),
		  .Scan_Sig( Scan_Sig )  // output - to top
	 );
	 
	 /*******************************************/
	 
	 

endmodule
