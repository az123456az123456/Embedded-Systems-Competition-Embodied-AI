`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:38:45 08/22/2016 
// Design Name: 
// Module Name:    XR_smg_interface 
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
module XR_smg_interface
(
    input CLK,
	 input RSTn,
	 output [7:0]SMG_Data,
	 output [5:0]Scan_Sig
);

    /******************************/ 
 
    wire [23:0]Number_Sig;
	 
    smg_control_module U1_inst
	 (
	     .CLK( CLK ),
		  .RSTn( RSTn ),
		  .Number_Sig( Number_Sig ) // output - to U2
	 );
	 
	 /******************************/ 
	 
	 smg_interface U2_inst
	 (
	     .CLK( CLK ),
		  .RSTn( RSTn ),
		  .Number_Sig( Number_Sig ), // input - from U1
		  .SMG_Data( SMG_Data ),     // output - to top
		  .Scan_Sig( Scan_Sig )      // output - to top
	 );
	 
    /******************************/ 

endmodule