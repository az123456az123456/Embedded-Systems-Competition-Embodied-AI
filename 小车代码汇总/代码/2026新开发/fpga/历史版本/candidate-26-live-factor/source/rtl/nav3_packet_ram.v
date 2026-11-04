`timescale 1ns/1ps
// Exactly one writer and one synchronous reader; no reset-clearing FF array.
module nav3_packet_ram # (parameter integer WORDS=400)(
 input wire wr_clk,wr_en,input wire [8:0]wr_addr,input wire [31:0]wr_data,
 input wire rd_clk,rd_en,input wire [8:0]rd_addr,output reg [31:0]rd_data
);
 (* syn_ramstyle="block_ram" *) reg [31:0] words[0:WORDS-1];
 always @(posedge wr_clk)if(wr_en)words[wr_addr]<=wr_data;
 always @(posedge rd_clk)if(rd_en)rd_data<=words[rd_addr];
endmodule
