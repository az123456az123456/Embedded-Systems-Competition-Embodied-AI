`timescale 1ns/1ps
// One source-clock telemetry stage. Capture the complete ACK + attached NAV1
// snapshot on the same edge as its valid bit. This path never grants motion.
module motor_status_align(
 input wire clk,reset_n,in_valid,
 input wire [127:0] in_status,
 input wire [207:0] in_nav,
 output wire out_valid,
 output wire [127:0] out_status,
 output wire [207:0] out_nav
);
 reg [336:0] held;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)held<=337'b0;
  else held<={in_valid,in_status,in_nav};
 end
 assign {out_valid,out_status,out_nav}=held;
endmodule
