`timescale 1ns/1ps
// Stable bundled-data request/ACK crossing. Reset must assert both endpoints.
module nav3_async_mailbox #(parameter integer WIDTH=32, parameter integer CAPTURE_DST=1)(
 input wire src_clk,src_reset_n,src_valid,input wire [WIDTH-1:0]src_data,
 output wire src_ready,
 input wire dst_clk,dst_reset_n,dst_ready,
 output reg dst_valid,output wire [WIDTH-1:0]dst_data
);
 reg request,ack;reg [WIDTH-1:0]held;
 (* syn_async_reg=1 *) reg[1:0]ack_sync,req_sync;
 assign src_ready=request==ack_sync[1];
 // Capture the stable bundle on the SAME edge that publishes dst_valid.
 // Consumers see only destination-domain registers, including while stalled.
 // The two-stage request and held-until-ACK protocol qualify the whole bus;
 // independently synchronizing data bits would not preserve bus coherence.
 generate if(CAPTURE_DST)begin: captured
  // Unused/constant fields may be folded; live data retains this clock edge.
  // Verify held-to-capture connectivity in the actual mapped netlist.
  reg [WIDTH-1:0] payload;
  always @(posedge dst_clk or negedge dst_reset_n)begin
   if(!dst_reset_n)payload<=0;
   else if(!dst_valid && req_sync[1]!=ack)payload<=held;
  end
  assign dst_data=payload;
 end else begin: bundled_wire
  assign dst_data=held;
 end endgenerate
 always @(posedge src_clk or negedge src_reset_n)begin
  if(!src_reset_n)begin request<=0;held<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[0],ack};
   if(src_valid&&src_ready)begin held<=src_data;request<=!request;end
  end
 end
 always @(posedge dst_clk or negedge dst_reset_n)begin
  if(!dst_reset_n)begin req_sync<=0;ack<=0;dst_valid<=0;end
  else begin
   req_sync<={req_sync[0],request};
   if(dst_valid)begin
    if(dst_ready)begin dst_valid<=0;ack<=req_sync[1];end
   end else if(req_sync[1]!=ack)dst_valid<=1;
  end
 end
endmodule
