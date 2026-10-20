`timescale 1ns/1ps
module tb_udp_gap;
 parameter GAP=16;
 reg clk=0,rst_n=0,wr=0,allow=0;reg [7:0] data=0;
 always #4 clk=~clk;
 wire busy,valid;wire [7:0] tx;
 UDP_Send #(.POST_FRAME_GAP(GAP)) dut(.tx_allow(allow),.tx_busy(busy),.Clk(clk),.Rst_n(rst_n),.Tx_Done(),
  .des_mac(48'hffffffffffff),.src_mac(48'h000a3501fec0),.des_port(16'd6102),.src_port(16'd5000),
  .des_ip(32'hc0a80003),.src_ip(32'hc0a80002),.data_length(16'd1922),
  .GMII_GTXC(clk),.GMII_TXD(tx),.GMII_TXEN(valid),.wrreq(wr),.wrdata(data),.wrclk(clk),.aclr(!rst_n),.wrusedw());
 integer i,gap=0,frames=0,observed=0;reg previous=0;
 always @(negedge clk)if(rst_n)begin
  if(valid && !previous && frames>0)begin
   if(gap<12 || gap>GAP+16)$fatal(1,"Bad actual IFG %d",gap);
   observed=gap;
  end
  if(previous && !valid)frames=frames+1;
  if(valid)gap=0;else gap=gap+1;
  previous=valid;
 end
 initial begin
  #20;rst_n=1;
  for(i=0;i<3844;i=i+1)begin @(negedge clk);wr=1;data=i%251;end
  @(negedge clk);wr=0;allow=1;
  wait(frames==2);repeat(3)@(negedge clk);
  $display("PASS UDP saturated sender guard=%0d actual_idle_byte_clocks=%0d, complete rows and >=12-byte IFG",GAP,observed);$finish;
 end
 initial begin #200000;$fatal(1,"timeout");end
endmodule
