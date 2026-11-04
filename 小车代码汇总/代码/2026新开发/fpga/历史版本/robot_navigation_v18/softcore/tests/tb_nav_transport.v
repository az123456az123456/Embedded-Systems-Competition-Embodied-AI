`timescale 1ns/1ps
`include "build/nav_count.vh"
module tb_nav_transport;
 reg clk=0,netclk=0,reset_n=0;
 always #10 clk=~clk;
 always #4 netclk=~netclk;
 reg [7:0] rx_data=0;reg rx_valid=0;
 wire parsed_valid,request_valid,request_ready,reply_valid,reply_ready;
 wire [511:0] parsed_data,request_data,reply_data;
 wire [207:0] status;wire trap;
 wire tx_pending,tx_busy,tx_valid;wire [7:0] tx_data;
 wire [31:0] accepted,rejected;
 nav_packet_rx parser(.clk(netclk),.rst_n(reset_n),.data(rx_data),.valid(rx_valid),.error(1'b0),
 .command_valid(parsed_valid),.command_data(parsed_data),.accepted_frames(accepted),.rejected_frames(rejected));
 imu_snapshot_cdc rxcdc(.src_clk(netclk),.src_rst_n(reset_n),.src_valid(parsed_valid),.src_data(parsed_data),.src_ready(),
 .dst_clk(clk),.dst_rst_n(reset_n),.dst_ready(request_ready),.dst_valid(request_valid),.dst_data(request_data));
 robot_nav_soc soc(.clk(clk),.reset_n(reset_n),.imu_valid(1'b0),.imu_packet(512'b0),
 .motor_valid(1'b0),.motor_packet(128'b0),.sensor_valid(1'b0),.sensor_packet(512'b0),
 .nav_request_valid(request_valid),.nav_request_packet(request_data),.nav_request_ready(request_ready),
 .nav_reply_valid(reply_valid),.nav_reply_packet(reply_data),.nav_reply_ready(reply_ready),.nav_status(status),.cpu_trap(trap));
 nav_message_gmii sender(.source_clk(clk),.source_rst_n(reset_n),.message_valid(reply_valid),.message_data(reply_data),.message_ready(reply_ready),
 .tx_clk(netclk),.tx_rst_n(reset_n),.tx_allow(1'b1),.tx_pending(tx_pending),.tx_busy(tx_busy),.gmii_valid(tx_valid),.gmii_data(tx_data));
 reg [735:0] vectors [0:`NAV_VECTORS-1];
 reg expect_reply [0:`NAV_VECTORS-1];
 integer i,j,frames=0,fd,outbytes=0,oldframes;
 reg prev_valid=0;
 always @(posedge clk)if(reset_n&&trap)$fatal(1,"RV32 trapped addr=%x",soc.mem_addr);
 always @(negedge netclk)begin
  if(tx_valid)begin $fwrite(fd,"%02x",tx_data);outbytes=outbytes+1;end
  if(prev_valid&&!tx_valid)begin $fwrite(fd,"\n");frames=frames+1;if(outbytes!=92)$fatal(1,"Badframe bytes%0d",outbytes);outbytes=0;end
  prev_valid=tx_valid;
 end
 initial begin
  $readmemh("build/nav_vectors.hex",vectors);$readmemh("build/nav_expect.hex",expect_reply);
  fd=$fopen("build/nav_actual.hex","w");
  repeat(6)@(negedge netclk);reset_n=1;
  wait(soc.status==3);
  for(i=0;i<`NAV_VECTORS;i=i+1)begin
   oldframes=frames;
   for(j=0;j<92;j=j+1)begin @(negedge netclk);rx_valid=1;rx_data=vectors[i][735-j*8 -:8];end
   @(negedge netclk);rx_valid=0;rx_data=0;
   // Retry PLAN while the actual softcore is still computing, before any
   // reply exists. The queued duplicate must later replay the same result.
   if(i+1<`NAV_VECTORS && vectors[i+1]==vectors[i])begin
    repeat(100)@(negedge netclk);
    for(j=0;j<92;j=j+1)begin @(negedge netclk);rx_valid=1;rx_data=vectors[i][735-j*8 -:8];end
    @(negedge netclk);rx_valid=0;rx_data=0;
    wait(frames==oldframes+2);i=i+1;
   end else if(expect_reply[i])wait(frames==oldframes+1);
   else begin repeat(30000)@(negedge netclk);if(frames!=oldframes)$fatal(1,"Invalidframe accepted");end
   repeat(100)@(negedge netclk);
  end
  if(rejected!=1)$fatal(1,"Rejectedcount expected1 got%0d",rejected);
  $fclose(fd);
  $display("PASS real RV32IM firmware + RX CRC16/FCS + two clock CDC + atomic map + A* + reply framing: %0d requests %0d replies",accepted,frames);
  $finish;
 end
 initial begin #3000000000;$fatal(1,"Navigation integration timeout");end
endmodule
