`timescale 1ns/1ps
module tb;
 parameter PAD=0; localparam W=5,H=3,RIGHT=(PAD==0?0:36),OW=PAD+W+RIGHT;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,in_valid=0,in_sof=0,in_eol=0,in_eof=0,in_abort=0;
 reg[9:0]in_x=0,in_y=0;reg[23:0]in_rgb=0;
 wire out_valid,out_sof,out_eol,out_eof,out_abort;wire[9:0]out_x,out_y;wire[23:0]out_rgb;wire[6:0]fifo_peak;
 integer frame_id=0,complete=0,samples=0,cycles=0;
 rgb_letterbox #(.ACTIVE_WIDTH(W),.ACTIVE_HEIGHT(H),.PAD_LEFT(PAD),.PAD_RIGHT(RIGHT)) dut(.*);
 function[23:0]color;input integer f,n;begin color={8'h81+8'(f),8'h31+8'(n),8'h55^8'(f+n)};end endfunction
 task pixel;input integer f,n;begin
  @(negedge clk);frame_id=f;in_valid=1;in_x=10'(n%W);in_y=10'(n/W);in_sof=(n==0);in_eol=(n%W==W-1);in_eof=(n==W*H-1);in_rgb=color(f,n);
 end endtask
 task idle;input integer n;integer i;begin
  @(negedge clk);in_valid=0;in_sof=0;in_eol=0;in_eof=0;
  for(i=0;i<n;i=i+1)@(negedge clk);
 end endtask
 task full;input integer f;integer n;begin
  for(n=0;n<W*H;n=n+1)begin pixel(f,n);if(n%3==1)idle(1);end
  idle(500);
 end endtask
 always @(posedge clk)begin
  #1;cycles=cycles+1;
  if(rst_n)begin
   if(dut.count>64)$fatal(1,"capacity over64");
   if(dut.head_valid!==(dut.count!=0))$fatal(1,"head count mismatch");
   if(dut.read_enable&&dut.store&&dut.read_address==dut.store_address)$fatal(1,"R/W collision");
   if(out_valid)begin
    if((^out_rgb)===1'bx)$fatal(1,"uninitialized RAM leaked after reset/SOF");
    if(out_x>=OW||out_y>=H)$fatal(1,"coordinate overflow");
    if(out_sof!==(out_x==0&&out_y==0)||out_eol!==(out_x==OW-1)||out_eof!==(out_x==OW-1&&out_y==H-1))$fatal(1,"bad flags");
    if(out_rgb!==(out_x<PAD||out_x>=PAD+W?24'h727272:color(frame_id,out_y*W+out_x-PAD)))$fatal(1,"bad/stale pixel f%0d xy%0d/%0d got%h",frame_id,out_x,out_y,out_rgb);
    samples=samples+1;if(out_eof)complete=complete+1;
   end
  end
  if(cycles>10000)$fatal(1,"timeout");
 end
 integer n;
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  full(1);
  for(n=0;n<8;n=n+1)pixel(2,n);
  @(negedge clk);in_valid=0;in_abort=1;
  @(negedge clk);in_abort=0;
  full(3);
  for(n=0;n<5;n=n+1)pixel(4,n);
  @(negedge clk);rst_n=0;in_valid=0;
  repeat(3)@(negedge clk);rst_n=1;
  full(5);
  for(n=0;n<7;n=n+1)pixel(6,n);
  full(7);
  if(complete!=4)$fatal(1,"wrong complete frame count %0d",complete);
  $display("PASS PAD=%0d complete=%0d samples=%0d peak=%0d",PAD,complete,samples,fifo_peak);$finish;
 end
endmodule
