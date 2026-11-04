`timescale 1ns/1ps
// Real bridge + ISP. Directed mailbox update tests supplement the full-pixel
// golden; no clock-domain false paths or hardware throughput are modeled here.
module tb_session_pipeline;
 reg clk=0,rst_n=0,init_done=1,href=0,vsync=0;
 reg [7:0] raw=0;reg session_valid=0;reg [15:0] session_epoch=0;
 wire write,fault;wire [35:0] word;wire [31:0] completed,dropped;
 integer changes=0,checks=0,i;
 always #5 clk=~clk;
 rgb_capture_bridge dut(.clk(clk),.rst_n(rst_n),.init_done(init_done),.href(href),.vsync(vsync),.raw(raw),
  .session_valid(session_valid),.session_epoch(session_epoch),.uptime_gray(32'd0),.fifo_full(1'b0),
  .fifo_write(write),.fifo_word(word),.fault(fault),.completed_frames(completed),.dropped_frames(dropped));
 always @(posedge clk)if(rst_n&&dut.new_session)changes<=changes+1;
 task tick;begin @(posedge clk);#1;end endtask
 task set_request(input valid,input [15:0] epoch);begin @(negedge clk);session_valid=valid;session_epoch=epoch;end endtask
 task check(input condition,input string message);begin checks=checks+1;if(!condition)$fatal(1,"%s",message);end endtask
 initial begin
  repeat(3)tick();@(negedge clk);rst_n=1;
  set_request(1,16'h1234);tick();check(dut.epoch==0,"epoch bypassed sample stage");check(!dut.new_session,"change bypassed compare register");
  set_request(0,16'hffff);tick();check(dut.epoch==0,"epoch bypassed apply stage");check(dut.new_session,"registered change not produced");check(dut.accepted_session_epoch==16'h1234,"held bus mutated accepted request");
  set_request(0,16'h5555);tick();check(dut.epoch==16'h1234&&!dut.new_session,"sampled epoch was not applied");check(changes==1,"wrong initial pulse count");
  for(i=0;i<12;i=i+1)begin set_request(0,16'(i*1999));tick();check(dut.epoch==16'h1234&&!dut.new_session,"unqualified held change altered epoch");end
  // Contiguous repeated same-epoch strobes cannot repeat an abort while a
  // previous request is accepted but not yet applied by the ISP state machine.
  for(i=0;i<10;i=i+1)begin set_request(1,16'h1234);tick();check(!dut.new_session,"same epoch generated change");end
  set_request(0,0);repeat(3)tick();check(changes==1,"same epoch was not idempotent");
  // Unlike real mailboxes, this stress input can submit adjacent local cycles.
  set_request(1,16'h2222);tick();set_request(1,16'h3333);tick();check(dut.epoch==16'h1234&&dut.new_session,"first request comparison stage");
  set_request(0,16'h9999);tick();check(dut.epoch==16'h2222&&dut.new_session,"first staged request overwritten");
  set_request(0,0);tick();check(dut.epoch==16'h3333&&!dut.new_session,"second staged request overwritten");
  repeat(3)tick();check(changes==3,"back-to-back unique pulse count");
  // Without a new physical VSYNC, mid-frame HREF cannot invent a source SOF.
  @(negedge clk);href=1;repeat(20)tick();check(dut.frame_sequence==0,"new session skipped physical VSYNC");
  @(negedge clk);href=0;vsync=1;repeat(3)tick();@(negedge clk);vsync=0;repeat(3)tick();
  @(negedge clk);href=1;tick();check(dut.frame_sequence==1&&dut.active,"physical SOF not accepted");
  for(i=0;i<25;i=i+1)begin set_request(1,16'h3333);tick();check(dut.frame_sequence==1&&dut.active&&!dut.new_session,"same epoch interrupted active source");end
  set_request(0,0);repeat(3)tick();check(changes==3,"active repeated epoch generated pulse");
  // A genuine new epoch aborts the source and waits for the next VSYNC again.
  set_request(1,16'h4444);tick();set_request(0,0);tick();tick();check(dut.epoch==16'h4444&&!dut.active&&dut.frame_sequence==0,"new epoch did not reset source");
  repeat(10)tick();check(dut.frame_sequence==0&&changes==4,"old HREF became a new frame");
  // Reset while a request has been sampled must clear all pending stages.
  set_request(1,16'h7777);tick();@(negedge clk);rst_n=0;session_valid=0;href=0;repeat(3)tick();
  @(negedge clk);rst_n=1;repeat(5)tick();check(dut.epoch==0&&!dut.new_session&&changes==4,"reset replayed stale pending request");
  // A/A/B/A stresses duplicate suppression without losing a true return to A.
  set_request(1,16'h1111);tick();set_request(1,16'h1111);tick();set_request(1,16'h2222);tick();
  check(dut.epoch==16'h1111&&!dut.new_session,"contiguous A repeated abort");
  set_request(1,16'h1111);tick();set_request(0,0);tick();check(dut.epoch==16'h2222,"B in A/A/B/A was lost");
  tick();check(dut.epoch==16'h1111&&!dut.new_session,"return to A was lost");repeat(2)tick();check(changes==7,"A/A/B/A unique changes wrong");
  // A request coincident with physical SOF may begin a short old frame, then
  // must abort it and require a fresh VSYNC rather than shifting its pixels.
  @(negedge clk);vsync=1;repeat(3)tick();@(negedge clk);vsync=0;repeat(2)tick();
  @(negedge clk);href=1;session_valid=1;session_epoch=16'h5555;tick();check(dut.frame_sequence==1,"old physical SOF not sampled");
  set_request(0,16'h9999);tick();tick();check(dut.epoch==16'h5555&&dut.frame_sequence==0&&!dut.active,"SOF-coincident epoch did not abort");
  repeat(10)tick();check(dut.frame_sequence==0,"old SOF reused without new VSYNC");
  @(negedge clk);href=0;vsync=1;repeat(3)tick();@(negedge clk);vsync=0;repeat(2)tick();
  @(negedge clk);href=1;tick();check(dut.frame_sequence==1&&dut.active,"next healthy physical SOF not recovered");
  $display("PASS registered session pipeline checks=%0d changes=%0d: sampled bus isolation, valid gating, contiguous same/unique strobes, apply ordering, next physical VSYNC",checks,changes);
  $finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
