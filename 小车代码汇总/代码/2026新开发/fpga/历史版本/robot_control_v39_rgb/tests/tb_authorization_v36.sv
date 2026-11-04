`timescale 1ns/1ps
// Exactly 1 kHz parameter/clock accelerates 30 s to 30000 edges. The RTL
// counter and all real protocol/sequence predicates are exercised unchanged.
module tb_authorization_v36;
 reg clk=0,rst_n=0,command_valid=0;
 reg [127:0] command_data=0;
 always #500000 clk=~clk;
 wire [3:0] motor_pwm,motor_in1,motor_in2,status_flags;
 wire signed [15:0] applied_left,applied_right;
 wire [31:0] last_sequence;
 motor_remote_control #(.CLK_HZ(1000),.PWM_HZ(100),.RAMP_STEP_MS(1)) dut(
  .clk(clk),.rst_n(rst_n),.command_valid(command_valid),.command_data(command_data),
  .motor_pwm(motor_pwm),.motor_in1(motor_in1),.motor_in2(motor_in2),
  .status_flags(status_flags),.last_sequence(last_sequence),
  .applied_left(applied_left),.applied_right(applied_right));
 integer checks=0;
 task tick;begin @(negedge clk);end endtask
 task check;input bit condition;input [511:0] message;begin
  checks=checks+1;if(!condition)$fatal(1,"%0s count=%0d flags=%h",message,dut.watchdog_count,status_flags);
 end endtask
 task send;input[31:0] seq;input bit armed;input[15:0] l,r;input[31:0] magic;
 begin
  tick();command_data={magic,8'd1,7'd0,armed,seq,l,r,8'd10,8'd15};command_valid=1;
  tick();command_valid=0;tick();
 end endtask
 task accepted;input[31:0] seq;input bit armed;input[15:0] l,r;begin
  send(seq,armed,l,r,32'h4d4f5431);
  check(last_sequence==seq,"accepted current sequence");
  check(dut.watchdog_count==0,"accepted command resets authority immediately");
 end endtask
 task until_count;input[31:0] count;begin
  while(dut.watchdog_count<count)tick();
 end endtask
 initial begin
  repeat(4)tick();rst_n=1;tick();
  accepted(1,0,0,0);accepted(2,1,600,600);
  until_count(499);check(status_flags[0]&&!status_flags[1]&&applied_left>0,"499ms retains motion");
  tick();check(status_flags[0]&&status_flags[2]&&status_flags[1],"500ms pauses but retains armed/configured");
  check(motor_pwm==0&&applied_left==0&&applied_right==0,"short transport fault still immediately stops outputs");
  until_count(29999);check(status_flags[0],"29999ms remains armed");
  tick();check(!status_flags[0]&&!status_flags[2]&&status_flags[1],"30000ms cancels authorization");
  // Expiry cannot be undone by an armed packet without safe configuration.
  send(3,1,600,600,32'h4d4f5431);check(!status_flags[0]&&!status_flags[2],"no unconfigured armed recovery");
  accepted(4,0,0,0);accepted(5,1,300,300);
  until_count(29000);accepted(6,1,0,0);
  until_count(29999);check(status_flags[0],"new valid idle gets full fresh30s");
  tick();check(!status_flags[0],"idle authority expires after its own30s");
  accepted(7,0,0,0);accepted(8,1,400,400);
  until_count(1000);
  send(8,1,400,400,32'h4d4f5431);
  check(dut.watchdog_count>1000&&last_sequence==8,"duplicate cannot reset timer");
  send(9,1,400,400,32'h42414431);
  check(dut.watchdog_count>1000&&last_sequence==8,"invalid protocol cannot reset timer");
  send(9,1,1001,400,32'h4d4f5431);
  check(dut.watchdog_count>1000&&last_sequence==8,"invalid duty cannot reset timer");
  until_count(29999);tick();check(!status_flags[0],"rejected traffic cannot keep authority alive");
  accepted(10,0,0,0);accepted(11,1,300,300);
  repeat(80)tick();accepted(12,0,0,0);
  check(!status_flags[0]&&motor_pwm==0&&applied_left==0,"explicit stop is immediate");
  accepted(13,1,300,300);until_count(700);
  accepted(14,1,0,0);
  check(status_flags[0]&&!status_flags[1]&&applied_left==0,"fresh authorized zero resumes only idle after pause");
  repeat(100)tick();check(applied_left==0&&motor_pwm==0,"old throttle never replays from a zero command");
  $display("PASS v36 authority checks=%0d short_pause_ms=500 authority_ms=30000 clock_hz=1000",checks);$finish;
 end
 initial begin #200000000000;$fatal(1,"timeout");end
endmodule
