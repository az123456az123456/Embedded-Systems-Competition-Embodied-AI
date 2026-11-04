`timescale 1ns/1ps
module tb_motor_arc_v21;
 reg clk=0,rst_n=0,valid=0; reg [127:0] data=0;
 wire [3:0] pwm,in1,in2,flags;wire [127:0] status;wire [31:0] lastseq;
 wire signed [15:0] left,right;
 integer seqno=0,checks=0,s,c,k,i,j,reduction,inner,el,er; reg [3:0] observed;
 motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(80),.RAMP_STEP_MS(1),.REVERSAL_MS(1)) dut(
 .clk(clk),.rst_n(rst_n),.command_valid(valid),.command_data(data),.encoder_valid(1'b0),.encoder_deltas(128'b0),.encoder_illegal(4'b0),
 .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),.status_valid(),.status_data(status),.last_sequence(lastseq),.applied_left(left),.applied_right(right));
 always #5 clk=~clk;
 task clocks(input integer n);begin repeat(n)@(negedge clk);end endtask
 task command(input integer version,input bit arm,input signed[15:0] speed,curve,input [3:0] mask,inv);
 begin @(negedge clk);data={24'h4d4f54,8'h30+version[7:0],version[7:0],7'b0,arm,seqno[31:0],speed,curve,4'b0,mask,4'b0,inv};valid=1;
 @(negedge clk);valid=0;clocks(3);end endtask
 task fresh(input bit arm,input signed[15:0] speed,curve);
 begin seqno=seqno+1;command(3,arm,speed,curve,12,15);end endtask
 task check_target(input integer speed,curve);
 begin
   fresh(1,speed,curve);
   reduction=((speed<0?-speed:speed)*(curve<0?-curve:curve))/512;
   inner=speed<0?speed+reduction:speed-reduction;
   el=curve<0?inner:speed;er=curve<0?speed:inner;
   if(lastseq!==seqno || $signed(dut.target[0])!==el || $signed(dut.target[1])!==er)
     $fatal(1,"MOT3 mismatch speed=%0d curve=%0d seq=%0d/%0d L=%0d/%0d R=%0d/%0d",speed,curve,lastseq,seqno,$signed(dut.target[0]),el,$signed(dut.target[1]),er);
   if(status[127:96]!==32'h41434b33 || status[95:88]!==3)$fatal(1,"ACK3 identity");
   checks=checks+1;
 end endtask
 task reject(input bit arm,input integer speed,curve);
 integer oldseq;
 begin oldseq=lastseq;fresh(arm,speed,curve);if(lastseq!==oldseq)$fatal(1,"invalid accepted s=%0d c=%0d arm=%0d",speed,curve,arm);checks=checks+1;end endtask
 task settle(input integer speed,curve,input integer expected_left,expected_right);
 begin
   fresh(0,0,0); // deterministic complete zero before each physical test
   for(k=0;k<8;k=k+1)begin fresh(1,speed,curve);clocks(1000);end
   if(left!==expected_left || right!==expected_right)$fatal(1,"applied arc s=%0d c=%0d L=%0d/%0d R=%0d/%0d",speed,curve,left,expected_left,right,expected_right);
   if($signed(status[47:32])!==expected_left || $signed(status[31:16])!==expected_right)$fatal(1,"ACK3 does not report physical sides");
   observed=0;for(j=0;j<100;j=j+1)begin clocks(1);observed=observed|pwm;end
   if(observed!==15)$fatal(1,"both wheels must run during every normal arc");
   if(speed>0 && (in1!==0 || in2!==15))$fatal(1,"forward configured inversion");
   if(speed<0 && (in1!==15 || in2!==0))$fatal(1,"reverse configured inversion");
 end endtask
 initial begin
  clocks(3);rst_n=1;clocks(3);
  fresh(1,600,0);if(flags[0] || pwm!==0)$fatal(1,"MOT3 armed before STOP");
  fresh(0,0,0);if(flags!==4'b0100)$fatal(1,"MOT3 STOP handshake");
  // Exhaustively compare every legal integer speed/continuous Q8 curvature.
  for(s=-600;s<=600;s=s+1)for(c=-256;c<=256;c=c+1)check_target(s,c);
  reject(1,-601,0);reject(1,601,0);reject(1,-32768,0);reject(1,32767,0);
  reject(1,0,-257);reject(1,0,257);reject(1,0,-32768);reject(1,0,32767);
  reject(0,0,1);reject(0,1,0);reject(0,1,1);
  fresh(0,0,0);seqno=seqno+1;command(3,1,400,0,10,15);if(lastseq==seqno)$fatal(1,"armed mapping changed");
  // All eight joystick regions; left and right are path-relative in reverse.
  settle(600,0,600,600);    // forward
  settle(-600,0,-600,-600);// backward
  settle(600,-256,300,600);// left (forward arc)
  settle(600,256,600,300); // right (forward arc)
  settle(450,-181,291,450);// forward-left
  settle(450,181,450,291); // forward-right
  settle(-450,-181,-291,-450);// backward-left: RIGHT wheels reverse faster
  settle(-450,181,-450,-291);// backward-right: LEFT wheels reverse faster
  settle(600,91,600,494);settle(-600,-91,-494,-600);
  fresh(1,0,256);if(pwm!==0 || left!==0 || right!==0)$fatal(1,"zero speed must stop even nonzero curvature");
  // Shared sequence semantics across all versions; replay does not refresh.
  fresh(1,400,0);i=lastseq;command(1,1,600,600,12,15);if(lastseq!==i)$fatal(1,"MOT1 replay accepted after MOT3");
  command(2,1,0,100,12,15);if(lastseq!==i)$fatal(1,"MOT2 replay accepted after MOT3");
  for(k=0;k<10;k=k+1)begin command(3,1,400,0,12,15);clocks(1000);end
  if(pwm!==0 || !flags[1] || flags[2] || flags[0])$fatal(1,"MOT3 duplicate-only watchdog");
  fresh(1,400,0);if(flags[0])$fatal(1,"MOT3 rearm bypassed STOP");
  fresh(0,0,0);settle(-600,-256,-300,-600);fresh(0,0,0);
  if(pwm!==0 || left!==0 || right!==0 || flags[0])$fatal(1,"MOT3 immediate STOP");
  $display("PASS MOT3 %0d exhaustive integer speed/curvature golden cases, eight directions, reverse-left/right path convention, both-side PWM, physical ACK3, bounds/config/zero/replay/watchdog/rearm",checks);$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
