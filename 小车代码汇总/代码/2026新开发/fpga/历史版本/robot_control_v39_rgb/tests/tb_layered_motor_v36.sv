`timescale 1ns/1ps
module tb_layered_motor_v28;
 reg clk=0,rst_n=0,valid=0;reg [127:0] command=0;
 always #5 clk=~clk;
 reg [63:0] cpu_pwm={16'd600,16'd450,16'd300,16'd150};
 reg cpu_ready=0,cpu_trap=0;reg [47:0] ranges={16'd1000,16'd1000,16'd1000};reg [2:0] online=7;
 wire [3:0] pwm,in1,in2,flags;wire [127:0] ack;wire [31:0] seq,epoch,mseq,motion;wire [4:0] ctrl;wire [7:0] masks;
 wire signed [15:0] left,right;
 motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(50),.RAMP_STEP_MS(1),.STATUS_MS(5)) dut(
 .clk(clk),.rst_n(rst_n),.command_valid(valid),.command_data(command),.encoder_valid(1'b0),.encoder_deltas(128'd0),.encoder_illegal(4'd0),
 .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),.status_valid(),.status_data(ack),.last_sequence(seq),.applied_left(left),.applied_right(right),
 .cpu_pwm(cpu_pwm),.cpu_ready(cpu_ready),.cpu_trap(cpu_trap),.raw_sonar_mm(ranges),.raw_sonar_online(online),
 .control_flags(ctrl),.control_epoch(epoch),.control_sequence(mseq),.control_motion(motion),.control_masks(masks));
 task send(input integer proto,input integer armed,input integer n,input integer a,input integer b,input [3:0] mask,input [3:0] invert);
 begin @(negedge clk);command={24'h4d4f54,8'h30+proto[7:0],proto[7:0],7'd0,armed[0],n[31:0],a[15:0],b[15:0],4'b0,mask,4'b0,invert};valid=1;
 @(negedge clk);valid=0;repeat(5)@(negedge clk);end endtask
 task stopped;begin if(pwm!==0 || in1!==0 || in2!==0)$fatal(1,"expected immediate all-wheel stop pwm=%b in=%b,%b",pwm,in1,in2);end endtask
 task count_pwm(input integer p1,input integer p2,input integer p3,input integer p4);
 integer j,c1,c2,c3,c4;begin c1=0;c2=0;c3=0;c4=0;repeat(200)@(negedge clk);
 for(j=0;j<100;j=j+1)begin @(negedge clk);c1+=pwm[0];c2+=pwm[1];c3+=pwm[2];c4+=pwm[3];end
 if(c1!=p1 || c2!=p2 || c3!=p3 || c4!=p4)$fatal(1,"PWM counts %0d,%0d,%0d,%0d expected %0d,%0d,%0d,%0d",c1,c2,c3,c4,p1,p2,p3,p4);end endtask
 initial begin repeat(3)@(negedge clk);rst_n=1;
 send(4,1,1,300,0,10,15);stopped;if(seq!=0)$fatal(1,"arming without configure");
 send(4,0,1,0,0,10,15);stopped;if(ack[127:96]!=32'h41434b34 || ack[95:88]!=4 || flags!=4)$fatal(1,"ACK4 capability before ready");
 send(4,1,2,300,0,10,15);stopped;cpu_ready=1;count_pwm(15,30,45,60);
 if(left!=450 || right!=300 || in1!=0 || in2!=15 || flags!=13)$fatal(1,"physical mapping/ACK4 flags");
 send(4,1,3,301,0,10,15);if(seq!=2)$fatal(1,"out-of-range v accepted");
 send(4,1,3,300,1001,10,15);if(seq!=2)$fatal(1,"out-of-range w accepted");
 send(4,1,2,0,0,10,15);if(seq!=2)$fatal(1,"replay accepted");
 send(4,1,3,0,0,10,15);stopped;send(4,1,4,300,0,10,15);count_pwm(15,30,45,60);
 cpu_ready=0;#1;stopped;if(flags[0]!=1 || flags[3]!=0)$fatal(1,"CPU loss changed external authorization");
 cpu_ready=1;repeat(200)@(negedge clk);cpu_trap=1;#1;stopped;cpu_trap=0;
 online=6;#1;stopped;if(flags[3]!=1 || flags[0]!=1)$fatal(1,"sonar loss changes cpu-ready/armed");online=7;
 ranges[15:0]=199;#1;stopped;send(4,1,5,0,-500,10,15);count_pwm(15,30,45,60); // front obstacle permits right scan
 ranges[47:32]=199;#1;stopped;ranges[47:32]=1000;ranges[15:0]=1000;
 send(4,1,6,-300,0,10,15);cpu_pwm={-16'sd200,-16'sd200,-16'sd200,-16'sd200};
 repeat(40)@(negedge clk);stopped;repeat(220)@(negedge clk);count_pwm(20,20,20,20);
 if(in1!=15 || in2!=0 || left!=-200 || right!=-200)$fatal(1,"reverse deadtime/invert");
 send(1,1,7,200,200,10,15);if(ack[95:88]!=1 || seq!=7)$fatal(1,"shared sequence protocol switch");
 send(4,1,6,300,0,10,15);if(seq!=7)$fatal(1,"cross protocol stale command");
 send(4,1,8,300,0,10,15);cpu_pwm={16'd200,16'd200,16'd200,16'd200};repeat(1000)@(negedge clk);
 send(4,1,8,300,0,10,15);repeat(5100)@(negedge clk);stopped;if(!flags[0] || !flags[1] || !flags[2])$fatal(1,"externalwatchdog replay refreshed");
 send(4,1,7,300,0,10,15);stopped;send(4,0,9,0,0,10,15);send(4,1,10,300,0,10,15);count_pwm(20,20,20,20);
 $display("PASS v28 MOT4 bounds/config/sharedsequence/watchdog; independent4PWM/physicalinvert/deadtime/ACK4; CPUtrap/ready/rawnear/invalid gates; zeroPWMstop");$finish;end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
