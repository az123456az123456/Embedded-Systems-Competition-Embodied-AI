`timescale 1ns/1ps
module tb_reactive_motor_v29;
reg clk=0,rst_n=0,valid=0;always #5 clk=~clk;
reg[127:0] command=0;reg[63:0] cpu_pwm={-16'sd200,16'sd200,-16'sd200,16'sd200};
reg ready=1,trap=0;reg[47:0]ranges={16'd250,16'd250,16'd250};reg[2:0]online=7;
wire[3:0]pwm,in1,in2,flags;wire[127:0]ack;wire[31:0]seq;
motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(50),.RAMP_STEP_MS(1),.STATUS_MS(5))dut(
.clk(clk),.rst_n(rst_n),.command_valid(valid),.command_data(command),.encoder_valid(1'b0),.encoder_deltas(128'd0),.encoder_illegal(4'd0),
.motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),.status_valid(),.status_data(ack),.last_sequence(seq),.applied_left(),.applied_right(),
.cpu_pwm(cpu_pwm),.cpu_ready(ready),.cpu_trap(trap),.raw_sonar_mm(ranges),.raw_sonar_online(online),.control_flags(),.control_epoch(),.control_sequence(),.control_motion(),.control_masks());
task send(input integer arm,input integer n,input integer v,input integer w);
begin @(negedge clk);command={32'h4d4f5434,8'd4,7'd0,arm[0],n[31:0],v[15:0],w[15:0],8'h0a,8'h0f};valid=1;@(negedge clk);valid=0;repeat(5)@(negedge clk);end endtask
task stopped;begin if(pwm!==0||in1!==0||in2!==0)$fatal(1,"raw protection must stop immediately");end endtask
task moving;integer j,c;begin c=0;repeat(250)@(negedge clk);for(j=0;j<100;j++)begin @(negedge clk);c+=|pwm;end if(c==0)$fatal(1,"clear requested direction must move");end endtask
initial begin
 repeat(4)@(negedge clk);rst_n=1;send(0,1,0,0);send(1,2,0,600);moving();
 if(!flags[0]||!flags[3])$fatal(1,"MOT4 authority/CPUready lost");
 ranges[31:16]=190;#1;stopped();if(!flags[0]||!flags[3])$fatal(1,"near side dearmed instead of blocking PWM");
 cpu_pwm={16'sd200,-16'sd200,16'sd200,-16'sd200};send(1,3,0,-600);moving();
 ranges[47:32]=190;#1;stopped();ranges[47:32]=250;ranges[31:16]=250;
 cpu_pwm={16'd200,16'd200,16'd200,16'd200};send(1,4,20,0);moving();
 ranges[15:0]=200;#1;stopped();ranges[15:0]=250;moving();
 online=6;#1;stopped();repeat(400)@(negedge clk);if(!flags[0]||!flags[3])$fatal(1,"temporary offline cleared authority/CPUready");
 online=7;moving();
 online=6;repeat(5200)@(negedge clk);stopped();if(flags[0]||!flags[1]||flags[2])$fatal(1,"sonar WAIT prolonged external lease");
 online=7;repeat(200)@(negedge clk);stopped(); // source recovery cannot rearm old lease
 send(0,1,0,0);send(1,2,0,600);cpu_pwm={-16'sd200,16'sd200,-16'sd200,16'sd200};moving();
 ready=0;#1;stopped();ready=1;moving();trap=1;#1;stopped();
 $display("PASS v29 raw200mm floor:250mm pureturn/lowdrive,190mm directional side and opposite scan;offline PWMzero preservesready/authorization,original externallease expires;CPUready/trap hardgate retained");$finish;
end
initial begin #5000000;$fatal(1,"timeout");end
endmodule
