`timescale 1ns/1ps
module tb_motor_legacy_equivalence_v21;
 reg clk=0,rst_n=0,valid=0;reg[127:0] data=0;reg encvalid=0;reg[127:0] deltas=0;reg[3:0] illegal=0;
 wire[3:0] pwm[0:1],in1[0:1],in2[0:1],flags[0:1];wire sv[0:1];wire[127:0] status[0:1];wire[31:0] seqno[0:1];wire signed[15:0] left[0:1],right[0:1];
 motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(10),.RAMP_STEP_MS(1),.REVERSAL_MS(1),.ENCODER_SYNC_ENABLE(1)) dut(
 .clk(clk),.rst_n(rst_n),.command_valid(valid),.command_data(data),.encoder_valid(encvalid),.encoder_deltas(deltas),.encoder_illegal(illegal),.motor_pwm(pwm[0]),.motor_in1(in1[0]),.motor_in2(in2[0]),.status_flags(flags[0]),.status_valid(sv[0]),.status_data(status[0]),.last_sequence(seqno[0]),.applied_left(left[0]),.applied_right(right[0]));
 motor_remote_control_v20_reference #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(10),.RAMP_STEP_MS(1),.REVERSAL_MS(1),.ENCODER_SYNC_ENABLE(1)) refdut(
 .clk(clk),.rst_n(rst_n),.command_valid(valid),.command_data(data),.encoder_valid(encvalid),.encoder_deltas(deltas),.encoder_illegal(illegal),.motor_pwm(pwm[1]),.motor_in1(in1[1]),.motor_in2(in2[1]),.status_flags(flags[1]),.status_valid(sv[1]),.status_data(status[1]),.last_sequence(seqno[1]),.applied_left(left[1]),.applied_right(right[1]));
 always #5 clk=~clk;
 integer cycles=0,i,n,v,seed=32'h13821;reg[31:0] seqin=0,r;reg signed[15:0] x,y;reg[3:0] mask,inv;reg arm;
 always @(negedge clk)if(rst_n)begin
 if({pwm[0],in1[0],in2[0],flags[0],sv[0],status[0],seqno[0],left[0],right[0]}!=={pwm[1],in1[1],in2[1],flags[1],sv[1],status[1],seqno[1],left[1],right[1]})$fatal(1,"legacy mismatch at cycle%0d",cycles);
 cycles=cycles+1;end
 task send(input integer version,input bit a,input signed[15:0] a0,a1,input[3:0] m,z);
 begin @(posedge clk);#1;data={24'h4d4f54,8'h30+version[7:0],version[7:0],7'b0,a,seqin,a0,a1,4'b0,m,4'b0,z};valid=1;@(posedge clk);#1;valid=0;end endtask
 initial begin
 repeat(4)@(posedge clk);#1;rst_n=1;
 for(i=0;i<30000;i=i+1)begin
 r=$random(seed);v=r[0]?1:2;arm=r[1:0]!=0;mask=r[3:0];inv=r[7:4];
 if(i%100==0)begin seqin=seqin+1;send(v,0,0,0,12,15);end
 if(r[8:7]!=0)seqin=seqin+1;
 if(v==1)begin x=$signed(r[31:16])%1200;y=$signed(r[15:0])%1200;end
 else begin x=$signed(r[31:16])%10000;y=$signed(r[15:0])%260;end
 if(i%4!=0)begin mask=12;inv=15;end
 if(i%8==0)begin x=0;y=0;end
 send(v,arm,x,y,mask,inv);
 n=r[16:9]%300;repeat(n)begin @(posedge clk);#1;encvalid=0;if(n%50==0)begin encvalid=1;deltas={$random(seed),$random(seed),$random(seed),$random(seed)};illegal=r[3:0];end end
 end
 valid=0;repeat(1200)@(posedge clk);
 $display("PASS MOT1/MOT2 v20 reference cycle equivalence %0d cycles; 30000 randomized commands, encoder feedback, invalid/replayed packets, watchdog and status",cycles);$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
