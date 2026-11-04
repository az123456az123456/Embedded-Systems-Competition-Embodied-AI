`timescale 1ns/1ps
module tb_encoder_pipeline_v28;
 reg clk=0,rst_n=0,enable=0,reverse=0,valid=0;reg [9:0] base=400;reg [127:0] deltas=0;reg [3:0] illegal=0;
 always #10 clk=~clk;
 wire [47:0] actual,expected;wire active,refactive,fault,reffault;
 motor_encoder_sync #(.CLK_HZ(1000),.SETTLE_SAMPLES(0)) dut(.clk(clk),.rst_n(rst_n),.enable(enable),.reverse(reverse),.base_duty(base),.sample_valid(valid),.deltas(deltas),.illegal(illegal),.trims(actual),.active(active),.fault(fault));
 motor_encoder_sync_v26_reference #(.CLK_HZ(1000),.SETTLE_SAMPLES(0)) reference(.clk(clk),.rst_n(rst_n),.enable(enable),.reverse(reverse),.base_duty(base),.sample_valid(valid),.deltas(deltas),.illegal(illegal),.trims(expected),.active(refactive),.fault(reffault));
 integer cycle=0,checks=0,delayed=0,i,seed=28;reg [31:0] r;
 always @(negedge clk)if(rst_n)begin
   if(dut.pending==3 && reference.pending==0 && enable && !illegal)delayed++;
   else begin
     if({actual,active,fault}!={expected,refactive,reffault})$fatal(1,"pipeline altered encoder arithmetic/safety atcycle%0d new%h/%b/%b expected%h/%b/%b",cycle,actual,active,fault,expected,refactive,reffault);
     checks++;
   end
   cycle++;
 end
 initial begin repeat(3)@(posedge clk);#1;rst_n=1;enable=1;
 for(i=0;i<2000;i++)begin
   repeat(45)@(posedge clk);#1;r=$random(seed);valid=1;
   deltas={32'd1000+(r[7:0]),32'd1000+(r[15:8]),32'd1000+(r[23:16]),32'd1000+(r[31:24])};
   illegal=i%77==0?1:0;if(i%99==0)reverse=~reverse;if(i%101==0)base=300+(i%200);
   @(posedge clk);#1;valid=0;illegal=0;
 end
 enable=0;repeat(10)@(posedge clk);#1;
 if(delayed<100)$fatal(1,"pipeline alignmentnotexercised");
 $display("PASS encoder v28 vs frozenv26 arithmetic/safety equality: %0d checks, %0d preciselyonecycle delayed trim updates; real50MHz extra20ns",checks,delayed);$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
