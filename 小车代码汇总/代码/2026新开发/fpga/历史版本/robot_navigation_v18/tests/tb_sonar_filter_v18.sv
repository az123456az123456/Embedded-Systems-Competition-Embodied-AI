`timescale 1ns/1ps
module tb_sonar_filter_v18;
reg clk=0;always #5 clk=~clk;
reg rst_n=0;reg [7:0] rx_byte=0;reg rx_valid=0;
wire [15:0] mm;wire valid,online,fault,obstacle,slow_down,stop_req;
ultrasonic_sensor #(.CLK_FREQ(10000)) dut(.clk(clk),.rst_n(rst_n),.rx_byte(rx_byte),.rx_valid(rx_valid),
.distance_mm(mm),.distance_valid(valid),.online(online),.sensor_fault(fault),.obstacle(obstacle),.slow_down(slow_down),.stop_req(stop_req));
integer count=0,checks=0,i,raw_min,raw_max,out_min,out_max,c;reg [31:0] old_age;
always @(negedge clk) if(valid)count=count+1;
task cycles(input integer n);repeat(n)@(negedge clk);endtask
task byte_in(input [7:0] b);begin @(negedge clk);rx_byte=b;rx_valid=1;@(negedge clk);rx_valid=0;end endtask
task frame(input [15:0] d,input bit bad);reg [7:0] cs;begin
cs=8'hff+d[15:8]+d[7:0];byte_in(8'hff);byte_in(d[15:8]);byte_in(d[7:0]);byte_in(cs+bad);cycles(2);end endtask
task check(input bit condition,input [511:0] message);begin checks=checks+1;if(!condition)$fatal(1,"%0s (mm=%0d valid_count=%0d)",message,mm,count);end endtask
initial begin
cycles(5);rst_n=1;cycles(3);check(fault&&stop_req,"reset fault");
frame(1000,0);check(mm==1000&&online&&count==1,"first observation no warmup delay");
raw_min=980;raw_max=1020;out_min=65535;out_max=0;
for(i=0;i<60;i=i+1)begin frame(i%2?1020:980,0);if(mm<out_min)out_min=mm;if(mm>out_max)out_max=mm;cycles(1000);end
check(out_max-out_min<=(raw_max-raw_min)/2,"alternating jitter peak-to-peak halved");
c=count;frame(250,0);check(count==c+1&&mm==250&&stop_req,"near step immediate, one frame");
cycles(300);old_age=dut.data_age;c=count;
frame(2000,0);check(mm==250&&count==c&&dut.data_age>old_age,"single far spike held without timestamp renewal");
cycles(1000);frame(2040,0);check(mm==2000&&count==c+1&&!stop_req,"two consistent far observations clear using nearer of pair");
c=count;old_age=dut.data_age;frame(2500,0);frame(3000,0);frame(3500,0);
check(mm==2000&&count==c&&dut.data_age>old_age,"inconsistent far spikes never revive old observation");
frame(500,0);check(mm==500&&count==c+1,"near return cancels far candidate");
c=count;frame(2000,0);frame(2010,1);frame(2000,0);check(count==c&&mm==500,"checksum fault breaks far confirmation chain");
frame(2000,0);check(mm==2000&&count==c+1,"fresh far pair recovers");
frame(4500,0);frame(4500,0);check(mm==4500&&online,"maximum in-range no arithmetic wrap");
frame(65535,0);check(!online&&fault&&stop_req&&mm==4500,"65535 invalid does not wrap into range");
frame(30,0);check(mm==30&&online&&stop_req,"minimum range and invalid recovery");
frame(29,0);check(!online,"below minimum invalid");frame(4501,0);check(!online,"above maximum invalid");
frame(1000,0);c=count;frame(2000,0);byte_in(8'hff);byte_in(8'h08);cycles(210);frame(2000,0);
check(count==c&&mm==1000,"partial frame timeout breaks confirmation");
frame(2000,0);check(mm==2000,"timeout recovery far pair");
c=count;cycles(10010);check(!online&&fault&&stop_req,"stale range invalidated");
check(count==c,"timeout emits no fake measurement");frame(3333,0);check(mm==3333&&online,"reconnect first valid observation no stale filter state");
// A 0.3m/s retreat can change 150mm per500ms in processed-output mode.
// Clearance eventually releases at the second far observation. With500ms
// processed output, the old estimate may conservatively time out in between.
frame(1000,0);cycles(5000);frame(1150,0);cycles(5000);frame(1300,0);
check(online&&mm>=1150&&mm<=1300,"500ms-mode retreat150mm per sample releases by second observation");
frame(3333,0);frame(3333,0);
for(i=0;i<50;i=i+1)begin frame(65535,0);frame(0,0);end
check(!online&&mm==3333,"repeated invalid unsigned extremes retain last distance but not validity");
$display("PASS sonar v18: %0d checks, 60 jitter samples raw span40mm filtered span%0dmm, immediate near step, conservative far validation, checksum/range/timeout/overflow",checks,out_max-out_min);$finish;
end
initial begin #20000000;$fatal(1,"timeout");end
endmodule
