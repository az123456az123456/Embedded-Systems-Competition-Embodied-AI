`timescale 1ns/1ps
module tb_sonar_filter_v24;
reg clk=0;always #5 clk=~clk;reg rst_n=0;reg[7:0] b=0;reg rv=0;
wire[15:0] mm;wire valid,online,fault,obstacle,slow_down,stop_req;
ultrasonic_sensor #(.CLK_FREQ(10000)) dut(.clk(clk),.rst_n(rst_n),.rx_byte(b),.rx_valid(rv),.distance_mm(mm),.distance_valid(valid),.online(online),.sensor_fault(fault),.obstacle(obstacle),.slow_down(slow_down),.stop_req(stop_req));
integer checks=0,count=0,i,minimum,maximum,c;reg[31:0] age_before;
always@(negedge clk)if(valid)count=count+1;
task ticks(input integer n);repeat(n)@(negedge clk);endtask
task byte_in(input[7:0] v);begin @(negedge clk);b=v;rv=1;@(negedge clk);rv=0;end endtask
task frame(input[15:0] v,input bit bad);reg[7:0] cs;begin cs=8'hff+v[15:8]+v[7:0];byte_in(8'hff);byte_in(v[15:8]);byte_in(v[7:0]);byte_in(cs+bad);ticks(2);end endtask
task check(input bit condition,input[511:0] message);begin checks=checks+1;if(!condition)$fatal(1,"%0s mm=%0d n=%0d online=%b",message,mm,count,online);end endtask
initial begin
 ticks(5);rst_n=1;ticks(3);check(fault&&stop_req,"startup unknown stops");
 frame(1000,0);check(online&&mm==1000&&count==1,"first real valid value no warmup");minimum=65535;maximum=0;
 for(i=0;i<60;i=i+1)begin frame(i%2?1020:980,0);if(mm<minimum)minimum=mm;if(mm>maximum)maximum=mm;ticks(1000);end
 check(maximum-minimum<=10,"quarter IIR reduces raw40mm jitter to10mm");
 c=count;frame(250,0);check(mm==250&&count==c+1&&stop_req,"near one frame immediate");ticks(100);age_before=dut.data_age;c=count;
 frame(2000,0);check(mm==250&&count==c&&dut.data_age>age_before,"single far held and no freshness renewal");
 frame(2040,0);check(mm==2000&&count==c+1&&online&&!stop_req,"consistent far pair nearer wins");
 c=count;frame(4501,0);check(mm==2000&&count==c,"numeric outlimit first cannot clear nearer value");
 frame(65534,0);check(mm==4500&&count==c+1&&online,"different numeric outlimits clamp before confirmation");
 frame(6000,0);check(mm==4500&&online,"numeric6000 valid clamp4500");frame(4001,0);check(mm==4001&&online,"valid4001 remains exact");frame(4200,0);frame(4200,0);check(mm==4200&&online,"valid4200 exact after large far confirmation");
 frame(0,0);check(fault&&stop_req&&mm==4200,"zero is invalid not far");frame(65535,0);check(fault&&mm==4200,"FFFF invalid not far");
 frame(250,0);check(mm==250&&online&&stop_req,"near offline recovery immediate");
 frame(0,0);check(fault,"invalid frame puts offline");c=count;frame(6000,0);check(mm==250&&!online&&count==c,"offline preserves near reference against one far spike");frame(6001,0);check(mm==4500&&online&&count==c+1,"offline consistent far pair recovers");
 frame(200,0);ticks(10020);check(fault&&stop_req&&mm==200,"disconnection timeout no fake far");c=count;frame(5000,0);check(mm==200&&!online&&count==c,"timeout preserves near anchor");ticks(5000);frame(5010,0);check(mm==4500&&online&&count==c+1,"candidate survives500ms processed mode while data stale");
 frame(200,0);frame(5000,0);ticks(10020);c=count;frame(5000,0);check(mm==200&&!online&&count==c,"expired far candidate cannot confirm");frame(5000,0);check(mm==4500&&online,"two new far recover after candidate expiry");
 frame(200,0);frame(5000,0);frame(5000,1);frame(5000,0);check(mm==200,"checksum failure breaks far confirmation");frame(5000,0);check(mm==4500,"fresh pair after checksum failure");
 frame(200,0);frame(5000,0);byte_in(8'hff);byte_in(8'h01);ticks(210);frame(5000,0);check(mm==200,"partial frame timeout breaks candidate");frame(5000,0);check(mm==4500,"fresh pair after partial timeout");
 for(i=1;i<30;i=i+1)begin frame(i,0);check(!online&&fault&&stop_req,"blind zone never promoted to far");end
 frame(30,0);check(mm==30&&online&&stop_req,"valid minimum immediate");
 $display("PASS sonar v24 %0d checks; raw jitter40mm filtered span%0dmm; immediate near, quarter far, numeric outlimit clamp, invalid/timeout/offline recovery",checks,maximum-minimum);$finish;
end
initial begin #30000000;$fatal(1,"timeout");end
endmodule
