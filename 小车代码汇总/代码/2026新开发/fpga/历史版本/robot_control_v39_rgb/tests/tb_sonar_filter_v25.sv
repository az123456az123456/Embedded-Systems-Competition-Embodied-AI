`timescale 1ns/1ps
module tb_sonar_filter_v25;
reg clk=0;always #5 clk=~clk;reg rst_n=0;reg[7:0] b=0;reg rv=0;
wire[15:0] mm,age;wire valid,online,fault,obstacle,slow_down,stop_req;
ultrasonic_sensor #(.CLK_FREQ(10000)) dut(.clk(clk),.rst_n(rst_n),.rx_byte(b),.rx_valid(rv),.distance_mm(mm),.distance_valid(valid),.distance_age_ms(age),.online(online),.sensor_fault(fault),.obstacle(obstacle),.slow_down(slow_down),.stop_req(stop_req));
integer checks=0,count=0,i,c,minimum,maximum;reg[31:0] stamp,previous_stamp;
always@(negedge clk)if(valid)count=count+1;
task ticks(input integer n);repeat(n)@(negedge clk);endtask
task ms(input integer n);ticks(n*10);endtask
task byte_in(input[7:0] v);begin @(negedge clk);b=v;rv=1;@(negedge clk);rv=0;end endtask
task frame(input[15:0] v,input bit bad);reg[7:0] cs;begin cs=8'hff+v[15:8]+v[7:0];byte_in(8'hff);byte_in(v[15:8]);byte_in(v[7:0]);byte_in(cs+bad);ticks(2);end endtask
task check(input bit condition,input[767:0] message);begin checks=checks+1;if(!condition)$fatal(1,"%0s mm=%0d age=%0d n=%0d online=%b",message,mm,age,count,online);end endtask
initial begin
 ticks(5);rst_n=1;ticks(3);check(fault&&stop_req&&age==65535,"startup unknown stops");
 frame(1000,0);check(mm==1000&&online&&age<=1,"first actual sample no warmup");
 minimum=65535;maximum=0;
 for(i=0;i<60;i=i+1)begin frame(i%2?1020:980,0);if(mm<minimum)minimum=mm;if(mm>maximum)maximum=mm;check(mm==980,"jitter output remains an actually observed conservative980, never IIR synthetic");ms(100);end
 check(maximum==minimum&&minimum==980,"rolling lower envelope rejects alternating40mm jitter");
 frame(250,0);stamp=dut.distance_stamp_ms;check(mm==250&&stop_req&&age<=1,"near immediate and actual source timestamp");ms(100);c=count;
 frame(1200,0);check(mm==250&&count==c&&dut.distance_stamp_ms==stamp&&age>=100,"one far never renews old near age");previous_stamp=dut.candidate_stamp_ms;
 ms(100);frame(2400,0);check(mm==1200&&online&&dut.distance_stamp_ms==previous_stamp&&age>=100,"widely differing far pair confirmed min with previous frame real timestamp");
 for(i=0;i<25;i=i+1)begin ms(100);frame(i%2?2400:1200,0);check(online&&mm==1200&&age<=102,"variable far surfaces cannot starve into stale/offline");end
 frame(250,0);frame(1000,0);previous_stamp=dut.candidate_stamp_ms;ms(100);frame(2000,0);check(mm==1000&&dut.distance_stamp_ms==previous_stamp,"rising range initial pair");
 ms(100);frame(3000,0);check(mm==2000&&age>=100&&age<=102,"rolling recession advances every frame, no mutual200mm gate");
 frame(300,0);frame(5000,0);ms(100);frame(5010,0);check(mm==4500&&online&&age<=1,"equal clamped far values use current actual timestamp");
 frame(4001,0);check(mm==4001&&online,"4001 preserved exactly");frame(4200,0);ms(20);frame(4300,0);check(mm==4200&&age>=20,"sub200mm farther value never synthesized by IIR");
 frame(0,0);check(fault&&stop_req,"zero invalid, no far inference");frame(65535,0);check(fault,"FFFF invalid, no far inference");
 frame(250,0);frame(0,0);frame(6000,0);check(!online&&mm==250,"invalid break and firstfar cannot clear near");frame(65534,0);check(online&&mm==4500,"two new numeric outlimit values recover");
 frame(250,0);frame(2000,0);frame(3000,1);frame(4000,0);check(mm==250,"CRC break requires two NEW far observations");frame(4200,0);check(mm==4000,"new pair after badCRC min real sample");
 frame(250,0);frame(2000,0);byte_in(8'hff);byte_in(8'h01);ms(21);frame(4000,0);check(mm==250,"partial timeout breaks confirmation");frame(4100,0);check(mm==4000,"new far pair after partial timeout");
 frame(250,0);stamp=dut.distance_stamp_ms;ms(100);frame(1000,0);previous_stamp=dut.candidate_stamp_ms;ms(500);frame(2200,0);
 check(mm==1000&&online&&dut.distance_stamp_ms==previous_stamp&&age>=500&&age<=502,"500ms sparse pair retains honest500ms age, never reportsfresh");
 ms(501);check(!online&&fault&&stop_req,"honest old adopted sample expires even though confirmation arrived later");
 frame(3000,0);check(!online&&mm==1000,"offline transition broke previous confirmation chain");frame(3500,0);check(online&&mm==3000,"two NEW far observations recover afteroffline");
 frame(250,0);ms(900);frame(2000,0);ms(101);frame(3000,0);check(!online&&mm==250,"timeout transition invalidates precrossing far candidate");frame(3500,0);check(online&&mm==3000,"posttimeout pair recovers with real sourceage");
 frame(250,0);frame(1000,0);ms(1001);frame(2000,0);check(mm==250&&!online,"expired candidate cannot confirm");frame(3000,0);check(mm==2000&&online,"new pair after candidate expiration");
 for(i=1;i<30;i=i+1)begin frame(i,0);check(!online&&fault&&stop_req,"blind zone never promoted to far");end
 frame(30,0);check(mm==30&&online,"minimum valid near immediate");
 @(negedge clk);dut.uptime_ms=32'hfffffff0;dut.distance_stamp_ms=32'hfffffff0;dut.far_pending=0;dut.online=1;
 frame(1000,0);previous_stamp=dut.candidate_stamp_ms;ms(100);frame(2000,0);
 check(mm==1000&&online&&age>=100&&age<=102&&dut.distance_stamp_ms==previous_stamp,"uint32 device time wrap preserves actual old candidate age");
 @(negedge clk);dut.distance_stamp_ms=dut.uptime_ms-32'd65534;ms(3);check(age==65535,"actual source age saturates instead of wrapping");
 $display("PASS sonar v25 %0d checks; rolling2 actual observations, no200mm starvation, immediate near, real priorframe/500ms age, no synthetic endpoints, invalid/CRC/timeout/offline chain breaks",checks);$finish;
end
initial begin #30000000;$fatal(1,"timeout");end
endmodule
