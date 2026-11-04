`timescale 1ns/1ps
module tb_sonar_capture_v25;
reg clk=0;always #5 clk=~clk;reg rst_n=0;reg[2:0] rx=7;wire valid;wire[511:0] packet;
robot_sensor_capture #(.CLK_FREQ(100000),.BAUD(10000)) dut(.clk(clk),.rst_n(rst_n),.encoder_a(4'd0),.encoder_b(4'd0),.sonar_rx(rx),.sensor_valid(valid),.sensor_packet(packet));
integer hex_fd,case_fd,case_id=0,frames=0,checks=0,j,age_checks=0,i;reg[511:0] latest;
reg[31:0] tick_stamp[0:2],last_source_id[0:2],expected_source_id;reg[2:0] have_id=0;
reg[31:0] source_id,advance;reg[15:0] sample_age;
always@(posedge clk)if(rst_n&&dut.tick_ms)begin
 tick_stamp[0]=dut.sonar.channel[0].u_driver.u_sensor.distance_stamp_ms;
 tick_stamp[1]=dut.sonar.channel[1].u_driver.u_sensor.distance_stamp_ms;
 tick_stamp[2]=dut.sonar.channel[2].u_driver.u_sensor.distance_stamp_ms;
end
always@(negedge clk)if(valid)begin
 latest=packet;frames=frames+1;
 if(packet[479:472]!=1||packet[7:0]!=0)$fatal(1,"SNS1 version/reserved byte changed");
 if((packet[471:464]|packet[463:456])!=7||(packet[471:464]&packet[463:456]))$fatal(1,"invalid SNS1 masks");
 for(j=0;j<3;j=j+1)if(packet[464+j])begin
  sample_age=packet[79-j*16 -:16];source_id=packet[415:384]-sample_age;
  if(sample_age>1000)$fatal(1,"online with adopted actual age over1000ms: ch%0d age%0d",j,sample_age);
  if(source_id!=tick_stamp[j])$fatal(1,"SNS age falsifies actual source: ch%0d expected%0d got%0d age%0d",j,tick_stamp[j],source_id,sample_age);
  if(have_id[j])begin advance=source_id-last_source_id[j];if(advance>=32'h80000000)$fatal(1,"actual source observation ID regressed");end
  have_id[j]=1;last_source_id[j]=source_id;age_checks=age_checks+1;
 end
 $fdisplay(hex_fd,"%0128h",packet);$fdisplay(case_fd,"%0d\t%0128h",case_id,packet);
end
task automatic ticks(input integer n);repeat(n)@(negedge clk);endtask
task automatic byte_in(input integer ch,input[7:0] b);integer i;begin @(negedge clk);rx[ch]=0;ticks(10);for(i=0;i<8;i=i+1)begin rx[ch]=b[i];ticks(10);end rx[ch]=1;ticks(10);end endtask
task automatic frame(input integer ch,input[15:0] mm,input bit bad);reg[7:0] cs;begin cs=8'hff+mm[15:8]+mm[7:0];byte_in(ch,8'hff);byte_in(ch,mm[15:8]);byte_in(ch,mm[7:0]);byte_in(ch,cs+bad);ticks(110);end endtask
task check(input bit condition,input[511:0] label);begin checks=checks+1;if(!condition)$fatal(1,"%0s valid=%b fault=%b front=%0d",label,latest[471:464],latest[463:456],latest[127:112]);end endtask
initial begin
 hex_fd=$fopen("reports/sonar-v25-sns.hex","w");case_fd=$fopen("reports/sonar-v25-sns-cases.tsv","w");
 ticks(5);rst_n=1;ticks(30);fork frame(0,1000,0);frame(1,2000,0);frame(2,3000,0);join
 check(latest[471:464]==7,"three inputs initially valid");
 case_id=1;frame(0,4501,0);check(latest[127:112]==1000,"first4501 not clear");frame(0,4501,0);check(latest[127:112]==4500&&latest[464],"4501pair valid4500");
 case_id=2;frame(0,1000,0);frame(0,6000,0);frame(0,6000,0);check(latest[127:112]==4500&&latest[464],"6000pair valid4500");
 case_id=3;frame(0,1000,0);frame(0,65534,0);frame(0,65534,0);check(latest[127:112]==4500&&latest[464],"65534pair valid4500");
 case_id=4;frame(0,0,0);check(!latest[464]&&latest[456],"zero invalid notfar");
 case_id=5;frame(0,4001,0);check(latest[127:112]==4001&&latest[464],"4001valid raw preserved");
 case_id=6;frame(0,65535,0);check(!latest[464]&&latest[456],"ffffinvalid notfar");
 case_id=7;frame(0,250,0);check(latest[127:112]==250&&latest[464],"near immediate afteroffline");
 case_id=8;frame(0,6000,1);ticks(5100);check(latest[127:112]==250,"bad checksum never publishedfar");
 case_id=9;ticks(100200);check(latest[471:464]==0&&latest[463:456]==7,"disconnected timeout all invalid");
 case_id=10;frame(0,250,0);frame(0,1000,0);expected_source_id=dut.sonar.channel[0].u_driver.u_sensor.candidate_stamp_ms;
 ticks(10000);frame(0,2000,0);
 check(latest[127:112]==1000&&latest[79:64]>=100&&latest[415:384]-latest[79:64]==expected_source_id,"actualSNS confirmed older candidate retains its100ms observation identity");
 frame(0,1000,0);check(latest[127:112]==1000&&latest[79:64]<=2&&latest[415:384]-latest[79:64]!=expected_source_id,"equal new actual sample genuinely refreshes observation time");
 case_id=11;frame(0,250,0);
 for(i=0;i<15;i=i+1)begin ticks(9600);frame(0,i%2?2400:1200,0);if(i>0)check(latest[464]&&latest[127:112]==1200,"alternating disparate far echoes do not starve valid output");end
 case_id=12;frame(0,250,0);ticks(10000);frame(0,1000,0);expected_source_id=dut.sonar.channel[0].u_driver.u_sensor.candidate_stamp_ms;
 ticks(50000);frame(0,2200,0);
 check(latest[464]&&latest[127:112]==1000&&latest[79:64]>=500&&latest[79:64]<=510&&latest[415:384]-latest[79:64]==expected_source_id,"500ms sparse UART confirmation publishes genuine old source age, not0");
 case_id=13;ticks(60000);check(!latest[464]&&latest[456],"sparse old actual sample expires despite recent confirmation receipt");
 $fclose(hex_fd);$fclose(case_fd);$display("PASS v25 realUART -> unchangedSNS1: %0d checks %0d frames %0d actualage/monotonicID invariants; variedfar no starvation, priorframe100ms and sparse500ms honestage, validage<=1000, clamp/invalid/CRC/timeout",checks,frames,age_checks);$finish;
end
initial begin #50000000;$fatal(1,"timeout");end
endmodule
