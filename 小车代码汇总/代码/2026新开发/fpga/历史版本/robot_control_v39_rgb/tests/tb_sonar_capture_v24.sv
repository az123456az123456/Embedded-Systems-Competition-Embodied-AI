`timescale 1ns/1ps
module tb_sonar_capture_v24;
reg clk=0;always #5 clk=~clk;reg rst_n=0;reg[2:0] rx=7;wire valid;wire[511:0] packet;
robot_sensor_capture #(.CLK_FREQ(100000),.BAUD(10000)) dut(.clk(clk),.rst_n(rst_n),.encoder_a(4'd0),.encoder_b(4'd0),.sonar_rx(rx),.sensor_valid(valid),.sensor_packet(packet));
integer hex_fd,case_fd,case_id=0,frames=0,checks=0;reg[511:0] latest;
always@(negedge clk)if(valid)begin latest=packet;frames=frames+1;$fdisplay(hex_fd,"%0128h",packet);$fdisplay(case_fd,"%0d\t%0128h",case_id,packet);end
task automatic ticks(input integer n);repeat(n)@(negedge clk);endtask
task automatic byte_in(input integer ch,input[7:0] b);integer i;begin @(negedge clk);rx[ch]=0;ticks(10);for(i=0;i<8;i=i+1)begin rx[ch]=b[i];ticks(10);end rx[ch]=1;ticks(10);end endtask
task automatic frame(input integer ch,input[15:0] mm,input bit bad);reg[7:0] cs;begin cs=8'hff+mm[15:8]+mm[7:0];byte_in(ch,8'hff);byte_in(ch,mm[15:8]);byte_in(ch,mm[7:0]);byte_in(ch,cs+bad);ticks(110);end endtask
task check(input bit condition,input[511:0] label);begin checks=checks+1;if(!condition)$fatal(1,"%0s valid=%b fault=%b front=%0d",label,latest[471:464],latest[463:456],latest[127:112]);end endtask
initial begin
 hex_fd=$fopen("reports/sonar-v24-sns.hex","w");case_fd=$fopen("reports/sonar-v24-sns-cases.tsv","w");
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
 $fclose(hex_fd);$fclose(case_fd);$display("PASS v24 realUART -> unchangedSNS1: %0d checks %0d frames;4501/6000/65534 valid4500,0/FFFF invalid,badCRC keepsonlylastfreshrealvalue,timeoutunknown",checks,frames);$finish;
end
initial begin #20000000;$fatal(1,"timeout");end
endmodule
