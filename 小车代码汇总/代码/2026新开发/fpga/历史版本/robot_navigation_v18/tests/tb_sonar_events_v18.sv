`timescale 1ns/1ps
module tb_sonar_events_v18;
reg clk=0;always #5 clk=~clk;reg rst_n=0;reg [3:0] ea=0,eb=0;reg [2:0] rx=7;
wire valid;wire [511:0] packet;
robot_sensor_capture #(.CLK_FREQ(100000),.BAUD(10000)) dut(.clk(clk),.rst_n(rst_n),.encoder_a(ea),.encoder_b(eb),.sonar_rx(rx),.sensor_valid(valid),.sensor_packet(packet));
integer trace_fd;
integer events=0,periodic=0,checks=0,cycle_count=0,last_update=-1,event_cycle=-1,i,last_seq=0,last_uptime=0,delta_sum=0;
reg [511:0] last_event,last_periodic;
always @(negedge clk)begin
cycle_count=cycle_count+1;
if(rst_n&&dut.updated[0])last_update=cycle_count;
if(valid)begin
 $fdisplay(trace_fd,"%0128h",packet);
 checks=checks+1;
 if(packet[447:416]!==last_seq+1)$fatal(1,"packet sequence must advance for event and periodic");
 if(packet[415:384]<=last_uptime)$fatal(1,"duplicate or backwards snapshot timestamp");
 last_seq=packet[447:416];last_uptime=packet[415:384];
 if(packet[31:16]==0)begin
 events=events+1;last_event=packet;event_cycle=cycle_count;
 if(packet[255:128]!==0)$fatal(1,"event pretended to contain encoder window");
 end else if(packet[31:16]==50)begin
 periodic=periodic+1;last_periodic=packet;delta_sum=delta_sum+$signed(packet[255:224]);
 if(packet[415:384]%50)$fatal(1,"sonar event shifted encoder window boundary");
 end else $fatal(1,"unsupported window");
end
end
task automatic cycles(input integer n);repeat(n)@(negedge clk);endtask
task automatic byte_in(input integer ch,input [7:0] b);integer j;begin
@(negedge clk);rx[ch]=0;cycles(10);for(j=0;j<8;j=j+1)begin rx[ch]=b[j];cycles(10);end rx[ch]=1;cycles(10);end endtask
task automatic frame(input integer ch,input [15:0] d);reg [7:0]cs;begin cs=8'hff+d[15:8]+d[7:0];byte_in(ch,8'hff);byte_in(ch,d[15:8]);byte_in(ch,d[7:0]);byte_in(ch,cs);end endtask
task automatic step(input [1:0] ab);begin @(negedge clk);ea[0]=ab[1];eb[0]=ab[0];cycles(24);end endtask
initial begin
trace_fd=$fopen("reports/sonar-fast-events.hex","w");
cycles(5);rst_n=1;cycles(30);
fork frame(0,1000);frame(1,2000);frame(2,3000);join
cycles(110);
if(events!=1||last_event[471:464]!=7)$fatal(1,"simultaneous observations must coalesce and include all three channels");
if(event_cycle-last_update>101||event_cycle-last_update<0)$fatal(1,"accepted sonar event latency exceeded 1ms");
step(2'b10);step(2'b11);step(2'b01);step(2'b00);
cycles(700);frame(0,250);cycles(110);
if(last_event[127:112]!=250||last_event[383:352]!=4||last_event[79:64]>1)$fatal(1,"near event has wrong range/count/age");
cycles(10000);
if(periodic<2||delta_sum!=4||last_periodic[383:352]!=4)$fatal(1,"event shifted encoder delta accumulation");
// A far spike must not generate measurement refresh; its old measurement ages.
i=events;frame(0,4000);cycles(110);
if(events!=i)$fatal(1,"rejected far sample generated a fresh event");
if(dut.age[0]==0)$fatal(1,"rejected far sample renewed age");
frame(0,4020);cycles(110);if(events!=i+1||last_event[127:112]!=4000)$fatal(1,"confirmed clearance event missing");
// No UART frames for >1s: status loss must promptly publish fault.
i=events;cycles(100200);if(events<=i||last_event[471:464]!=0||last_event[463:456]!=7)$fatal(1,"timeout status transition not published");
// Age saturation is tested at the arithmetic boundary without 65s simulation.
@(negedge clk);dut.age[0]=16'hfffe;cycles(210);if(dut.age[0]!=16'hffff)$fatal(1,"age wrapped to fresh");
$fclose(trace_fd);
$display("PASS sonar v18 events: %0d coherent snapshots, <1ms coalescing, zero delta events, independent50ms windows, accepted-only age, expiry event, saturation",checks);$finish;
end
initial begin #10000000;$fatal(1,"timeout");end
endmodule
