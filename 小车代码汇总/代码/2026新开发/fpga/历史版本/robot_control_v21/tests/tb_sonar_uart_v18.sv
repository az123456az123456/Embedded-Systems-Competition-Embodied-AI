`timescale 1ns/1ps
module tb_sonar_uart_v18;
reg clk=0;always #5 clk=~clk;reg rst_n=0,rx=1;
wire [7:0] data;wire valid;
a02_uart_rx #(.CLK_FREQ(1000000),.BAUD(10000)) dut(.clk(clk),.rst_n(rst_n),.rx(rx),.data(data),.valid(valid));
integer got=0,phase,b,n,checks=0;reg [7:0] last;
always @(negedge clk)if(valid)begin got=got+1;last=data;end
task automatic cycles(input integer n);repeat(n)@(negedge clk);endtask
task automatic value_bit(input bit val,input integer clocks,input integer glitch);
integer k;begin for(k=0;k<clocks;k=k+1)begin rx=(k==glitch)?!val:val;cycles(1);end end endtask
task automatic send_byte(input [7:0] val,input integer clocks,input integer glitch,input bit badstop);
integer j;begin @(negedge clk);value_bit(0,clocks,glitch);for(j=0;j<8;j=j+1)value_bit(val[j],clocks,glitch);
value_bit(!badstop,clocks,badstop?-1:glitch);rx=1;cycles(200);end endtask
initial begin
cycles(5);rst_n=1;cycles(10);
// Sweep physical input glitches across every possible center sample alignment.
for(phase=44;phase<=58;phase=phase+1)begin
n=got;send_byte(8'ha5,100,phase,0);checks=checks+1;
if(got!=n+1||last!=8'ha5)$fatal(1,"center single-clock glitch at phase%0d escaped vote",phase);
end
for(b=0;b<256;b=b+1)begin n=got;send_byte(b,100,-1,0);checks=checks+1;if(got!=n+1||last!=b)$fatal(1,"byte%0d",b);end
n=got;send_byte(8'hff,100,-1,1);checks=checks+1;if(got!=n)$fatal(1,"invalid stop accepted");
n=got;rx=0;cycles(5);rx=1;cycles(1100);checks=checks+1;if(got!=n)$fatal(1,"false start glitch accepted");
n=got;send_byte(8'h3c,98,-1,0);checks=checks+1;if(got!=n+1||last!=8'h3c)$fatal(1,"+2percent baud error");
n=got;send_byte(8'hc3,102,-1,0);checks=checks+1;if(got!=n+1||last!=8'hc3)$fatal(1,"-2percent baud error");
$display("PASS sonar UART v18: %0d checks, all256 bytes, glitch phase sweep, invalidstop/start, +/-2percent baud",checks);$finish;
end
initial begin #10000000;$fatal(1,"timeout");end
endmodule
