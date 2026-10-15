`timescale 1ns/1ps
module tb_ultrasonic;
 reg clk=0; always #5 clk=~clk;
 reg rst_n=0, rx=1;
 wire [7:0] byte_data;
 wire byte_valid;
 a02_uart_rx #(.CLK_FREQ(1000000),.BAUD(10000)) receiver(clk,rst_n,rx,byte_data,byte_valid);
 wire [15:0] distance;
 wire valid, online, fault, obstacle, slow, stop_req;
 ultrasonic_sensor #(.CLK_FREQ(1000000),.FRAME_TIMEOUT_MS(2),.DATA_TIMEOUT_MS(10)) dut
 (clk,rst_n,byte_data,byte_valid,distance,valid,online,fault,obstacle,slow,stop_req);
 integer count=0;
 always @(posedge clk) if(valid) count=count+1;
 task check; input condition; input [511:0] message; begin
 if(condition !== 1'b1) begin $display("FAIL: %0s",message); $fatal(1); end
 end endtask
 task send_byte; input [7:0] value; input good_stop; integer k; begin
 @(negedge clk); rx=0; repeat(100) @(negedge clk);
 for(k=0;k<8;k=k+1) begin rx=value[k]; repeat(100) @(negedge clk); end
 rx=good_stop; repeat(100) @(negedge clk); rx=1; repeat(10) @(negedge clk);
 end endtask
 task frame; input [15:0] mm; input corrupt; begin
 send_byte(8'hff,1); send_byte(mm[15:8],1); send_byte(mm[7:0],1);
 send_byte((8'hff+mm[15:8]+mm[7:0]) ^ corrupt,1); repeat(10) @(negedge clk);
 end endtask
 reg send=0; reg [7:0] tx_data=0; wire tx,busy;
 a02_uart_tx #(.CLK_FREQ(1000000),.BAUD(10000)) transmitter(clk,rst_n,send,tx_data,tx,busy);
 wire [7:0] loop_data; wire loop_valid;
 a02_uart_rx #(.CLK_FREQ(1000000),.BAUD(10000)) loopback(clk,rst_n,tx,loop_data,loop_valid);
 integer loops=0;
 always @(posedge clk) if(loop_valid) begin
 if(loop_data !== 8'ha5) $fatal(1,"UART TX loopback mismatch"); loops=loops+1;
 end
 initial begin
 repeat(5) @(negedge clk); rst_n=1; repeat(5) @(negedge clk);
 check(fault && stop_req,"startup fail safe");
 tx_data=8'ha5; send=1; @(negedge clk); send=0;
 frame(1953,0); check(distance==1953 && online && !stop_req && !slow && !obstacle,"manual example 1953mm");
 check(loops==1,"UART TX frame");
 frame(250,0); check(stop_req && slow && obstacle,"near obstacle stop");
 frame(320,0); check(stop_req,"stop hysteresis hold");
 frame(350,0); check(!stop_req && slow,"stop hysteresis release");
 frame(900,0); check(!slow && obstacle,"slow release");
 frame(1300,0); check(!obstacle,"obstacle release");
 frame(100,1); check(distance==1300 && count==6,"bad checksum ignored");
 send_byte(8'hff,0); send_byte(0,1); send_byte(100,1); send_byte(99,1);
 check(count==6,"bad stop ignored");
 send_byte(8'hff,1); repeat(2200) @(negedge clk);
 frame(1000,0); check(distance==1000 && count==7,"partial frame timeout recovery");
 frame(0,0); check(fault && stop_req && distance==1000,"out of range fail safe");
 frame(4500,0); check(online && !stop_req,"maximum range");
 frame(30,0); check(online && stop_req,"minimum range");
 repeat(11000) @(negedge clk); check(fault && stop_req && slow,"disconnect timeout");
 frame(1953,0); check(online && !stop_req,"reconnect recovery");
 $display("PASS: UART RX/TX, valid frames, checksum, stop bit, timeout, range, hysteresis, reconnect");
 $finish;
 end
 initial begin #10000000; $fatal(1,"test watchdog"); end
endmodule
