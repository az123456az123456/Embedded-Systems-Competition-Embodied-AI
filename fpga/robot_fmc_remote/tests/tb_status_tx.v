`timescale 1ns/1ps
module tb_status_tx;
    reg clk=0,rst_n=0,valid=0;reg [127:0] payload;
    wire ready,gmii_valid;wire [7:0] gmii_data;
    motor_status_tx dut(.clk(clk),.rst_n(rst_n),.valid(valid),.payload(payload),
        .ready(ready),.gmii_data(gmii_data),.gmii_valid(gmii_valid));
    always #4 clk=~clk;
    reg [7:0] bytes[0:100];integer n=0,i,j;
    reg [31:0] c32;reg [15:0] c16;
    function [31:0] crc32;
        input [31:0] c;input [7:0] b;reg [31:0] x;integer k;
        begin x=c^b;for(k=0;k<8;k=k+1)x=x[0]?(x>>1)^32'hedb88320:x>>1;crc32=x;end
    endfunction
    function [15:0] crc16;
        input [15:0] c;input [7:0] b;reg [15:0] x;integer k;
        begin x=c^{b,8'b0};for(k=0;k<8;k=k+1)x=x[15]?(x<<1)^16'h1021:x<<1;crc16=x;end
    endfunction
    always @(negedge clk)if(gmii_valid)begin bytes[n]=gmii_data;n=n+1;end
    initial begin
        payload={32'h41434b31,8'd1,8'd5,32'h12345678,16'hfed4,16'd300,8'd3,8'd12};
        repeat(3)@(negedge clk);rst_n=1;
        @(negedge clk);valid=1;@(negedge clk);valid=0;
        wait(!ready);wait(ready);repeat(3)@(negedge clk);
        if(n!=72)$fatal(1,"wrong ACK frame length");
        for(i=0;i<7;i=i+1)if(bytes[i]!=8'h55)$fatal(1,"bad preamble");
        if(bytes[7]!=8'hd5 || bytes[20]!=8'h88 || bytes[21]!=8'hb6)$fatal(1,"bad ACK header");
        c16=16'hffff;
        for(i=0;i<16;i=i+1)begin
            if(bytes[22+i]!=payload[127-i*8-:8])$fatal(1,"ACK payload byte mismatch");
            c16=crc16(c16,bytes[22+i]);
        end
        if({bytes[38],bytes[39]}!=c16)$fatal(1,"ACK CRC16 incorrect");
        for(i=40;i<68;i=i+1)if(bytes[i]!=0)$fatal(1,"ACK padding incorrect");
        c32=32'hffffffff;for(i=8;i<72;i=i+1)c32=crc32(c32,bytes[i]);
        if(c32!=32'hdebb20e3)$fatal(1,"ACK Ethernet FCS incorrect");
        $display("PASS ACK1 network byte order, padding, payload CRC16 and Ethernet FCS");$finish;
    end
    initial begin #10000;$fatal(1,"timeout");end
endmodule
