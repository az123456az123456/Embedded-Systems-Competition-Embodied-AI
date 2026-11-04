`timescale 1ns/1ps
module tb_nav_packets;
    reg clk=0,rst_n=0,valid=0;
    reg [127:0] payload;
    reg [207:0] nav_payload;
    wire ready,dv;
    wire [7:0] data;
    motor_status_tx #(.NAV_ENABLE(1)) dut(.clk(clk),.rst_n(rst_n),.valid(valid),
        .payload(payload),.nav_payload(nav_payload),.ready(ready),.gmii_data(data),.gmii_valid(dv));
    always #4 clk=~clk;
    reg [7:0] bytes[0:143];
    integer n=0,gap=999,frame_count=0;
    reg previous=0;
    always @(negedge clk)begin
        if(dv)begin
            if(!previous)begin
                if(gap<12)$fatal(1,"interframe gap too short: %0d",gap);
                frame_count=frame_count+1;
            end
            bytes[n]=data;n=n+1;gap=0;
        end else gap=gap+1;
        previous=dv;
    end
    function [31:0] crc32;
        input [31:0] c;input [7:0] b;reg [31:0] x;integer k;
        begin x=c^b;for(k=0;k<8;k=k+1)x=x[0]?(x>>1)^32'hedb88320:x>>1;crc32=x;end
    endfunction
    function [15:0] crc16;
        input [15:0] c;input [7:0] b;reg [15:0] x;integer k;
        begin x=c^{b,8'b0};for(k=0;k<8;k=k+1)x=x[15]?(x<<1)^16'h1021:x<<1;crc16=x;end
    endfunction
    integer i,base,frame;
    reg [15:0] c16;
    reg [31:0] c32;
    initial begin
        payload={32'h41434b31,8'd1,8'd5,32'h12345678,16'hfed4,16'd300,8'd3,8'd12};
        nav_payload={32'h4e415631,8'd1,8'd3,16'd127,32'd42,32'd300000,32'd127,32'h12345678,16'd0};
        repeat(4)@(negedge clk);rst_n=1;
        @(negedge clk);valid=1;@(negedge clk);valid=0;
        wait(!ready);wait(ready);repeat(2)@(negedge clk);
        if(n!=144 || frame_count!=2)$fatal(1,"expected separate ACK and NAV frames");
        for(frame=0;frame<2;frame=frame+1)begin
            base=frame*72;
            for(i=0;i<7;i=i+1)if(bytes[base+i]!=8'h55)$fatal(1,"preamble mismatch");
            if(bytes[base+7]!=8'hd5 || bytes[base+20]!=8'h88 || bytes[base+21]!=(frame?8'hb7:8'hb6))$fatal(1,"header mismatch");
            c32=32'hffffffff;for(i=8;i<72;i=i+1)c32=crc32(c32,bytes[base+i]);
            if(c32!=32'hdebb20e3)$fatal(1,"FCS mismatch frame%0d",frame);
            c16=16'hffff;
            if(frame==0)begin
                for(i=0;i<16;i=i+1)begin
                    if(bytes[22+i]!=payload[127-i*8-:8])$fatal(1,"ACK data changed");
                    c16=crc16(c16,bytes[22+i]);
                end
                if({bytes[38],bytes[39]}!=c16)$fatal(1,"ACK crc mismatch");
                for(i=40;i<68;i=i+1)if(bytes[i]!=0)$fatal(1,"ACK padding must remain zero");
            end else begin
                for(i=0;i<26;i=i+1)begin
                    if(bytes[base+22+i]!=nav_payload[207-i*8-:8])$fatal(1,"NAV data mismatch");
                    c16=crc16(c16,bytes[base+22+i]);
                end
                if({bytes[base+48],bytes[base+49]}!=c16)$fatal(1,"NAV crc mismatch");
                for(i=50;i<68;i=i+1)if(bytes[base+i]!=0)$fatal(1,"NAV padding mismatch");
            end
        end
        $display("PASS separate ACK1/NAV1 frames, exact legacy ACK padding, both CRC16/FCS, big endian fields and IFG>=12");
        $finish;
    end
    initial begin #100000;$fatal(1,"NAV network timeout");end
endmodule
