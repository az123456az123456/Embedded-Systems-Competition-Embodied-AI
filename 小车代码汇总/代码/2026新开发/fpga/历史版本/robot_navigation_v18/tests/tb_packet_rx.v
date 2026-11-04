`timescale 1ns/1ps
module tb_packet_rx;
    reg clk=0,rst_n=0,valid=0,error=0;reg [7:0] data=0;
    wire command_valid;wire [127:0] command_data;wire [31:0] accepted,rejected;
    motor_packet_rx dut(.clk(clk),.rst_n(rst_n),.data(data),.valid(valid),.error(error),
        .command_valid(command_valid),.command_data(command_data),
        .accepted_frames(accepted),.rejected_frames(rejected));
    always #4 clk=~clk;
    reg [7:0] frame[0:63];reg [127:0] payload;
    reg [15:0] c16;reg [31:0] c32;integer i,k,commands=0;
    function [15:0] crc16;
        input [15:0] c;input [7:0] b;reg [15:0] x;integer j;
        begin x=c^{b,8'b0};for(j=0;j<8;j=j+1)x=x[15]?(x<<1)^16'h1021:x<<1;crc16=x;end
    endfunction
    function [31:0] crc32;
        input [31:0] c;input [7:0] b;reg [31:0] x;integer j;
        begin x=c^b;for(j=0;j<8;j=j+1)x=x[0]?(x>>1)^32'hedb88320:x>>1;crc32=x;end
    endfunction
    task prepare;
        begin
            for(i=0;i<64;i=i+1)frame[i]=0;
            frame[0]=2;frame[1]=8'hac;frame[2]=8'h13;frame[3]=8'h80;frame[4]=0;frame[5]=1;
            frame[6]=2;frame[7]=3;frame[8]=4;frame[9]=5;frame[10]=6;frame[11]=7;
            frame[12]=8'h88;frame[13]=8'hb6;
            payload={32'h4d4f5431,8'd1,8'd1,32'h12345678,16'd300,16'hfed4,8'd3,8'd12};
            c16=16'hffff;
            for(i=0;i<16;i=i+1)begin frame[14+i]=payload[127-i*8-:8];c16=crc16(c16,frame[14+i]);end
            frame[30]=c16[15:8];frame[31]=c16[7:0];
        end
    endtask
    task fcs;
        begin
            c32=32'hffffffff;
            for(i=0;i<60;i=i+1)c32=crc32(c32,frame[i]);
            c32=~c32;
            for(i=0;i<4;i=i+1)frame[60+i]=c32[i*8+:8];
        end
    endtask
    task send;
        input integer bytes;input integer inject_error;
        begin
            for(k=0;k<8;k=k+1)begin @(negedge clk);valid=1;data=k==7?8'hd5:8'h55;end
            for(k=0;k<bytes;k=k+1)begin @(negedge clk);data=frame[k];error=(inject_error && k==25);end
            @(negedge clk);valid=0;error=0;data=0;repeat(5)@(negedge clk);
        end
    endtask
    always @(posedge clk)if(command_valid)begin commands=commands+1;if(command_data!==payload)$fatal(1,"payload mismatch");end
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        prepare;fcs;send(64,0);if(commands!=1)$fatal(1,"valid frame rejected");
        prepare;fcs;frame[63]=frame[63]^1;send(64,0);
        prepare;frame[30]=frame[30]^1;fcs;send(64,0);
        prepare;frame[5]=2;fcs;send(64,0);
        prepare;frame[40]=1;fcs;send(64,0);
        prepare;fcs;send(64,1);
        prepare;fcs;send(62,0);
        if(commands!=1 || rejected!=6)$fatal(1,"corrupt frame accepted or counters wrong");
        prepare;fcs;send(64,0);
        if(commands!=2 || accepted!=2)$fatal(1,"receiver did not resynchronize");
        $display("PASS MOT1 FCS/CRC16/destination/padding/RX_ER/length validation and recovery");$finish;
    end
    initial begin #100000;$fatal(1,"timeout");end
endmodule
