`timescale 1ns/1ps
module tb_rgmii_rx;
    GSR GSR(.GSRI(1'b1));
    reg clk=0,rst_n=0,ctl=0;reg [3:0] rxd=0;
    wire command_valid;wire [127:0] command_data;
    motor_rgmii_rx dut(.reset_n(rst_n),.rgmii_rx_clk(clk),.rgmii_rxd(rxd),
        .rgmii_rx_ctl(ctl),.command_valid(command_valid),.command_data(command_data),.receive_reset_n());
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
    task send_byte;input [7:0] value;
        begin
            @(negedge clk);#1;rxd=value[3:0];ctl=1;
            @(posedge clk);#1;rxd=value[7:4];ctl=1;
        end
    endtask
    task send;
        begin
            for(k=0;k<8;k=k+1)send_byte(k==7?8'hd5:8'h55);
            for(k=0;k<64;k=k+1)send_byte(frame[k]);
            @(negedge clk);#1;ctl=0;rxd=0;
            repeat(12)@(negedge clk);
        end
    endtask
    always @(posedge clk)if(command_valid)begin commands=commands+1;if(command_data!==payload)$fatal(1,"payload mismatch");end
    initial begin
        repeat(6)@(negedge clk);rst_n=1;repeat(6)@(negedge clk);
        prepare;fcs;send;if(commands!=1)$fatal(1,"IDDR RGMII nibble order rejected valid frame");
        prepare;fcs;frame[63]=frame[63]^1;send;
        if(commands!=1)$fatal(1,"IDDR RGMII corruption accepted");
        prepare;fcs;send;if(commands!=2)$fatal(1,"IDDR RGMII did not recover");
        $display("PASS real Gowin IDDR functional model, RGMII DDR nibble/control order and FCS rejection");$finish;
    end
    initial begin #100000;$fatal(1,"timeout");end
endmodule
