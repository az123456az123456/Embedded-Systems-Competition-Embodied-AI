`timescale 1ns/1ps
module tb_sensor_network;
    reg source_clk=0,tx_clk=0,rst_n=0,sample_valid=0;
    always #10 source_clk=~source_clk;
    always #4 tx_clk=~tx_clk;
    reg [511:0] sample;
    wire pending,busy,dv;wire [7:0] data;
    reg camera_busy=1,ip=0,ib=0,iv=0,mp=0,mb=0,mv=0;
    wire ca,ia,ma,sa,shared_valid;wire [7:0] shared_data;
    sensor_status_gmii dut(.source_clk(source_clk),.source_rst_n(rst_n),.sensor_valid(sample_valid),.sensor_data(sample),
        .tx_clk(tx_clk),.tx_rst_n(rst_n),.tx_allow(sa),.tx_pending(pending),.tx_busy(busy),.gmii_valid(dv),.gmii_data(data));
    gmii_four_source_share arbiter(.camera_busy(camera_busy),.camera_valid(1'b0),.camera_data(8'h11),
        .imu_pending(ip),.imu_busy(ib),.imu_valid(iv),.imu_data(8'h22),
        .motor_pending(mp),.motor_busy(mb),.motor_valid(mv),.motor_data(8'h33),
        .sensor_pending(pending),.sensor_busy(busy),.sensor_valid(dv),.sensor_data(data),
        .camera_allow(ca),.imu_allow(ia),.motor_allow(ma),.sensor_allow(sa),.gmii_valid(shared_valid),.gmii_data(shared_data));
    reg [7:0] bytes[0:127];integer n=0,gap=0,fd,i;
    always @(negedge tx_clk)begin
        if(dv)begin bytes[n]=data;n=n+1;gap=0;if(!shared_valid||shared_data!==data)$fatal(1,"shared stream not source");end
        else if(n>0)gap=gap+1;
        if(busy&&(ca||ia||ma))$fatal(1,"SNS1 ownership or IFG preempted");
    end
    function [31:0] crc32;
        input [31:0] c;input [7:0] b;reg [31:0] x;integer k;
        begin x=c^b;for(k=0;k<8;k=k+1)x=x[0]?(x>>1)^32'hedb88320:x>>1;crc32=x;end
    endfunction
    function [15:0] crc16;
        input [15:0] c;input [7:0] b;reg [15:0] x;integer k;
        begin x=c^{b,8'b0};for(k=0;k<8;k=k+1)x=x[15]?(x<<1)^16'h1021:x<<1;crc16=x;end
    endfunction
    reg [511:0] expected;
    reg [15:0] c16;reg [31:0] c32;
    initial begin
        expected={32'h534e5331,8'd1,8'd7,8'd0,8'd0,32'd42,32'd123456,
            32'd4,-32'd8,32'd0,32'h7fffffff,32'd1,-32'd2,32'd0,32'd3,
            16'd1234,16'd2000,16'd4500,16'd4,16'd5,16'd6,16'd50,8'd0,8'd0};
        sample=expected;
        repeat(5)@(negedge source_clk);rst_n=1;
        @(negedge source_clk);sample_valid=1;@(negedge source_clk);sample_valid=0;sample=~expected;
        wait(pending);repeat(20)@(negedge tx_clk);
        if(n||busy||sa||ma||ia)$fatal(1,"pending SNS1 preempted camera");
        camera_busy=0;mp=1;#1;
        if(sa||ia||ca||!ma)$fatal(1,"ACK/NAV must win over sensor");
        mp=0;ip=1;#1;if(!sa||ia||ca)$fatal(1,"SNS1 must precede pending IMU");
        wait(busy);wait(!busy);@(negedge tx_clk);#1;
        if(n!=92||gap<12)$fatal(1,"wire length/IFG %d/%d",n,gap);
        for(i=0;i<7;i=i+1)if(bytes[i]!==8'h55)$fatal(1,"preamble");
        if(bytes[7]!==8'hd5||bytes[20]!==8'h88||bytes[21]!==8'hb8)$fatal(1,"header");
        c16=16'hffff;
        for(i=0;i<64;i=i+1)begin
            if(bytes[i+22]!==expected[511-i*8 -: 8])$fatal(1,"CDC snapshot torn at byte%d",i);
            c16=crc16(c16,bytes[i+22]);
        end
        if({bytes[86],bytes[87]}!==c16)$fatal(1,"CCITT-FALSE mismatch");
        c32=32'hffffffff;for(i=8;i<92;i=i+1)c32=crc32(c32,bytes[i]);
        if(c32!==32'hdebb20e3)$fatal(1,"Ethernet FCS mismatch");
        fd=$fopen("reports/sensor_frame_preamble.bin","wb");
        for(i=0;i<92;i=i+1)$fwrite(fd,"%c",bytes[i]);$fclose(fd);
        fd=$fopen("reports/sensor_frame_ethernet.bin","wb");
        for(i=8;i<92;i=i+1)$fwrite(fd,"%c",bytes[i]);$fclose(fd);
        ip=0;#1;if(!ca)$fatal(1,"camera starvation");
        $display("PASS SNS1 real asynchronous CDC, immutable payload, priority, protected IFG, all64 bytes/CRC16/FCS; exported reports/sensor_frame_ethernet.bin");
        $finish;
    end
    initial begin #100000;$fatal(1,"network timeout");end
endmodule
