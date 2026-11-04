`timescale 1ns/1ps
module tb_sensor_mmio;
    reg clk=0,reset_n=0,sv=0,iv=0,mv=0;
    always #10 clk=~clk;
    reg [511:0] sp=0,ip=0;
    reg [127:0] mp=0;
    wire [207:0] nav;wire trap;
    robot_nav_soc dut(.clk(clk),.reset_n(reset_n),.imu_valid(iv),.imu_packet(ip),
        .motor_valid(mv),.motor_packet(mp),.sensor_valid(sv),.sensor_packet(sp),.nav_status(nav),.cpu_trap(trap));
    // Exercise the actual SoC bus decoder while the separate firmware test
    // exercises the real CPU. These overrides never exist in synthesis.
    reg req=0;reg [31:0] addr=0,wd=0;reg [3:0] strobes=0;
    task transaction;input [31:0] a,w;input [3:0] ws;output [31:0] value;
        begin
            @(negedge clk);addr=a;wd=w;strobes=ws;req=1;
            @(negedge clk);if(!dut.mem_ready)$fatal(1,"MMIO handshake missing");value=dut.mem_rdata;req=0;
            @(negedge clk);
        end
    endtask
    integer k;reg [31:0] value;
    reg [511:0] first_sample,second_sample;
    initial begin
        force dut.mem_valid=req;force dut.mem_instr=1'b0;force dut.mem_addr=addr;
        force dut.mem_wstrb=strobes;force dut.mem_wdata=wd;
        for(k=0;k<16;k=k+1)begin first_sample[511-k*32 -: 32]=32'h12000000+k;second_sample[511-k*32 -: 32]=32'hab000000+k;end
        repeat(4)@(negedge clk);reset_n=1;
        @(negedge clk);sp=first_sample;ip={16{32'h12345678}};mp={4{32'h76543210}};sv=1;iv=1;mv=1;
        @(negedge clk);sv=0;iv=0;mv=0;
        transaction(32'h1000002c,1,1,value);
        @(negedge clk);sp=second_sample;sv=1;@(negedge clk);sv=0;
        for(k=0;k<16;k=k+1)begin
            transaction(32'h100000c0+k*4,0,0,value);
            if(value!==first_sample[511-k*32 -: 32])$fatal(1,"torn sensor snapshot word%d",k);
        end
        transaction(32'h100000c0,32'hffffffff,4'hf,value);
        transaction(32'h100000c0,0,0,value);
        if(value!==first_sample[511:480])$fatal(1,"sensor MMIO must be read-only");
        transaction(32'h10000040,0,0,value);if(value!==32'h12345678)$fatal(1,"old IMU MMIO changed");
        transaction(32'h10000080,0,0,value);if(value!==32'h76543210)$fatal(1,"old motor MMIO changed");
        transaction(32'h1000002c,1,1,value);
        for(k=0;k<16;k=k+1)begin
            transaction(32'h100000c0+k*4,0,0,value);
            if(value!==second_sample[511-k*32 -: 32])$fatal(1,"new sensor snapshot word%d",k);
        end
        $display("PASS actual SNS1 MMIO decoder C0..FC: all16 BE32 words, coherent explicit latch, read-only writes, unchanged IMU/motor snapshots");$finish;
    end
    initial begin #200000;$fatal(1,"MMIO timeout");end
endmodule
