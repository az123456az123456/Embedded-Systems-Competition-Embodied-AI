`timescale 1ns/1ps
`include "softcore/firmware/firmware_symbols.vh"
module tb_nav_soc;
    reg clk=0,reset_n=0,imu_valid=0,motor_valid=0;
    reg [511:0] imu_packet=0;
    reg [127:0] motor_packet=0;
    wire [207:0] nav_status;
    wire trap;
    robot_nav_soc dut(.clk(clk),.reset_n(reset_n),
        .imu_valid(imu_valid),.imu_packet(imu_packet),
        .motor_valid(motor_valid),.motor_packet(motor_packet),
        .sensor_valid(1'b0),.sensor_packet(512'b0),
        .nav_request_valid(1'b0),.nav_request_packet(512'b0),.nav_request_ready(),
        .nav_reply_valid(),.nav_reply_packet(),.nav_reply_ready(1'b1),
        .nav_status(nav_status),.cpu_trap(trap));
    always #10 clk=~clk;
    integer clocks=0;
    integer word_index;
    reg [31:0] first_heartbeat;
    always @(posedge clk)if(reset_n)begin
        clocks<=clocks+1;
        if(trap)$fatal(1,"CPU trapped at cycle %0d addr=%08x",clocks,dut.mem_addr);
    end
    initial begin
        // Poison BSS independently of the zero-filled initialization image:
        // startup must clear it before incrementing selftest_passes.
        #1;
        for(word_index=(`NAV_FW_BSS_START>>2);word_index<(`NAV_FW_BSS_END>>2);word_index=word_index+1)
            dut.ram[word_index]=32'ha5a5a5a5;
        // The unused SRAM gap and lower stack guard must remain untouched.
        dut.ram[(`NAV_FW_STACK_BOTTOM>>2)-1]=32'hc001c0de;
        dut.ram[`NAV_FW_STACK_BOTTOM>>2]=32'h5a7ac001;
        repeat(5)@(negedge clk);reset_n=1;
        @(negedge clk);
        imu_packet={32'h494d5539,32'h0192000f,32'h12345678,416'b0};
        motor_packet={32'h41434b31,96'h0123456789abcdef01234567};
        imu_valid=1;motor_valid=1;
        @(negedge clk);imu_valid=0;motor_valid=0;
        wait(dut.status[1]);
        if(dut.status!==3 || dut.fw_error!==0 || dut.path_length!==127 || dut.expanded!==127)
            $fatal(1,"A* summary wrong status=%h error=%h path=%0d expanded=%0d",dut.status,dut.fw_error,dut.path_length,dut.expanded);
        if(dut.astar_cycles==0)$fatal(1,"A* cycle counter missing");
        wait(dut.observed_seq==32'h12345678);
        if(dut.ram[`NAV_FW_SELFTEST_PASSES>>2]!==1)
            $fatal(1,"BSS startup clear or selftest counter wrong");
        for(word_index=0;word_index<16;word_index=word_index+1)
            if(dut.ram[(`NAV_FW_OBSERVED_IMU>>2)+word_index]!==imu_packet[511-word_index*32 -:32])
                $fatal(1,"IMU snapshot word %0d mismatch",word_index);
        // observed_seq is written before the subsequent four motor reads.
        wait(dut.ram[(`NAV_FW_OBSERVED_MOTOR>>2)+3]==motor_packet[31:0]);
        for(word_index=0;word_index<4;word_index=word_index+1)
            if(dut.ram[(`NAV_FW_OBSERVED_MOTOR>>2)+word_index]!==motor_packet[127-word_index*32 -:32])
                $fatal(1,"Motor read-only snapshot word %0d mismatch",word_index);
        first_heartbeat=dut.heartbeat;
        wait(dut.heartbeat>first_heartbeat);
        // Change the input after the prior snapshot: software must explicitly
        // latch a new coherent sample rather than read an unheld source bus.
        @(negedge clk);imu_valid=1;
        imu_packet={32'h494d5539,32'h0192000f,32'h87654321,416'b0};
        @(negedge clk);imu_valid=0;
        wait(dut.observed_seq==32'h87654321);
        // A second complete pass verifies stable repeat execution and that the
        // planner can reuse its SRAM workspace after all four previous cases.
        if(dut.ram[`NAV_FW_SELFTEST_PASSES>>2]!=1)$fatal(1,"Unexpected periodic planner selftest");
        if(dut.ram[(`NAV_FW_STACK_BOTTOM>>2)-1]!==32'hc001c0de ||
           dut.ram[`NAV_FW_STACK_BOTTOM>>2]!==32'h5a7ac001)
            $fatal(1,"Firmware stack/workspace guard overwritten");
        if(nav_status[207:176]!==32'h4e415631 || nav_status[167:160]!==8'h03)
            $fatal(1,"NAV1 header/flags incorrect");
        $display("PASS actual RV32IM firmware boot, 64x64 A* 4 boot selftests, BSS clear/stack guards, byte/halfword RAM access, coherent IMU snapshot, motor read-only MMIO and advancing heartbeat; cycles=%0d Astar_cycles=%0d",clocks,dut.astar_cycles);
        $finish;
    end
    initial begin #2000000000;$fatal(1,"CPU firmware timeout");end
endmodule
