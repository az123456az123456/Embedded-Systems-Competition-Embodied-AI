`timescale 1ns/1ps
// Auxiliary bare-metal processor. All inputs are already in clk50m domain.
// v28 adds a bounded atomic motor-output mailbox; external authorization and
// an independent60ms CPU commit watchdog remain outside firmware control.
module robot_nav_soc #(
    parameter integer CLK_HZ=50000000,
    // FIRMWARE_HEX names only the 16384-word LOW bank in this RAM80 ABI.
    parameter FIRMWARE_HEX="softcore/firmware/firmware_low.hex",
    parameter TAIL_PREFIX="softcore/firmware/firmware_tail"
)(
    input wire clk, reset_n,
    input wire imu_valid,
    input wire [511:0] imu_packet,
    input wire motor_valid,
    input wire [127:0] motor_packet,
    input wire sensor_valid,
    input wire [511:0] sensor_packet,
    input wire nav_request_valid,
    input wire [511:0] nav_request_packet,
    output wire nav_request_ready,
    output wire nav_reply_valid,
    output wire [511:0] nav_reply_packet,
    input wire nav_reply_ready,
    output wire [207:0] nav_status,
    output wire cpu_trap,
    input wire [4:0] control_flags,
    input wire [31:0] control_epoch,control_sequence,control_motion,
    input wire [7:0] control_masks,
    input wire [127:0] encoder_positions,
    input wire [3:0] encoder_illegal,
    input wire [47:0] sonar_mm,
    input wire [95:0] sonar_seq,sonar_stamp,
    input wire [2:0] sonar_online,
    output reg [63:0] control_pwm,
    output wire control_ready,
    output reg [47:0] filter_mm,
    output reg [95:0] filter_seq,
    output wire [2:0] filter_valid,
    output reg [127:0] control_diagnostics,
    // One outstanding extension transaction. req is a pulse, never repeated
    // while the CPU waits for a synchronous packet-RAM/authority response.
    output reg exec_bus_req,
    output reg [11:0] exec_bus_offset,
    output reg [31:0] exec_bus_wdata,
    output reg [3:0] exec_bus_wstrb,
    input wire exec_bus_rsp_valid,
    input wire [31:0] exec_bus_rsp_data,
    output wire [31:0] local_sample_id,local_sample_ms,local_imu_ms,
    output reg [31:0] local_sample_deadline,local_imu_deadline,
    output wire local_sample_valid,
    output wire [31:0] device_ms
);
    wire mem_valid, mem_instr, mem_ready;
    wire [31:0] mem_addr, mem_wdata, mem_rdata;
    wire [3:0] mem_wstrb;
    wire core_trap;
    reg bus_fault;
    picorv32 #(
        .ENABLE_COUNTERS(1),.ENABLE_COUNTERS64(0),
        .ENABLE_REGS_DUALPORT(1),.BARREL_SHIFTER(1),
        .TWO_CYCLE_ALU(1),.TWO_CYCLE_COMPARE(1),
        .ENABLE_MUL(1),.ENABLE_FAST_MUL(0),.ENABLE_DIV(1),
        .ENABLE_IRQ(0),.ENABLE_PCPI(0),.COMPRESSED_ISA(0),
        .CATCH_MISALIGN(1),.CATCH_ILLINSN(1),
        .PROGADDR_RESET(32'h00000000),.STACKADDR(32'h00014000)
    ) cpu (
        .clk(clk),.resetn(reset_n && !bus_fault),.trap(core_trap),
        .mem_valid(mem_valid),.mem_instr(mem_instr),.mem_ready(mem_ready),
        .mem_addr(mem_addr),.mem_wdata(mem_wdata),.mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),.pcpi_wr(1'b0),.pcpi_rd(32'b0),
        .pcpi_wait(1'b0),.pcpi_ready(1'b0),.irq(32'b0)
    );

    // Synchronous read and four byte write enables infer on-chip block RAM.
    // Startup is in this initialized RAM, so no separate Flash partition/XIP.
    wire [31:0] ram_q;
    reg ready;
    reg [31:0] io_q;
    reg transaction_ram;
    wire ram_select;
    wire io_select=(mem_addr[31:12]==20'h10000);
    reg exec_waiting;
    wire request=mem_valid && !ready && !exec_waiting;
    assign mem_ready=ready;
    assign mem_rdata=transaction_ram ? ram_q : io_q;
    nav_ram80 #(.LOW_HEX(FIRMWARE_HEX),.TAIL_PREFIX(TAIL_PREFIX)) memory(
        .clk(clk),.request(request),.address(mem_addr),.wdata(mem_wdata),
        .wstrb(mem_wstrb),.rdata(ram_q),.address_valid(ram_select));
    reg [31:0] cycle_count,uptime_ms;
    assign device_ms=uptime_ms;
    reg [31:0] ctrl_snapshot[0:43];
    reg [511:0] latest_imu,snapshot_imu;
    reg [31:0] imu_new_deadline;
    reg [31:0] imu_arrival_ms,imu_new_sequence,imu_new_ms,sample_id;
    wire [15:0] imu_source_age=latest_imu[63:48];
    assign local_sample_id=ctrl_snapshot[7];
    assign local_sample_ms=ctrl_snapshot[1];
    assign local_imu_ms=ctrl_snapshot[14];
    assign local_sample_valid=ctrl_snapshot[15][0];
    reg [31:0] ctrl_shadow[0:15];
    reg [31:0] commit_value,commit_ms,committed_flags;
    reg have_commit;
    integer ci;
    assign control_ready=have_commit && !cpu_trap && committed_flags[1:0]==2'b11 &&
        (uptime_ms-commit_ms)<32'd60;
    assign filter_valid=committed_flags[10:8];
    reg [15:0] ms_div;
    reg [31:0] heartbeat,status,path_length,astar_cycles,expanded,observed_seq,fw_error;
    reg [127:0] latest_motor,snapshot_motor;
    reg [511:0] latest_sensor,snapshot_sensor;
    assign nav_request_ready=1'b1;
    assign nav_reply_valid=1'b0;
    assign nav_reply_packet=512'b0;
    function [31:0] merge_bytes;
        input [31:0] old_value,new_value;
        input [3:0] strobes;
        integer b;
        begin
            merge_bytes=old_value;
            for(b=0;b<4;b=b+1)
                if(strobes[b])merge_bytes[b*8 +: 8]=new_value[b*8 +: 8];
        end
    endfunction
    always @(posedge clk) begin
        if(!reset_n)begin
            ready<=0;transaction_ram<=0;io_q<=0;bus_fault<=0;
            exec_waiting<=0;exec_bus_req<=0;exec_bus_offset<=0;
            exec_bus_wdata<=0;exec_bus_wstrb<=0;
            cycle_count<=0;uptime_ms<=0;local_sample_deadline<=0;local_imu_deadline<=0;imu_new_deadline<=0;ms_div<=0;imu_arrival_ms<=0;imu_new_sequence<=0;imu_new_ms<=0;sample_id<=0;
            commit_value<=0;commit_ms<=0;have_commit<=0;committed_flags<=0;
            control_pwm<=0;filter_mm<=0;filter_seq<=0;control_diagnostics<=0;
            for(ci=0;ci<44;ci=ci+1)ctrl_snapshot[ci]<=0;
            for(ci=0;ci<16;ci=ci+1)ctrl_shadow[ci]<=0;
            heartbeat<=0;status<=0;path_length<=0;astar_cycles<=0;
            expanded<=0;observed_seq<=0;fw_error<=0;
            latest_imu<=0;snapshot_imu<=0;latest_motor<=0;snapshot_motor<=0;
            latest_sensor<=0;snapshot_sensor<=0;
            
        end else begin
            cycle_count<=cycle_count+1'b1;
            if(ms_div==CLK_HZ/1000-1)begin ms_div<=0;uptime_ms<=uptime_ms+1'b1;end
            else ms_div<=ms_div+1'b1;
            if(imu_valid)begin latest_imu<=imu_packet;imu_arrival_ms<=uptime_ms;
                if(imu_packet[447:416]!=imu_new_sequence)begin imu_new_sequence<=imu_packet[447:416];imu_new_ms<=uptime_ms;imu_new_deadline<=uptime_ms+250;end
            end
            if(motor_valid)latest_motor<=motor_packet;
            if(sensor_valid)latest_sensor<=sensor_packet;
            ready<=0;
            exec_bus_req<=0;
            if(exec_waiting && exec_bus_rsp_valid)begin
                ready<=1;transaction_ram<=0;io_q<=exec_bus_rsp_data;
                exec_waiting<=0;
            end
            if(request)begin
                ready<=1;transaction_ram<=ram_select;io_q<=0;
                if(!ram_select && (!io_select || mem_instr))bus_fault<=1;
                if(io_select && !mem_instr)begin
                    if(mem_addr[11:10]!=0)begin
                        ready<=0;exec_waiting<=1;exec_bus_req<=1;
                        exec_bus_offset<=mem_addr[11:0];
                        exec_bus_wdata<=mem_wdata;exec_bus_wstrb<=mem_wstrb;
                    end else if(mem_addr[9])begin
                        if(mem_addr[8])begin
                            io_q<=ctrl_shadow[mem_addr[5:2]];
                            if(|mem_wstrb)begin
                                ctrl_shadow[mem_addr[5:2]]<=merge_bytes(ctrl_shadow[mem_addr[5:2]],mem_wdata,mem_wstrb);
                                if(mem_addr[7:2]==6'h0b && mem_wstrb==4'hf && (!have_commit || mem_wdata!=commit_value))begin
                                    commit_value<=mem_wdata;commit_ms<=uptime_ms;have_commit<=1;
                                    control_pwm<={ctrl_shadow[3][15:0],ctrl_shadow[2][15:0],ctrl_shadow[1][15:0],ctrl_shadow[0][15:0]};
                                    filter_mm<={ctrl_shadow[6][15:0],ctrl_shadow[5][15:0],ctrl_shadow[4][15:0]};
                                    committed_flags<=ctrl_shadow[7];
                                    filter_seq<={ctrl_shadow[10],ctrl_shadow[9],ctrl_shadow[8]};
                                    control_diagnostics<={ctrl_shadow[15],ctrl_shadow[14],ctrl_shadow[13],ctrl_shadow[12]};
                                end
                            end
                        end else begin
                            if(mem_addr[7:2]<44)io_q<=ctrl_snapshot[mem_addr[7:2]];
                            if(mem_addr[7:2]==0 && mem_wstrb[0] && mem_wdata[0])begin
                                sample_id<=sample_id+1;ctrl_snapshot[7]<=sample_id+1;local_sample_deadline<=uptime_ms+250;local_imu_deadline<=imu_new_deadline-{16'b0,imu_source_age};
                                ctrl_snapshot[14]<=imu_new_ms-{16'b0,imu_source_age};
                                ctrl_snapshot[15]<={31'b0,(imu_new_sequence!=0&&latest_imu[448]&&latest_imu[450]&&imu_source_age<=100&&encoder_illegal==0)};
                                ctrl_snapshot[1]<=uptime_ms;
                                ctrl_snapshot[2]<=control_epoch;
                                ctrl_snapshot[3]<={27'b0,control_flags};
                                ctrl_snapshot[4]<=control_sequence;
                                ctrl_snapshot[5]<=control_motion;
                                ctrl_snapshot[6]<={24'b0,control_masks};
                                for(ci=0;ci<4;ci=ci+1)ctrl_snapshot[8+ci]<=encoder_positions[ci*32 +:32];
                                ctrl_snapshot[12]<={28'b0,encoder_illegal};
                                ctrl_snapshot[13]<=imu_arrival_ms;
                                for(ci=0;ci<16;ci=ci+1)ctrl_snapshot[16+ci]<=latest_imu[511-ci*32 -:32];
                                for(ci=0;ci<3;ci=ci+1)begin
                                    ctrl_snapshot[32+ci]<={sonar_seq[ci*32 +:16],sonar_mm[ci*16 +:16]};
                                    ctrl_snapshot[36+ci]<=sonar_stamp[ci*32 +:32];
                                    ctrl_snapshot[41+ci]<=sonar_seq[ci*32 +:32];
                                end
                                ctrl_snapshot[40]<={21'b0,~sonar_online,5'b0,sonar_online};
                            end
                        end
                    end else if(mem_addr[8])begin
                        // Deprecated NAV2 window: acknowledge access, return
                        // empty request / permanently ready reply; no storage.
                        if(mem_addr[7:2]==6'h02)io_q<=1;
                    end else case(mem_addr[7:2])
                        6'h00:io_q<=32'h4e415631;
                        6'h01:io_q<=uptime_ms;
                        6'h02:io_q<=cycle_count;
                        6'h03:io_q<=32'h00000007;
                        6'h04:begin io_q<=heartbeat;if(|mem_wstrb)heartbeat<=merge_bytes(heartbeat,mem_wdata,mem_wstrb);end
                        6'h05:begin io_q<=status;if(|mem_wstrb)status<=merge_bytes(status,mem_wdata,mem_wstrb);end
                        6'h06:begin io_q<=path_length;if(|mem_wstrb)path_length<=merge_bytes(path_length,mem_wdata,mem_wstrb);end
                        6'h07:begin io_q<=astar_cycles;if(|mem_wstrb)astar_cycles<=merge_bytes(astar_cycles,mem_wdata,mem_wstrb);end
                        6'h08:begin io_q<=expanded;if(|mem_wstrb)expanded<=merge_bytes(expanded,mem_wdata,mem_wstrb);end
                        6'h09:begin io_q<=observed_seq;if(|mem_wstrb)observed_seq<=merge_bytes(observed_seq,mem_wdata,mem_wstrb);end
                        6'h0a:begin io_q<=fw_error;if(|mem_wstrb)fw_error<=merge_bytes(fw_error,mem_wdata,mem_wstrb);end
                        6'h0b:if(mem_wstrb[0] && mem_wdata[0])begin
                            snapshot_imu<=latest_imu;snapshot_motor<=latest_motor;snapshot_sensor<=latest_sensor;
                        end
                        default:begin
                            if(mem_addr[7:6]==2'b01)
                                io_q<=snapshot_imu[511-(mem_addr[5:2]*32) -: 32];
                            if(mem_addr[7:6]==2'b11)
                                io_q<=snapshot_sensor[511-(mem_addr[5:2]*32) -: 32];
                            if(mem_addr[7:4]==4'h8)
                                io_q<=snapshot_motor[127-(mem_addr[3:2]*32) -: 32];
                        end
                    endcase
                end
            end
        end
    end
    assign cpu_trap=core_trap || bus_fault;
    wire [7:0] flags={4'b0,status[2],cpu_trap,status[1:0]};
    assign nav_status={32'h4e415631,8'd1,flags,path_length[15:0],
                       heartbeat,astar_cycles,expanded,observed_seq,fw_error[15:0]};
endmodule
