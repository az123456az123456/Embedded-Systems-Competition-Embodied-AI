`timescale 1ns/1ps
// Auxiliary bare-metal processor. All inputs are already in clk50m domain.
// There is intentionally no connection from this CPU to motor command/PWM.
module robot_nav_soc #(
    parameter FIRMWARE_HEX="softcore/firmware/firmware.hex"
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
    output wire cpu_trap
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
        .PROGADDR_RESET(32'h00000000),.STACKADDR(32'h00010000)
    ) cpu (
        .clk(clk),.resetn(reset_n && !bus_fault),.trap(core_trap),
        .mem_valid(mem_valid),.mem_instr(mem_instr),.mem_ready(mem_ready),
        .mem_addr(mem_addr),.mem_wdata(mem_wdata),.mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),.pcpi_wr(1'b0),.pcpi_rd(32'b0),
        .pcpi_wait(1'b0),.pcpi_ready(1'b0),.irq(32'b0)
    );

    // Synchronous read and four byte write enables infer on-chip block RAM.
    // Startup is in this initialized RAM, so no separate Flash partition/XIP.
    reg [31:0] ram [0:16383];
    initial $readmemh(FIRMWARE_HEX,ram);
    reg [31:0] ram_q;
    reg ready;
    reg [31:0] io_q;
    reg transaction_ram;
    wire ram_select=(mem_addr[31:16]==16'h0000);
    wire io_select=(mem_addr[31:9]==23'h080000);
    wire request=mem_valid && !ready;
    assign mem_ready=ready;
    assign mem_rdata=transaction_ram ? ram_q : io_q;
    always @(posedge clk) begin
        if(request && ram_select) begin
            // GW5AT SP does not support read-before-write. Reads are omitted
            // during stores (PicoRV32 ignores store rdata), selecting normal
            // hold-on-write mode instead of the unsupported WRITE_MODE=2.
            if(mem_wstrb==0)ram_q<=ram[mem_addr[15:2]];
            if(mem_wstrb[0])ram[mem_addr[15:2]][7:0]<=mem_wdata[7:0];
            if(mem_wstrb[1])ram[mem_addr[15:2]][15:8]<=mem_wdata[15:8];
            if(mem_wstrb[2])ram[mem_addr[15:2]][23:16]<=mem_wdata[23:16];
            if(mem_wstrb[3])ram[mem_addr[15:2]][31:24]<=mem_wdata[31:24];
        end
    end
    reg [31:0] cycle_count,uptime_ms;
    reg [15:0] ms_div;
    reg [31:0] heartbeat,status,path_length,astar_cycles,expanded,observed_seq,fw_error;
    reg [511:0] latest_imu,snapshot_imu;
    reg [127:0] latest_motor,snapshot_motor;
    reg [511:0] latest_sensor,snapshot_sensor;
    reg [511:0] request_packet,reply_packet;
    reg request_pending,reply_pending;
    assign nav_request_ready=!request_pending;
    assign nav_reply_valid=reply_pending;
    assign nav_reply_packet=reply_packet;
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
            cycle_count<=0;uptime_ms<=0;ms_div<=0;
            heartbeat<=0;status<=0;path_length<=0;astar_cycles<=0;
            expanded<=0;observed_seq<=0;fw_error<=0;
            latest_imu<=0;snapshot_imu<=0;latest_motor<=0;snapshot_motor<=0;
            latest_sensor<=0;snapshot_sensor<=0;
            request_packet<=0;reply_packet<=0;request_pending<=0;reply_pending<=0;
        end else begin
            cycle_count<=cycle_count+1'b1;
            if(ms_div==49999)begin ms_div<=0;uptime_ms<=uptime_ms+1'b1;end
            else ms_div<=ms_div+1'b1;
            if(imu_valid)latest_imu<=imu_packet;
            if(motor_valid)latest_motor<=motor_packet;
            if(sensor_valid)latest_sensor<=sensor_packet;
            if(nav_request_valid && nav_request_ready) begin
                request_packet<=nav_request_packet;request_pending<=1;
            end
            if(nav_reply_valid && nav_reply_ready)reply_pending<=0;
            ready<=0;
            if(request)begin
                ready<=1;transaction_ram<=ram_select;io_q<=0;
                if(!ram_select && (!io_select || mem_instr))bus_fault<=1;
                if(io_select && !mem_instr)begin
                    if(mem_addr[8])begin
                        case(mem_addr[7:2])
                            6'h00:io_q<={31'b0,request_pending};
                            6'h01:if(mem_wstrb[0] && mem_wdata[0])request_pending<=0;
                            6'h02:io_q<={31'b0,!reply_pending};
                            6'h03:if(mem_wstrb[0] && mem_wdata[0] && !reply_pending)reply_pending<=1;
                            default:begin
                                if(mem_addr[7:6]==2'b01)io_q<=request_packet[511-(mem_addr[5:2]*32) -: 32];
                                if(mem_addr[7:6]==2'b10 && !reply_pending && |mem_wstrb)
                                    reply_packet[511-(mem_addr[5:2]*32) -:32]<=mem_wdata;
                            end
                        endcase
                    end else case(mem_addr[7:2])
                        6'h00:io_q<=32'h4e415631;
                        6'h01:io_q<=uptime_ms;
                        6'h02:io_q<=cycle_count;
                        6'h03:io_q<=32'h00000003;
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
