`timescale 1ns/1ps
// SNS1 snapshot in clk50m domain. All sensor acquisition is read-only and does
// not inhibit/enable the motor drive. Per-wheel deltas cover exact 50ms windows.
// Accepted sonar observations / validity changes also send on the next 1ms
// tick: window_ms=0, all deltas=0. Counts and uptime are from that same tick.
// Event coalescing captures all three channels without back-to-back CDC drops;
// it never moves the encoder baseline or restarts its 50ms reporting clock.
module robot_sensor_capture #(
    parameter integer CLK_FREQ=50_000_000,
    parameter integer BAUD=9600,
    parameter integer REPORT_MS=50,
    parameter integer ENCODER_FILTER_CYCLES=16,
    parameter [3:0] ENCODER_INVERT_MASK=4'b0000
)(
    input wire clk,rst_n,
    input wire [3:0] encoder_a,encoder_b,
    input wire [2:0] sonar_rx,
    output reg sensor_valid,
    output reg [511:0] sensor_packet
);
    localparam integer MS_CYCLES=CLK_FREQ/1000;
    localparam [15:0] WINDOW_MS=REPORT_MS;
    reg [31:0] ms_div,uptime_ms,packet_seq;
    reg [15:0] report_ms;
    reg sonar_event_pending;
    reg [2:0] previous_online;
    wire tick_ms=ms_div==MS_CYCLES-1;
    wire [127:0] positions;
    wire [3:0] illegal;
    reg [127:0] baseline;
    wire [47:0] distances;
    wire [2:0] updated,online,fault;
    reg [15:0] age [0:2];
    integer k;
    encoder_4ch #(.FILTER_CYCLES(ENCODER_FILTER_CYCLES),.INVERT_MASK(ENCODER_INVERT_MASK)) encoders(
        .clk(clk),.rst_n(rst_n),.encoder_a(encoder_a),.encoder_b(encoder_b),
        .positions(positions),.illegal_seen(illegal));
    ultrasonic_3ch #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD)) sonar(
        .clk(clk),.rst_n(rst_n),.rx(sonar_rx),.distance_mm(distances),
        .distance_valid(updated),.online(online),.sensor_fault(fault),
        .obstacle(),.slow_down(),.stop_req(),.any_obstacle(),.any_slow_down(),.any_stop_req(),
        .front_distance_mm(),.left_distance_mm(),.right_distance_mm(),
        .forward_slow_req(),.forward_stop_req(),.left_blocked(),.right_blocked());
    wire [31:0] d0=positions[31:0]-baseline[31:0];
    wire [31:0] d1=positions[63:32]-baseline[63:32];
    wire [31:0] d2=positions[95:64]-baseline[95:64];
    wire [31:0] d3=positions[127:96]-baseline[127:96];
    wire [31:0] next_packet_seq=packet_seq+32'd1;
    wire [31:0] next_uptime=uptime_ms+32'd1;
    wire periodic = report_ms == REPORT_MS-1;
    wire sonar_event = sonar_event_pending || (|updated) || (online != previous_online);
    // If a fresh sample arrives on the reporting tick, UART's registered online
    // and distance have already updated together; age becomes zero here too.
    wire [15:0] a0=updated[0]?16'd0:(age[0]==16'hffff ? 16'hffff : age[0]+16'd1);
    wire [15:0] a1=updated[1]?16'd0:(age[1]==16'hffff ? 16'hffff : age[1]+16'd1);
    wire [15:0] a2=updated[2]?16'd0:(age[2]==16'hffff ? 16'hffff : age[2]+16'd1);
    always @(posedge clk)begin
        if(!rst_n)begin
            ms_div<=0;uptime_ms<=0;packet_seq<=0;report_ms<=0;baseline<=0;
            sonar_event_pending<=0;previous_online<=0;
            sensor_valid<=0;sensor_packet<=0;
            for(k=0;k<3;k=k+1)age[k]<=16'hffff;
        end else begin
            sensor_valid<=0;previous_online<=online;
            if((|updated) || online!=previous_online)sonar_event_pending<=1;
            if(tick_ms)begin
                ms_div<=0;uptime_ms<=next_uptime;
                sonar_event_pending<=0;
                if(periodic)begin report_ms<=0;baseline<=positions;end
                else report_ms<=report_ms+1'b1;
                if(periodic || sonar_event)begin
                    packet_seq<=next_packet_seq;sensor_valid<=1;
                    sensor_packet<={32'h534e5331,8'd1,{5'b0,online},{5'b0,fault},{4'b0,ENCODER_INVERT_MASK},
                        next_packet_seq,next_uptime,
                        positions[31:0],positions[63:32],positions[95:64],positions[127:96],
                        periodic?d0:32'd0,periodic?d1:32'd0,periodic?d2:32'd0,periodic?d3:32'd0,
                        distances[15:0],distances[31:16],distances[47:32],a0,a1,a2,
                        periodic?WINDOW_MS:16'd0,{4'b0,illegal},8'd0};
                end
            end else ms_div<=ms_div+1'b1;
            for(k=0;k<3;k=k+1)begin
                if(updated[k])age[k]<=0;
                else if(tick_ms && age[k]!=16'hffff)age[k]<=age[k]+1'b1;
            end
        end
    end
endmodule
