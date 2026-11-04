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
    parameter integer CPU_FILTER_ENABLE=0,
    parameter [3:0] ENCODER_INVERT_MASK=4'b0000
)(
    input wire clk,rst_n,
    input wire [3:0] encoder_a,encoder_b,
    input wire [2:0] sonar_rx,
    output reg sensor_valid,
    output reg [511:0] sensor_packet,
    input wire [47:0] filter_mm,
    input wire [95:0] filter_seq,
    input wire [2:0] filter_valid,
    input wire filter_ready,
    output wire [127:0] encoder_positions,
    output wire [3:0] encoder_illegal,
    output wire [47:0] raw_sonar_mm,
    output reg [95:0] raw_sonar_seq,raw_sonar_stamp,
    output wire [2:0] raw_sonar_online
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
    wire [47:0] distances, source_ages;
    wire [2:0] updated,online,fault;
    wire [47:0] published_distances;
    assign encoder_positions=positions;
    assign encoder_illegal=illegal;
    assign raw_sonar_mm=distances;
    assign raw_sonar_online=online;
    genvar ch;
    generate for(ch=0;ch<3;ch=ch+1)begin:filter_publish
        // A new raw sample can precede the next CPU commit by20ms. Publish
        // that actual sample until its estimate exists; never invent an
        // invalid gap or refresh timestamps from a CPU heartbeat.
        wire matched=CPU_FILTER_ENABLE && filter_ready && filter_valid[ch] &&
            filter_seq[ch*32 +:32]==raw_sonar_seq[ch*32 +:32];
        wire near=distances[ch*16 +:16]<=16'd350;
        // A checksum-valid zero is mapped by the decoder to4500mm. Do not
        // let a first far Kalman estimate replace that fresh marker with the
        // prior near scene; near raw observations retain their own bypass.
        wire no_echo=distances[ch*16 +:16]==16'd4500;
        assign published_distances[ch*16 +:16]=matched && !near && !no_echo ? filter_mm[ch*16 +:16] : distances[ch*16 +:16];
    end endgenerate
    wire [15:0] age [0:2];
    assign age[0]=source_ages[15:0];
    assign age[1]=source_ages[31:16];
    assign age[2]=source_ages[47:32];
    integer k;
    encoder_4ch #(.FILTER_CYCLES(ENCODER_FILTER_CYCLES),.INVERT_MASK(ENCODER_INVERT_MASK)) encoders(
        .clk(clk),.rst_n(rst_n),.encoder_a(encoder_a),.encoder_b(encoder_b),
        .positions(positions),.illegal_seen(illegal));
    ultrasonic_3ch #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD),.RAW_MODE(CPU_FILTER_ENABLE)) sonar(
        .clk(clk),.rst_n(rst_n),.rx(sonar_rx),.distance_mm(distances),.distance_age_ms(source_ages),
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
    // Decoder and capture share clk/reset and the same1ms phase. The SNS
    // snapshot uses next_uptime, so carry the chosen ACTUAL observation age
    // across the pending tick. A confirmed older candidate is never age zero.
    wire [15:0] a0=(age[0]==16'hffff ? 16'hffff : age[0]+16'd1);
    wire [15:0] a1=(age[1]==16'hffff ? 16'hffff : age[1]+16'd1);
    wire [15:0] a2=(age[2]==16'hffff ? 16'hffff : age[2]+16'd1);
    always @(posedge clk)begin
        if(!rst_n)begin
            ms_div<=0;uptime_ms<=0;packet_seq<=0;report_ms<=0;baseline<=0;
            sonar_event_pending<=0;previous_online<=0;raw_sonar_seq<=0;raw_sonar_stamp<=0;
            sensor_valid<=0;sensor_packet<=0;
        end else begin
            sensor_valid<=0;previous_online<=online;
            for(k=0;k<3;k=k+1)if(updated[k])begin
                raw_sonar_seq[k*32 +:32]<=raw_sonar_seq[k*32 +:32]+1'b1;
                raw_sonar_stamp[k*32 +:32]<=uptime_ms;
            end
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
                        published_distances[15:0],published_distances[31:16],published_distances[47:32],a0,a1,a2,
                        periodic?WINDOW_MS:16'd0,{4'b0,illegal},8'd0};
                end
            end else ms_div<=ms_div+1'b1;
        end
    end
endmodule
