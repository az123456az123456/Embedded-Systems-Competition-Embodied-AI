`timescale 1ns/1ps
// A02YYUW UART byte-stream decoder. All outputs synchronous to clk.
// distance_valid pulses only when a checksum-valid, in-range observation is
// incorporated. Nearer returns are immediate; one far spike cannot clear an
// obstacle. Small farther jitter is averaged; large clearance steps require
// two consecutive mutually consistent observations. Rejected far spikes do
// not refresh the previous distance timestamp or online timeout.
// Defaults are engineering choices, not competition-mandated thresholds.
module ultrasonic_sensor #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer FRAME_TIMEOUT_MS = 20,
    parameter integer DATA_TIMEOUT_MS = 1000,
    parameter integer MIN_MM = 30,
    parameter integer MAX_MM = 4500,
    parameter integer STOP_MM = 300,
    parameter integer SLOW_MM = 800,
    parameter integer OBSTACLE_MM = 1200,
    parameter integer HYST_MM = 50,
    parameter integer FAR_STEP_MM = 120,
    parameter integer FAR_CONFIRM_MM = 200
)(
    input wire clk, rst_n,
    input wire [7:0] rx_byte,
    input wire rx_valid,
    output reg [15:0] distance_mm,
    output reg distance_valid,
    output reg online,
    output wire sensor_fault,
    output reg obstacle,
    output reg slow_down,
    output reg stop_req
);
    localparam integer FRAME_TICKS = (CLK_FREQ / 1000) * FRAME_TIMEOUT_MS;
    localparam integer DATA_TICKS = (CLK_FREQ / 1000) * DATA_TIMEOUT_MS;
    reg [1:0] state;
    reg [7:0] hi, lo;
    reg far_pending;
    reg [15:0] far_candidate;
    reg [31:0] frame_age, data_age;
    wire [7:0] checksum = 8'hff + hi + lo;
    wire [31:0] sample_mm = {16'd0, hi, lo};
    wire [15:0] raw_mm = sample_mm[15:0];
    wire [15:0] farther_mm = raw_mm - distance_mm;
    wire [15:0] candidate_delta = raw_mm >= far_candidate ? raw_mm-far_candidate : far_candidate-raw_mm;
    wire large_far = online && raw_mm > distance_mm && farther_mm > FAR_STEP_MM;
    wire confirm_far = far_pending && candidate_delta <= FAR_CONFIRM_MM;
    wire accept_observation = !large_far || confirm_far;
    wire [16:0] average_sum = {1'b0,raw_mm} + {1'b0,distance_mm} + 17'd1;
    wire [15:0] accepted_mm = !online || raw_mm <= distance_mm ? raw_mm :
        large_far ? (raw_mm < far_candidate ? raw_mm : far_candidate) : average_sum[16:1];
    assign sensor_fault = !online;
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= 0; hi <= 0; lo <= 0; far_pending <= 0; far_candidate <= 0;
            frame_age <= 0; data_age <= 0;
            distance_mm <= 0; distance_valid <= 0; online <= 0;
            obstacle <= 1; slow_down <= 1; stop_req <= 1;
        end else begin
            distance_valid <= 0;
            if (data_age < DATA_TICKS - 1) data_age <= data_age + 1;
            else begin
                online <= 0; far_pending <= 0; obstacle <= 1; slow_down <= 1; stop_req <= 1;
            end
            if (state == 0) frame_age <= 0;
            else if (frame_age >= FRAME_TICKS - 1) begin
                state <= 0; frame_age <= 0; far_pending <= 0;
            end else frame_age <= frame_age + 1;

            // Timeout takes precedence over old partial frames; allow a new header.
            if (rx_valid) begin
                frame_age <= 0;
                if (state != 0 && frame_age >= FRAME_TICKS - 1) begin
                    state <= (rx_byte == 8'hff) ? 1 : 0;
                end else case (state)
                    0: if (rx_byte == 8'hff) state <= 1;
                    1: begin hi <= rx_byte; state <= 2; end
                    2: begin lo <= rx_byte; state <= 3; end
                    3: begin
                        state <= 0;
                        if (rx_byte == checksum) begin
                            if (sample_mm >= MIN_MM && sample_mm <= MAX_MM) begin
                                if (accept_observation) begin
                                    distance_mm <= accepted_mm; distance_valid <= 1;
                                    online <= 1; data_age <= 0; far_pending <= 0;
                                    if (accepted_mm <= OBSTACLE_MM) obstacle <= 1;
                                    else if (accepted_mm >= OBSTACLE_MM + HYST_MM) obstacle <= 0;
                                    if (accepted_mm <= SLOW_MM) slow_down <= 1;
                                    else if (accepted_mm >= SLOW_MM + HYST_MM) slow_down <= 0;
                                    if (accepted_mm <= STOP_MM) stop_req <= 1;
                                    else if (accepted_mm >= STOP_MM + HYST_MM) stop_req <= 0;
                                end else begin
                                    far_pending <= 1; far_candidate <= raw_mm;
                                end
                            end else begin
                                online <= 0; far_pending <= 0; obstacle <= 1; slow_down <= 1; stop_req <= 1;
                            end
                        end else begin
                            far_pending <= 0;
                            if (rx_byte == 8'hff) state <= 1;
                        end
                    end
                endcase
            end
        end
    end
endmodule
