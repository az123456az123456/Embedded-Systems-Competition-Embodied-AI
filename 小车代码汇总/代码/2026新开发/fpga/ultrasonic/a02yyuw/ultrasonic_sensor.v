`timescale 1ns/1ps
// A02YYUW UART byte-stream decoder. All outputs synchronous to clk.
// distance_valid pulses for one clock only for checksum-valid, in-range frames.
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
    parameter integer HYST_MM = 50
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
    reg [31:0] frame_age, data_age;
    wire [7:0] checksum = 8'hff + hi + lo;
    wire [31:0] sample_mm = {16'd0, hi, lo};
    assign sensor_fault = !online;
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= 0; hi <= 0; lo <= 0;
            frame_age <= 0; data_age <= 0;
            distance_mm <= 0; distance_valid <= 0; online <= 0;
            obstacle <= 1; slow_down <= 1; stop_req <= 1;
        end else begin
            distance_valid <= 0;
            if (data_age < DATA_TICKS - 1) data_age <= data_age + 1;
            else begin
                online <= 0; obstacle <= 1; slow_down <= 1; stop_req <= 1;
            end
            if (state == 0) frame_age <= 0;
            else if (frame_age >= FRAME_TICKS - 1) begin
                state <= 0; frame_age <= 0;
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
                                distance_mm <= sample_mm[15:0]; distance_valid <= 1;
                                online <= 1; data_age <= 0;
                                if (sample_mm <= OBSTACLE_MM) obstacle <= 1;
                                else if (sample_mm >= OBSTACLE_MM + HYST_MM) obstacle <= 0;
                                if (sample_mm <= SLOW_MM) slow_down <= 1;
                                else if (sample_mm >= SLOW_MM + HYST_MM) slow_down <= 0;
                                if (sample_mm <= STOP_MM) stop_req <= 1;
                                else if (sample_mm >= STOP_MM + HYST_MM) stop_req <= 0;
                            end else begin
                                online <= 0; obstacle <= 1; slow_down <= 1; stop_req <= 1;
                            end
                        end else if (rx_byte == 8'hff) state <= 1;
                    end
                endcase
            end
        end
    end
endmodule
