`timescale 1ns/1ps
// One complete A02YYUW automatic-UART driver; instantiate once per sensor.
module ultrasonic_driver #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD = 9600,
    parameter integer FRAME_TIMEOUT_MS = 20,
    parameter integer DATA_TIMEOUT_MS = 1000,
    parameter integer STOP_MM = 300,
    parameter integer SLOW_MM = 800,
    parameter integer OBSTACLE_MM = 1200,
    parameter integer HYST_MM = 50,
    parameter integer RAW_MODE = 0,
    parameter integer MA_WINDOW = 4,
    parameter integer MA_MAX_SPAN_MS = 300,
    parameter integer FAST_NEAR_DELTA_MM = 50,
    parameter integer FAST_NEAR_MM = 350
)(
    input wire clk, rst_n, rx,
    output wire [15:0] distance_mm, distance_age_ms,
    output wire distance_valid, online, sensor_fault, obstacle, slow_down, stop_req
);
    wire [7:0] rx_byte;
    wire rx_valid;
    a02_uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx), .data(rx_byte), .valid(rx_valid)
    );
    ultrasonic_sensor #(
        .CLK_FREQ(CLK_FREQ), .FRAME_TIMEOUT_MS(FRAME_TIMEOUT_MS),
        .DATA_TIMEOUT_MS(DATA_TIMEOUT_MS), .STOP_MM(STOP_MM),
        .SLOW_MM(SLOW_MM), .OBSTACLE_MM(OBSTACLE_MM), .HYST_MM(HYST_MM),
        .RAW_MODE(RAW_MODE), .MA_WINDOW(MA_WINDOW), .MA_MAX_SPAN_MS(MA_MAX_SPAN_MS),
        .FAST_NEAR_DELTA_MM(FAST_NEAR_DELTA_MM), .FAST_NEAR_MM(FAST_NEAR_MM)
    ) u_sensor (
        .clk(clk), .rst_n(rst_n), .rx_byte(rx_byte), .rx_valid(rx_valid),
        .distance_mm(distance_mm), .distance_valid(distance_valid), .distance_age_ms(distance_age_ms), .online(online),
        .sensor_fault(sensor_fault), .obstacle(obstacle),
        .slow_down(slow_down), .stop_req(stop_req)
    );
endmodule
