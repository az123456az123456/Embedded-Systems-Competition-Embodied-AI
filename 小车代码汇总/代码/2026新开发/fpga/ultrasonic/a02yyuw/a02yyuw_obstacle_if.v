`timescale 1ns/1ps
// Motor controller interface: FRONT=0, LEFT=1, RIGHT=2. Distances in mm.
// *_data_valid is a level; distance_updated is a one-clock pulse per direction.
// An invalid distance retains the last good value; never use it without validity.
// *_obstacle means a confirmed detection, masked to zero when data is invalid.
// Unknown is signalled by sensor_fault, and any fault asserts forward_stop_req.
module a02yyuw_obstacle_if #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD = 9600,
    parameter integer DATA_TIMEOUT_MS = 1000,
    parameter integer OBSTACLE_MM = 1200,
    parameter integer SLOW_MM = 800,
    parameter integer STOP_MM = 300,
    parameter integer HYST_MM = 50
)(
    input wire clk, rst_n,
    input wire us_rx_front, us_rx_left, us_rx_right,
    output wire front_obstacle, left_obstacle, right_obstacle,
    output wire [15:0] front_distance_mm, left_distance_mm, right_distance_mm,
    output wire front_data_valid, left_data_valid, right_data_valid,
    output wire [2:0] distance_updated, sensor_fault,
    output wire forward_slow_req, forward_stop_req, left_blocked, right_blocked
);
    wire [2:0] online, obstacle;
    ultrasonic_3ch #(
        .CLK_FREQ(CLK_FREQ), .BAUD(BAUD), .DATA_TIMEOUT_MS(DATA_TIMEOUT_MS),
        .OBSTACLE_MM(OBSTACLE_MM), .SLOW_MM(SLOW_MM),
        .STOP_MM(STOP_MM), .HYST_MM(HYST_MM)
    ) u_sensors (
        .clk(clk), .rst_n(rst_n), .rx({us_rx_right,us_rx_left,us_rx_front}),
        .distance_mm(), .distance_valid(distance_updated), .online(online),
        .sensor_fault(sensor_fault), .obstacle(obstacle), .slow_down(), .stop_req(),
        .any_obstacle(), .any_slow_down(), .any_stop_req(),
        .front_distance_mm(front_distance_mm), .left_distance_mm(left_distance_mm),
        .right_distance_mm(right_distance_mm), .forward_slow_req(forward_slow_req),
        .forward_stop_req(forward_stop_req), .left_blocked(left_blocked),
        .right_blocked(right_blocked)
    );
    assign front_data_valid = online[0];
    assign left_data_valid = online[1];
    assign right_data_valid = online[2];
    assign front_obstacle = online[0] & obstacle[0];
    assign left_obstacle = online[1] & obstacle[1];
    assign right_obstacle = online[2] & obstacle[2];
endmodule
