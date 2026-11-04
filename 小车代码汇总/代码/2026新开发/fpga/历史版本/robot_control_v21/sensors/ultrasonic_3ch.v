`timescale 1ns/1ps
// Channel i occupies distance_mm[i*16 +: 16]; state vectors use bit i.
// Fixed mounting: channel 0 FRONT, channel 1 LEFT, channel 2 RIGHT.
// Three physically independent receivers and protocol state machines.
module ultrasonic_3ch #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD = 9600,
    parameter integer DATA_TIMEOUT_MS = 1000,
    parameter integer STOP_MM = 300,
    parameter integer SLOW_MM = 800,
    parameter integer OBSTACLE_MM = 1200,
    parameter integer HYST_MM = 50
)(
    input wire clk, rst_n,
    input wire [2:0] rx,
    output wire [47:0] distance_mm,
    output wire [2:0] distance_valid, online, sensor_fault, obstacle, slow_down, stop_req,
    output wire any_obstacle, any_slow_down, any_stop_req,
    output wire [15:0] front_distance_mm, left_distance_mm, right_distance_mm,
    output wire forward_slow_req, forward_stop_req,
    output wire left_blocked, right_blocked
);
    genvar i;
    generate for (i=0; i<3; i=i+1) begin: channel
        ultrasonic_driver #(
            .CLK_FREQ(CLK_FREQ), .BAUD(BAUD), .DATA_TIMEOUT_MS(DATA_TIMEOUT_MS),
            .STOP_MM(STOP_MM), .SLOW_MM(SLOW_MM),
            .OBSTACLE_MM(OBSTACLE_MM), .HYST_MM(HYST_MM)
        ) u_driver (
            .clk(clk), .rst_n(rst_n), .rx(rx[i]),
            .distance_mm(distance_mm[i*16 +: 16]), .distance_valid(distance_valid[i]),
            .online(online[i]), .sensor_fault(sensor_fault[i]), .obstacle(obstacle[i]),
            .slow_down(slow_down[i]), .stop_req(stop_req[i])
        );
    end endgenerate
    localparam integer FRONT = 0, LEFT = 1, RIGHT = 2;
    assign front_distance_mm = distance_mm[FRONT*16 +: 16];
    assign left_distance_mm = distance_mm[LEFT*16 +: 16];
    assign right_distance_mm = distance_mm[RIGHT*16 +: 16];
    // Forward-only requests. Any sensor fault inhibits motion conservatively.
    // Side proximity alone does not assert a forward request. These are not
    // turn/reverse permissions: vehicle geometry and motion control must decide.
    assign forward_stop_req = (|sensor_fault) | stop_req[FRONT];
    assign forward_slow_req = forward_stop_req | slow_down[FRONT];
    assign left_blocked = sensor_fault[LEFT] | stop_req[LEFT];
    assign right_blocked = sensor_fault[RIGHT] | stop_req[RIGHT];
    // Diagnostic reductions only; do not use as direction-aware motion commands.
    assign any_obstacle = |obstacle;
    assign any_slow_down = |slow_down;
    assign any_stop_req = |stop_req;
endmodule
