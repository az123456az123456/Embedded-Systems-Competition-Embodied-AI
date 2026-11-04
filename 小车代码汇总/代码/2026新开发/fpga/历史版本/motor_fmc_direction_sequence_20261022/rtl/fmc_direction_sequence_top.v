`timescale 1ns/1ps
module fmc_direction_sequence_top #(
    parameter integer CLK_HZ=50_000_000, RESET_MS=20
)(input wire clk50m,
  output wire [3:0] motor_pwm,motor_in1,motor_in2,led,
  output wire uart_tx);
    localparam integer RESET_CYCLES=(CLK_HZ/1000)*RESET_MS;
    reg [31:0] startup_count=0;
    always @(posedge clk50m)
        if(startup_count<RESET_CYCLES) startup_count<=startup_count+1'b1;
    wire reset_n=(startup_count>=RESET_CYCLES);
    wire event_valid,done,overflow;
    wire [3:0] event_id,step_number;
    motor_direction_sequence #(.CLK_HZ(CLK_HZ)) sequence_core(
        .clk(clk50m),.rst_n(reset_n),.motor_pwm(motor_pwm),.motor_in1(motor_in1),
        .motor_in2(motor_in2),.led(led),.event_valid(event_valid),.event_id(event_id),
        .step_number(step_number),.done(done));
    diag_uart_logger #(.CLK_HZ(CLK_HZ)) logger(
        .clk(clk50m),.rst_n(reset_n),.event_valid(event_valid),.event_id(event_id),
        .tx(uart_tx),.overflow(overflow));
endmodule
