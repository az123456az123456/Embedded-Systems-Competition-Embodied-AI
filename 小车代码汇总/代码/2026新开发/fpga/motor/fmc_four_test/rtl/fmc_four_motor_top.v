module fmc_four_motor_top(
    input wire clk50m,
    output wire [3:0] motor_pwm, motor_in1, motor_in2,
    output wire [3:0] led
);
    reg [20:0] startup=0;
    always @(posedge clk50m) if(!startup[20]) startup<=startup+1'b1;
    wire reset_n=startup[20];
    wire busy,done,backwards;
    wire [3:0] selected;
    reg [25:0] heartbeat=0;
    always @(posedge clk50m) heartbeat<=heartbeat+1'b1;
    four_motor_ramp_test test(
        .clk(clk50m),.rst_n(reset_n),
        .motor_pwm(motor_pwm),.motor_in1(motor_in1),.motor_in2(motor_in2),
        .busy(busy),.done(done),.reverse_dir(backwards),.selected(selected),.duty_percent()
    );
    assign led={done,backwards,busy,heartbeat[25]};
endmodule
