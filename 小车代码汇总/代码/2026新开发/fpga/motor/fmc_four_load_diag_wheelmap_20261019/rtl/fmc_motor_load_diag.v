`timescale 1ns/1ps
// Load isolation: M1 ramps up; M2/M3/M4 join one at a time at fixed duty.
// Wheel map: M1 right rear, M2 left rear, M3 right front, M4 left front.
// Reverse the two right-side motors (M1/M3); retain P3 wiring.
// No repeated stops or reverse command during the fixed-speed observation.
module fmc_motor_load_diag #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer PWM_HZ=20_000,
    parameter integer RESET_MS=20,
    parameter integer WAIT_MS=3000,
    parameter integer RAMP_STEP_MS=40,
    parameter integer JOIN_MS=1000,
    parameter integer HOLD_MS=15000
)(
    input wire clk50m,
    output reg [3:0] motor_pwm=0,motor_in1=0,motor_in2=0,
    output reg [3:0] led=0
);
    localparam integer RESET_CYCLES=(CLK_HZ/1000)*RESET_MS;
    localparam integer JOIN_CYCLES=(CLK_HZ/1000)*JOIN_MS;
    reg [31:0] startup_count=0;
    always @(posedge clk50m) if(startup_count<RESET_CYCLES) startup_count<=startup_count+1'b1;
    wire reset_n=(startup_count>=RESET_CYCLES);
    wire [3:0] raw_pwm,raw_in1,raw_in2;
    wire busy,done;
    wire [6:0] duty;
    reg [3:0] active_mask=4'b0001;
    reg [31:0] join_count=0;
    reg raw_command_active=0;
    four_motor_ramp_test #(
        .CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ),.MAX_DUTY_PERCENT(60),
        .START_WAIT_MS(WAIT_MS),.RAMP_STEP_MS(RAMP_STEP_MS),
        .HOLD_MS(HOLD_MS),.MODE(1),.REVERSE_TEST(0),
        .INVERT_MASK(4'b0101),.STAGGER_PWM(1)
    ) source(
        .clk(clk50m),.rst_n(reset_n),.motor_pwm(raw_pwm),
        .motor_in1(raw_in1),.motor_in2(raw_in2),
        .busy(busy),.done(done),.reverse_dir(),.selected(),.duty_percent(duty)
    );
    always @(posedge clk50m) begin
        if(!reset_n) begin active_mask<=1;join_count<=0;end
        else if(duty==60 && busy && active_mask!=15)begin
            if(join_count==JOIN_CYCLES-1)begin
                active_mask<=(active_mask<<1)|4'b0001;join_count<=0;
            end else join_count<=join_count+1'b1;
        end
        // Match the signal pipeline for PWM, direction and command LEDs.
        if(!reset_n)begin motor_pwm<=0;motor_in1<=0;motor_in2<=0;led<=0;raw_command_active<=0;end
        else begin
            raw_command_active<=busy && duty!=0;
            motor_pwm<=raw_pwm & active_mask;
            motor_in1<=raw_in1 & active_mask;
            motor_in2<=raw_in2 & active_mask;
            led<={4{raw_command_active}} & active_mask;
        end
    end
    // synthesis translate_off
    initial if(RESET_MS<1 || JOIN_MS<1 || HOLD_MS<=3*JOIN_MS)
        $fatal(1,"Diagnostic hold must exceed the three join intervals");
    // synthesis translate_on
endmodule
