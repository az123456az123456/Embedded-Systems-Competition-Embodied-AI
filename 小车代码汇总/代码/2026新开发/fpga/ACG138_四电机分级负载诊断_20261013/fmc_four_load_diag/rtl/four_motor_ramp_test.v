`timescale 1ns/1ps
// TB6612: STBY is supplied externally. PWM=0 in every stopped state.
// MODE 0: motors 1..4 individually, then all four; 1: all only; 2: individual only.
module four_motor_ramp_test #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer PWM_HZ=20_000,
    parameter integer MAX_DUTY_PERCENT=60,
    parameter integer RAMP_STEP_MS=40,
    parameter integer START_WAIT_MS=3000,
    parameter integer DIR_SETUP_MS=100,
    parameter integer HOLD_MS=3000,
    parameter integer STOP_MS=1000,
    parameter integer MODE=0,
    parameter integer REVERSE_TEST=1,
    parameter [3:0] INVERT_MASK=4'b1100,
    parameter integer STAGGER_PWM=1
)(
    input wire clk, rst_n,
    output reg [3:0] motor_pwm,
    output reg [3:0] motor_in1, motor_in2,
    output wire busy, done, reverse_dir,
    output wire [3:0] selected,
    output wire [6:0] duty_percent
);
    localparam integer MS_CYCLES=CLK_HZ/1000;
    localparam integer PWM_CYCLES=CLK_HZ/PWM_HZ;
    localparam integer CYCLES_PER_PERCENT=PWM_CYCLES/100;
    localparam integer PWM_COUNTER_WIDTH=$clog2(PWM_CYCLES);
    localparam WAIT=0,ARM=1,UP=2,HOLD=3,DOWN=4,PAUSE=5,DONE=6;
    reg [2:0] state;
    reg [2:0] target;
    reg backwards;
    reg [6:0] duty;
    reg [6:0] frame_duty [0:3];
    reg [PWM_COUNTER_WIDTH-1:0] pwm_count [0:3];
    reg [31:0] ms_count, elapsed_ms;
    wire [3:0] electrical_reverse={4{backwards}} ^ INVERT_MASK;
    wire ms_tick=(ms_count==MS_CYCLES-1);
    wire running=(state==UP || state==HOLD || state==DOWN);
    assign selected=(state==WAIT || state==DONE) ? 4'b0 :
                    target==4 ? 4'b1111 : (4'b0001<<target);
    assign busy=(state!=WAIT && state!=DONE);
    assign done=(state==DONE);
    assign reverse_dir=backwards;
    assign duty_percent=duty;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) ms_count<=0;
        else if(ms_tick) ms_count<=0;
        else ms_count<=ms_count+1'b1;
    end
    // All command changes happen on a millisecond boundary.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=WAIT; target<=(MODE==1 ? 4 : 0); backwards<=0;
            duty<=0; elapsed_ms<=0;
        end else if(ms_tick) begin
            case(state)
            WAIT: if(START_WAIT_MS==0 || elapsed_ms>=START_WAIT_MS-1) begin
                elapsed_ms<=0; state<=ARM;
            end else elapsed_ms<=elapsed_ms+1'b1;
            ARM: if(elapsed_ms>=DIR_SETUP_MS-1) begin
                elapsed_ms<=0; state<=UP;
            end else elapsed_ms<=elapsed_ms+1'b1;
            UP: if(elapsed_ms>=RAMP_STEP_MS-1) begin
                elapsed_ms<=0;
                if(duty>=MAX_DUTY_PERCENT) state<=HOLD;
                else begin
                    duty<=duty+1'b1;
                    if(duty==MAX_DUTY_PERCENT-1) state<=HOLD;
                end
            end else elapsed_ms<=elapsed_ms+1'b1;
            HOLD: if(elapsed_ms>=HOLD_MS-1) begin
                elapsed_ms<=0; state<=DOWN;
            end else elapsed_ms<=elapsed_ms+1'b1;
            DOWN: if(elapsed_ms>=RAMP_STEP_MS-1) begin
                elapsed_ms<=0;
                if(duty<=1) begin duty<=0; state<=PAUSE; end
                else duty<=duty-1'b1;
            end else elapsed_ms<=elapsed_ms+1'b1;
            PAUSE: if(elapsed_ms>=STOP_MS-1) begin
                elapsed_ms<=0;
                if(REVERSE_TEST && !backwards) begin backwards<=1; state<=ARM; end
                else if(MODE==1 || target==(MODE==2 ? 3 : 4)) begin
                    duty<=0; state<=DONE;
                end else begin target<=target+1'b1; backwards<=0; state<=ARM; end
            end else elapsed_ms<=elapsed_ms+1'b1;
            default: begin state<=DONE; duty<=0; elapsed_ms<=0; end
            endcase
        end
    end

    // Each motor latches duty at its own carrier boundary. Quarter-period
    // offsets spread switching edges; they do not replace adequate VM supply.
    // A zero command gates all four outputs immediately, regardless of phase.
    integer motor;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            motor_pwm<=0; motor_in1<=0; motor_in2<=0;
            for(motor=0;motor<4;motor=motor+1) begin
                pwm_count[motor]<=STAGGER_PWM ? motor*(PWM_CYCLES/4) : 0;
                frame_duty[motor]<=0;
            end
        end else begin
            motor_in1<=selected & ~electrical_reverse;
            motor_in2<=selected & electrical_reverse;
            for(motor=0;motor<4;motor=motor+1) begin
                if(pwm_count[motor]==PWM_CYCLES-1) begin
                    pwm_count[motor]<=0; frame_duty[motor]<=duty;
                end else pwm_count[motor]<=pwm_count[motor]+1'b1;
                motor_pwm[motor]<=selected[motor] && running && duty!=0 &&
                    pwm_count[motor]<CYCLES_PER_PERCENT*frame_duty[motor];
            end
        end
    end
    // synthesis translate_off
    initial begin
        if(CLK_HZ<1000 || CLK_HZ%1000!=0 || PWM_HZ<=0 || CLK_HZ%PWM_HZ!=0 ||
           PWM_CYCLES<100 || PWM_CYCLES%100!=0 || MAX_DUTY_PERCENT<0 ||
           MAX_DUTY_PERCENT>100 || RAMP_STEP_MS<1 || START_WAIT_MS<0 ||
           DIR_SETUP_MS<1 || HOLD_MS<1 || STOP_MS<1 || MODE<0 || MODE>2 ||
           (STAGGER_PWM!=0 && STAGGER_PWM!=1))
            $fatal(1,"Invalid motor test timing parameters");
    end
    // synthesis translate_on
endmodule
