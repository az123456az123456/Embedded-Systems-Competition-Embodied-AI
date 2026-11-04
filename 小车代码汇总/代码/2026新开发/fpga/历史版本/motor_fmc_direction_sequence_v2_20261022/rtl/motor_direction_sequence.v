`timescale 1ns/1ps
// One-shot, open-loop bench diagnostic. Motor numbers are fixed P3 channels.
// Hypothesis: M1 right-front, M2 left-rear, M3 right-rear, M4 left-front.
// v1 (mask 9) observation: fronts M1/M4 forward, rears M2/M3 backward.
// v2 toggles only the rear polarities: 9 XOR 6 = 15. Retest on hardware.
module motor_direction_sequence #(
    parameter integer CLK_HZ=50_000_000, PWM_HZ=20_000,
    parameter integer WAIT_MS=8000, PREP_MS=100, HOLD_MS=2000, STOP_MS=2000,
    parameter integer RAMP_STEP_MS=40, RAMP_STEP_PERCENT=2,
    parameter integer FAST_PERCENT=40, SLOW_PERCENT=20,
    parameter [3:0] INVERT_MASK=4'b1111
)(
    input wire clk, rst_n,
    output wire [3:0] motor_pwm, motor_in1, motor_in2,
    output wire [3:0] led,
    output reg event_valid,
    output reg [3:0] event_id,
    output wire [3:0] step_number,
    output wire done
);
    localparam integer MS_CYCLES=CLK_HZ/1000, PWM_CYCLES=CLK_HZ/PWM_HZ;
    localparam integer PWM_BITS=$clog2(PWM_CYCLES);
    localparam [6:0] FAST=FAST_PERCENT[6:0],SLOW=SLOW_PERCENT[6:0],RAMP=RAMP_STEP_PERCENT[6:0];
    function [PWM_BITS-1:0] pwm_value;
        input integer value;
        begin pwm_value=value[PWM_BITS-1:0];end
    endfunction
    function [PWM_BITS:0] duty_cycles;
        input [6:0] percent;
        reg [31:0] wide;
        begin wide=(PWM_CYCLES/100)*{25'b0,percent};duty_cycles=wide[PWM_BITS:0];end
    endfunction
    localparam WAIT=0, PREP=1, UP=2, HOLD=3, DOWN=4, PAUSE=5, DONE=6;
    reg [2:0] state;
    reg [3:0] step;
    reg [31:0] ms_count, elapsed, heartbeat;
    reg [6:0] magnitude[0:3], target[0:3];
    reg [6:0] frame_duty[0:3];
    reg [PWM_BITS-1:0] pwm_count[0:3];
    reg [3:0] negative;
    reg [6:0] left_duty, right_duty;
    reg left_negative, right_negative;
    wire tick=(ms_count==MS_CYCLES-1);
    wire running=(state==UP || state==HOLD || state==DOWN);
    reg up_complete, down_complete;
    integer k, j, p;

    // Stages 1..4 isolate each channel. Remaining stages use left M2/M4.
    always @* begin
        left_duty=0; right_duty=0; left_negative=0; right_negative=0;
        case(step)
          4: begin left_duty=FAST;right_duty=FAST;end
          5: begin left_duty=FAST;right_duty=FAST;left_negative=1;right_negative=1;end
          6: begin left_duty=FAST;right_duty=FAST;left_negative=1;end
          7: begin left_duty=FAST;right_duty=FAST;right_negative=1;end
          8: begin left_duty=SLOW;right_duty=FAST;end
          9: begin left_duty=FAST;right_duty=SLOW;end
          // Back-left is a path to the car's rear-left; its nose yaws right.
          10:begin left_duty=SLOW;right_duty=FAST;left_negative=1;right_negative=1;end
          11:begin left_duty=FAST;right_duty=SLOW;left_negative=1;right_negative=1;end
          default: begin end
        endcase
        negative=0;
        for(k=0;k<4;k=k+1) begin
            if(step<4) target[k]=(step==k[3:0] ? FAST : 7'b0);
            else if(k==1 || k==3) begin target[k]=left_duty;negative[k]=left_negative;end
            else begin target[k]=right_duty;negative[k]=right_negative;end
        end
        up_complete=1;down_complete=1;
        for(k=0;k<4;k=k+1) begin
            if(magnitude[k]+RAMP<target[k]) up_complete=0;
            if(magnitude[k]>RAMP) down_complete=0;
        end
    end
    assign step_number=(state==WAIT || state==DONE) ? 0 : step+1'b1;
    assign done=(state==DONE);
    // LEDs are active high on this board. D0 is the least significant bit.
    assign led=state==WAIT ? {4{heartbeat>=500}} : state==DONE ? 4'b1111 : step+1'b1;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=WAIT;step<=0;ms_count<=0;elapsed<=0;heartbeat<=0;
            event_valid<=0;event_id<=0;
            for(j=0;j<4;j=j+1) magnitude[j]<=0;
        end else begin
            event_valid<=0;
            if(tick) begin
                ms_count<=0;
                heartbeat<=heartbeat==999 ? 0 : heartbeat+1'b1;
                case(state)
                  WAIT: if(elapsed>=WAIT_MS-1) begin
                      state<=PREP;elapsed<=0;event_valid<=1;event_id<=1;
                  end else elapsed<=elapsed+1'b1;
                  PREP: if(elapsed>=PREP_MS-1) begin state<=UP;elapsed<=0;end
                        else elapsed<=elapsed+1'b1;
                  UP: if(elapsed>=RAMP_STEP_MS-1) begin
                      elapsed<=0;
                      for(j=0;j<4;j=j+1)
                          if(magnitude[j]+RAMP>=target[j]) magnitude[j]<=target[j];
                          else magnitude[j]<=magnitude[j]+RAMP;
                      if(up_complete) state<=HOLD;
                  end else elapsed<=elapsed+1'b1;
                  HOLD:if(elapsed>=HOLD_MS-1) begin state<=DOWN;elapsed<=0;end
                        else elapsed<=elapsed+1'b1;
                  DOWN:if(elapsed>=RAMP_STEP_MS-1) begin
                      elapsed<=0;
                      for(j=0;j<4;j=j+1)
                          if(magnitude[j]<=RAMP) magnitude[j]<=0;
                          else magnitude[j]<=magnitude[j]-RAMP;
                      if(down_complete) state<=PAUSE;
                  end else elapsed<=elapsed+1'b1;
                  PAUSE:if(elapsed>=STOP_MS-1) begin
                      elapsed<=0;
                      if(step==11) begin state<=DONE;event_valid<=1;event_id<=13;end
                      else begin state<=PREP;step<=step+1'b1;event_valid<=1;event_id<=step+2;end
                  end else elapsed<=elapsed+1'b1;
                  default: begin state<=DONE;elapsed<=0;for(j=0;j<4;j=j+1)magnitude[j]<=0;end
                endcase
            end else ms_count<=ms_count+1'b1;
        end
    end
    // Equal carriers, quarter-period phase spacing, per-frame duty latching.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(p=0;p<4;p=p+1) begin pwm_count[p]<=pwm_value(p*(PWM_CYCLES/4));frame_duty[p]<=0;end
        end else for(p=0;p<4;p=p+1) begin
            if(pwm_count[p]==pwm_value(PWM_CYCLES-1)) begin pwm_count[p]<=0;frame_duty[p]<=magnitude[p];end
            else pwm_count[p]<=pwm_count[p]+1'b1;
        end
    end
    genvar n;
    generate for(n=0;n<4;n=n+1) begin:g_motor
        wire active=rst_n && running && magnitude[n]!=0 && target[n]!=0;
        wire reverse_electrical=negative[n]^INVERT_MASK[n];
        assign motor_pwm[n]=active && {1'b0,pwm_count[n]}<duty_cycles(frame_duty[n]);
        assign motor_in1[n]=active && !reverse_electrical;
        assign motor_in2[n]=active && reverse_electrical;
    end endgenerate
    // synthesis translate_off
    initial if(CLK_HZ<1000 || CLK_HZ%1000!=0 || PWM_HZ<1 || CLK_HZ%PWM_HZ!=0 ||
       PWM_CYCLES<100 || PWM_CYCLES%100!=0 || WAIT_MS<1 || PREP_MS<1 || HOLD_MS<1 || STOP_MS<1 ||
       RAMP_STEP_MS<1 || RAMP_STEP_PERCENT<1 || RAMP_STEP_PERCENT>60 || FAST_PERCENT>60 || SLOW_PERCENT<1 ||
       SLOW_PERCENT>=FAST_PERCENT) $fatal(1,"Invalid diagnostic parameters");
    // synthesis translate_on
endmodule
