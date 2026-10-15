`timescale 1ns/1ps
module tb_staggered_pwm;
    reg clk=0,rst_n=0;
    always #5 clk=~clk;
    wire [3:0] pwm,in1,in2;
    wire done,rev;
    four_motor_ramp_test #(.CLK_HZ(100000),.PWM_HZ(1000),.MAX_DUTY_PERCENT(50),
        .RAMP_STEP_MS(1),.START_WAIT_MS(1),.DIR_SETUP_MS(2),.HOLD_MS(8),.STOP_MS(2),
        .MODE(1)) dut(.clk(clk),.rst_n(rst_n),.motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),
        .busy(),.done(done),.reverse_dir(rev),.selected(),.duty_percent());
    wire [3:0] full_pwm,zero_pwm;
    wire full_done,zero_done;
    four_motor_ramp_test #(.CLK_HZ(100000),.PWM_HZ(1000),.MAX_DUTY_PERCENT(100),
        .RAMP_STEP_MS(1),.START_WAIT_MS(1),.DIR_SETUP_MS(2),.HOLD_MS(8),.STOP_MS(2),
        .MODE(1),.REVERSE_TEST(0)) full(.clk(clk),.rst_n(rst_n),.motor_pwm(full_pwm),
        .motor_in1(),.motor_in2(),.busy(),.done(full_done),.reverse_dir(),.selected(),.duty_percent());
    four_motor_ramp_test #(.CLK_HZ(100000),.PWM_HZ(1000),.MAX_DUTY_PERCENT(0),
        .RAMP_STEP_MS(1),.START_WAIT_MS(1),.DIR_SETUP_MS(2),.HOLD_MS(2),.STOP_MS(2),
        .MODE(1),.REVERSE_TEST(0)) zero(.clk(clk),.rst_n(rst_n),.motor_pwm(zero_pwm),
        .motor_in1(),.motor_in2(),.busy(),.done(zero_done),.reverse_dir(),.selected(),.duty_percent());
    integer cycle=0,motor,count_on,last_rise[0:3],first_rise[0:3],high_count[0:3],rises[0:3];
    integer forward_windows=0,reverse_windows=0;
    reg [3:0] previous=0,pre_direction;
    reg [7:0] previous_dirs=0;
    reg previous_hold=0,stable,stable_full,pre_stop,previous_done=0;
    initial begin
        for(motor=0;motor<4;motor=motor+1) begin
            rises[motor]=0;last_rise[motor]=0;first_rise[motor]=0;high_count[motor]=0;
        end
        repeat(4)@(negedge clk);rst_n=1;
        wait(done && full_done && zero_done);repeat(300)@(negedge clk);
        if(forward_windows<1 || reverse_windows<1)$fatal(1,"Missing forward/reverse plateau");
        if(pwm!==0)$fatal(1,"Final stop");
        rst_n=0;@(negedge clk);
        if(pwm!==0 || in1!==0 || in2!==0)$fatal(1,"Reset must stop all phases");
        $display("PASS quarter-period PWM phases, exact period/duty, 0/100%%, 3/4 polarity, stop/reverse/reset");$finish;
    end
    always @(posedge clk) if(rst_n)begin
        stable=(dut.state==3 && dut.elapsed_ms>=2);
        stable_full=(full.state==3 && full.elapsed_ms>=2);
        pre_direction={4{rev}} ^ 4'b1100;
        pre_stop=(dut.duty==0 || !(dut.state==2 || dut.state==3 || dut.state==4));
        #1;cycle=cycle+1;
        if(zero_pwm!==0)$fatal(1,"Staggered 0%% output");
        if(stable_full && full_pwm!==15)$fatal(1,"Staggered 100%% output");
        if(pre_stop && pwm!==0)$fatal(1,"Zero/idle must gate every phase immediately");
        if({in1,in2}!=previous_dirs && (pwm!=0 || previous!=0))$fatal(1,"Direction changes only with PWM zero");
        if(stable)begin
            if(in1!==(~pre_direction) || in2!==pre_direction)$fatal(1,"Motor 3/4 direction mask");
            count_on=pwm[0]+pwm[1]+pwm[2]+pwm[3];
            if(count_on!=2)$fatal(1,"50%% staggered steady waveform must have two outputs high");
            if(!previous_hold)begin
                for(motor=0;motor<4;motor=motor+1)begin rises[motor]=0;high_count[motor]=0;end
            end
            for(motor=0;motor<4;motor=motor+1)begin
                if(pwm[motor] && !previous[motor])begin
                    if(rises[motor]!=0 && cycle-last_rise[motor]!=100)$fatal(1,"Carrier frequency motor %d",motor);
                    if(rises[motor]==0)first_rise[motor]=cycle;
                    last_rise[motor]=cycle;rises[motor]=rises[motor]+1;high_count[motor]=0;
                end
                if(pwm[motor])high_count[motor]=high_count[motor]+1;
                if(!pwm[motor] && previous[motor] && rises[motor]>1 && high_count[motor]!=50)$fatal(1,"PWM duty motor %d",motor);
            end
        end
        if(!stable && previous_hold)begin
            for(motor=0;motor<4;motor=motor+1)begin
                if(rises[motor]<3)$fatal(1,"Missing PWM periods");
                if((first_rise[motor]-first_rise[0]+100)%100!=(100-motor*25)%100)$fatal(1,"Quarter-phase offset");
            end
            if(rev)reverse_windows=reverse_windows+1;else forward_windows=forward_windows+1;
        end
        previous_hold=stable;previous=pwm;previous_dirs={in1,in2};previous_done=done;
    end
    initial begin #5000000;$fatal(1,"Timeout");end
endmodule
