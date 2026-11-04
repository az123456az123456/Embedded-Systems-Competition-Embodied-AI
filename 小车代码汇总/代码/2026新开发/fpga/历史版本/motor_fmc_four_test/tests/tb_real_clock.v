`timescale 1ns/1ps
module tb_real_clock;
    reg clk=0,rst_n=0;
    always #10 clk=~clk;
    wire [3:0] pwm,in1,in2;
    wire done;
    four_motor_ramp_test #(.START_WAIT_MS(0),.DIR_SETUP_MS(1),.RAMP_STEP_MS(1),
        .HOLD_MS(3),.STOP_MS(1),.MODE(1),.REVERSE_TEST(0)) dut(
        .clk(clk),.rst_n(rst_n),.motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),
        .busy(),.done(done),.reverse_dir(),.selected(),.duty_percent());
    integer motor,cycle=0,on_count,rises[0:3],last_rise[0:3],first_rise[0:3],high_count[0:3];
    reg [3:0] previous=0;
    reg stable=0;
    initial begin
        for(motor=0;motor<4;motor=motor+1)begin rises[motor]=0;last_rise[motor]=0;high_count[motor]=0;end
        repeat(3)@(negedge clk);rst_n=1;
        wait(done);repeat(3000)@(negedge clk);
        for(motor=0;motor<4;motor=motor+1)begin
            if(rises[motor]<10)$fatal(1,"Missing actual-clock PWM cycles");
            if((first_rise[motor]-first_rise[0]+2500)%2500!=(2500-motor*625)%2500)$fatal(1,"Actual-clock phase offset");
        end
        if(pwm!==0)$fatal(1,"Final stop");
        $display("PASS actual 50MHz: 20kHz period=2500, 60%% high=1500, phase spacing=625 clocks, corrected direction");$finish;
    end
    always @(posedge clk) if(rst_n)begin
        stable=(dut.state==3 && dut.elapsed_ms>=1);
        #1;cycle=cycle+1;
        if(stable)begin
            if(in1!==4'b0011 || in2!==4'b1100)$fatal(1,"Actual-clock direction polarity");
            on_count=pwm[0]+pwm[1]+pwm[2]+pwm[3];
            if(on_count<2 || on_count>3)$fatal(1,"60%% staggered steady wave overlap");
            for(motor=0;motor<4;motor=motor+1)begin
                if(pwm[motor] && !previous[motor])begin
                    if(rises[motor]!=0 && cycle-last_rise[motor]!=2500)$fatal(1,"20kHz frequency");
                    if(rises[motor]==0)first_rise[motor]=cycle;
                    last_rise[motor]=cycle;rises[motor]=rises[motor]+1;high_count[motor]=0;
                end
                if(pwm[motor])high_count[motor]=high_count[motor]+1;
                if(!pwm[motor] && previous[motor] && rises[motor]>1 && high_count[motor]!=1500)$fatal(1,"60%% duty width");
            end
        end
        previous=pwm;
    end
    initial begin #140000000;$fatal(1,"Timeout");end
endmodule
