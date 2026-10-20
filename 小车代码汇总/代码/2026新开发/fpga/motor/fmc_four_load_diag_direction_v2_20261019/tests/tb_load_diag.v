`timescale 1ns/1ps
module tb_load_diag;
    reg clk=0;
    always #5 clk=~clk;
    wire [3:0] pwm,in1,in2,led;
    fmc_motor_load_diag #(.CLK_HZ(100000),.PWM_HZ(1000),.RESET_MS(1),
        .WAIT_MS(2),.RAMP_STEP_MS(1),.JOIN_MS(3),.HOLD_MS(20)) dut(
        .clk50m(clk),.motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.led(led));
    integer cycle=0,i,seen_stages=0,rises[0:3],last_rise[0:3],low_count[0:3];
    reg [3:0] previous_led=0,previous_pwm=0;
    reg observe;
    initial begin
        for(i=0;i<4;i=i+1)begin rises[i]=0;last_rise[i]=0;low_count[i]=0;end
        wait(dut.done);repeat(300)@(negedge clk);
        if(seen_stages!=15 || pwm!==0 || led!==0 || in1!==0 || in2!==0)$fatal(1,"Incomplete stage/stop");
        for(i=0;i<4;i=i+1)if(rises[i]<5)$fatal(1,"Missing four-motor observation");
        $display("PASS staged loads 1/2/3/4, command LEDs, polarity, continuous 60%% PWM with no low-frequency gaps, final stop");$finish;
    end
    always @(posedge clk)begin
        observe=(dut.source.state==3 && dut.active_mask==15 && dut.source.elapsed_ms>=11);
        #1;cycle=cycle+1;
        for(i=0;i<4;i=i+1)if(pwm[i])begin
            if(in1[i]!==1 || in2[i]!==0)$fatal(1,"Corrected wheel polarity M%0d",i+1);
        end
        if(pwm & ~led)$fatal(1,"Inactive command LED with PWM active");
        if(led!=0 && led!=1 && led!=3 && led!=7 && led!=15)$fatal(1,"Unexpected stage mask");
        if(led!=0 && previous_led!=0 && led<previous_led)$fatal(1,"Repeated command shutoff before end");
        if(led==1)seen_stages=seen_stages|1;
        if(led==3)seen_stages=seen_stages|2;
        if(led==7)seen_stages=seen_stages|4;
        if(led==15)seen_stages=seen_stages|8;
        if(observe)begin
            if(led!==15 || in1!==15 || in2!==0)$fatal(1,"Four motor command/polarity");
            for(i=0;i<4;i=i+1)begin
                if(pwm[i] && !previous_pwm[i])begin
                    if(rises[i]>0 && cycle-last_rise[i]!=100)$fatal(1,"Interrupted PWM period");
                    last_rise[i]=cycle;rises[i]=rises[i]+1;
                end
                if(pwm[i])low_count[i]=0;else low_count[i]=low_count[i]+1;
                if(low_count[i]>40)$fatal(1,"Unexpected PWM off interval");
            end
        end
        previous_pwm=pwm;previous_led=led;
    end
    initial begin #2500000;$fatal(1,"Timeout");end
endmodule
