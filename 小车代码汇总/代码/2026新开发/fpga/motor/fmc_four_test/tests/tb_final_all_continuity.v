`timescale 1ns/1ps
module tb_final_all_continuity;
    reg clk=0,rst_n=0;
    always #5 clk=~clk;
    wire [3:0] pwm;
    wire done;
    four_motor_ramp_test #(.CLK_HZ(100000),.PWM_HZ(1000),.RAMP_STEP_MS(1),
        .START_WAIT_MS(1),.DIR_SETUP_MS(1),.HOLD_MS(8),.STOP_MS(1)) dut(
        .clk(clk),.rst_n(rst_n),.motor_pwm(pwm),.motor_in1(),.motor_in2(),
        .busy(),.done(done),.reverse_dir(),.selected(),.duty_percent());
    integer cycle=0,i,individual_seen=0,windows=0,rises[0:3],last[0:3],off_count[0:3];
    reg observing,previous_observing=0;
    reg [3:0] previous_pwm=0;
    initial begin
        for(i=0;i<4;i=i+1)begin rises[i]=0;last[i]=0;off_count[i]=0;end
        repeat(3)@(negedge clk);rst_n=1;
        wait(done);repeat(100)@(negedge clk);
        if(individual_seen!=15 || windows!=2)$fatal(1,"Missing final stage coverage");
        if(pwm!==0)$fatal(1,"Final stop");
        $display("PASS final all-motor stage after individual tests: continuous PWM both directions, no extra off intervals");$finish;
    end
    always @(posedge clk)if(rst_n)begin
        observing=(dut.target==4 && dut.state==3 && dut.elapsed_ms>=2);
        if(dut.target<4 && dut.duty==60)individual_seen=individual_seen|(1<<dut.target);
        #1;cycle=cycle+1;
        if(observing)begin
            if(individual_seen!=15)$fatal(1,"All stage entered before individual tests");
            if(!previous_observing)for(i=0;i<4;i=i+1)begin rises[i]=0;off_count[i]=0;end
            for(i=0;i<4;i=i+1)begin
                if(pwm[i] && !previous_pwm[i])begin
                    if(rises[i]>0 && cycle-last[i]!=100)$fatal(1,"Final stage PWM interrupted");
                    rises[i]=rises[i]+1;last[i]=cycle;
                end
                if(pwm[i])off_count[i]=0;else off_count[i]=off_count[i]+1;
                if(off_count[i]>40)$fatal(1,"Final stage low interval exceeds 40%% of carrier");
            end
        end
        if(!observing && previous_observing)begin
            for(i=0;i<4;i=i+1)if(rises[i]<4)$fatal(1,"Insufficient final-stage waveform");
            windows=windows+1;
        end
        previous_observing=observing;previous_pwm=pwm;
    end
    initial begin #1800000;$fatal(1,"Timeout");end
endmodule
