`timescale 1ns/1ps
module tb_four_motor_ramp_test;
    reg clk=0, rst_n=0;
    always #5 clk=!clk;
    wire [3:0] pwm,in1,in2,selected;
    wire busy,done,rev;
    wire [6:0] duty;
    four_motor_ramp_test #(.CLK_HZ(100000),.PWM_HZ(1000),.MAX_DUTY_PERCENT(20),
        .RAMP_STEP_MS(2),.START_WAIT_MS(3),.DIR_SETUP_MS(2),.HOLD_MS(4),.STOP_MS(3),.STAGGER_PWM(0)) dut(
        .clk(clk),.rst_n(rst_n),.motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),
        .selected(selected),.busy(busy),.done(done),.reverse_dir(rev),.duty_percent(duty));
    wire [3:0] full_pwm,zero_pwm;
    wire full_done,zero_done;
    four_motor_ramp_test #(.CLK_HZ(100000),.PWM_HZ(1000),.MAX_DUTY_PERCENT(100),
        .RAMP_STEP_MS(1),.START_WAIT_MS(0),.DIR_SETUP_MS(1),.HOLD_MS(4),.STOP_MS(1),
        .MODE(1),.REVERSE_TEST(0),.STAGGER_PWM(0)) full(
        .clk(clk),.rst_n(rst_n),.motor_pwm(full_pwm),.motor_in1(),.motor_in2(),
        .selected(),.busy(),.done(full_done),.reverse_dir(),.duty_percent());
    four_motor_ramp_test #(.CLK_HZ(100000),.PWM_HZ(1000),.MAX_DUTY_PERCENT(0),
        .RAMP_STEP_MS(1),.START_WAIT_MS(0),.DIR_SETUP_MS(1),.HOLD_MS(1),.STOP_MS(1),
        .MODE(1),.REVERSE_TEST(0),.STAGGER_PWM(0)) zero(
        .clk(clk),.rst_n(rst_n),.motor_pwm(zero_pwm),.motor_in1(),.motor_in2(),
        .selected(),.busy(),.done(zero_done),.reverse_dir(),.duty_percent());
    integer cycle=0,i,highs=0,frame_expected=0,frame_number=0;
    integer seen_fwd=0,seen_rev=0,up_steps=0,down_steps=0;
    reg [3:0] old_in1=0,old_in2=0,old_pwm=0;
    reg [6:0] old_duty=0;
    reg old_running=0,frame_stable=0,saw_full=0,full_was_100=0,was_done=0;
    reg [3:0] frame_mask;
    reg pre_rev;
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        wait(done && full_done && zero_done);
        repeat(1000) @(negedge clk);
        if(seen_fwd!=31 || seen_rev!=31 || up_steps!=200 || down_steps!=200 ||
           !saw_full || frame_number<100) $fatal(1,"Coverage missing %d %d %d %d",seen_fwd,seen_rev,up_steps,down_steps);
        rst_n=0;@(negedge clk);
        if(pwm!==0 || in1!==0 || in2!==0 || done!==0) $fatal(1,"Reset must stop immediately");
        $display("PASS four individual motors + all, both directions, ramps, exact PWM counts, 0/100%%, stop/done/reset");
        $finish;
    end
    // Independent observations at pins; pre-edge values define this PWM slot.
    always @(posedge clk) if(rst_n) begin
        frame_expected=dut.frame_duty[0];
        frame_mask=selected;pre_rev=rev;
        old_running=(dut.state==2 || dut.state==3 || dut.state==4) && duty!=0;
        full_was_100=(full.state==3 && full.frame_duty[0]==100 && full.elapsed_ms>=1);
        was_done=done;
        if(dut.pwm_count[0]==0) begin highs=0;frame_stable=old_running; end
        if(!old_running) frame_stable=0;
        #1;
        cycle=cycle+1;
        if(zero_pwm!==0) $fatal(1,"0%% PWM must stay zero");
        if(in1!=(frame_mask & ~({4{pre_rev}} ^ 4'b1100)) || in2!=(frame_mask & ({4{pre_rev}} ^ 4'b1100)))
            $fatal(1,"Motor 3/4 polarity inversion mismatch");
        if((in1 & in2)!=0) $fatal(1,"Invalid direction combination");
        if(pwm!=0 && (pwm!=frame_mask || !old_running)) $fatal(1,"Unexpected active motor");
        if((in1!=old_in1 || in2!=old_in2) && (pwm!=0 || old_pwm!=0)) $fatal(1,"Direction changed during PWM");
        if(cycle<=300 && pwm!=0) $fatal(1,"Startup delay violated");
        if(duty>20) $fatal(1,"Maximum duty exceeded");
        if(duty>old_duty) begin
            if(duty!=old_duty+1) $fatal(1,"Acceleration step");up_steps=up_steps+1;
        end
        if(duty<old_duty) begin
            if(duty!=old_duty-1) $fatal(1,"Deceleration step");down_steps=down_steps+1;
        end
        if(pwm!=0) begin
            highs=highs+1;
            i=(frame_mask==15 ? 4 : frame_mask==8 ? 3 : frame_mask==4 ? 2 : frame_mask==2 ? 1 : 0);
            if(rev) seen_rev=seen_rev|(1<<i);else seen_fwd=seen_fwd|(1<<i);
        end
        if(dut.pwm_count[0]==0 && frame_stable) begin
            if(highs!=frame_expected) $fatal(1,"PWM high count %d expected %d",highs,frame_expected);
            frame_number=frame_number+1;
        end
        if(full_was_100) begin
            if(full_pwm!==15) $fatal(1,"100%% must stay high");saw_full=1;
        end
        if(was_done && (pwm!==0 || in1!==0 || in2!==0)) $fatal(1,"Done must remain stopped");
        old_in1=in1;old_in2=in2;old_pwm=pwm;old_duty=duty;
    end
    initial begin #2000000;$fatal(1,"Test timed out");end
endmodule
