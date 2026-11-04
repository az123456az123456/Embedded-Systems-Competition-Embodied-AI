`timescale 1ns/1ps
// User-confirmed chassis: M1 RR, M2 LR, M3 RF, M4 LF; all positive IN1=1.
module tb_rotation_layout;
    reg clk=0,rst_n=0,valid=0;
    reg [127:0] command_data=0;
    wire [3:0] pwm,in1,in2,flags;
    wire signed [15:0] left,right;
    wire [127:0] status;
    wire [31:0] sequence_out;
    reg [31:0] sequence_in=0;
    integer i,j;
    reg [3:0] observed;
    always #5000 clk=~clk;
    motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),
        .WATCHDOG_MS(80),.RAMP_STEP_MS(1),.REVERSAL_MS(1)) dut(
        .clk(clk),.rst_n(rst_n),.command_valid(valid),.command_data(command_data),
        .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),
        .status_valid(),.status_data(status),.last_sequence(sequence_out),
        .applied_left(left),.applied_right(right));
    task cycles;
        input integer count;
        begin repeat(count) @(negedge clk); end
    endtask
    task command;
        input bit arm;
        input signed [15:0] l,r;
        begin
            sequence_in=sequence_in+1;
            @(negedge clk);
            command_data={32'h4d4f5431,8'd1,7'd0,arm,sequence_in,l,r,8'h0a,8'h00};
            valid=1;
            @(negedge clk);valid=0;
            cycles(4);
        end
    endtask
    task settle;
        input signed [15:0] l,r;
        begin
            for(i=0;i<5;i=i+1) begin command(1,l,r);cycles(1000);end
        end
    endtask
    task all_pwm_present;
        begin
            observed=0;
            for(j=0;j<110;j=j+1) begin cycles(1);observed=observed|pwm;end
            if(observed!==4'b1111) $fatal(1,"one motor has no PWM");
        end
    endtask
    initial begin
        cycles(3);rst_n=1;cycles(3);
        if(pwm!==0 || in1!==0 || in2!==0 || status[15:0]!==16'h0a0f)
            $fatal(1,"reset must be stopped with final layout defaults");
        command(1,300,-300);cycles(400);
        if(pwm!==0 || flags[0])$fatal(1,"motion before STOP handshake");
        command(0,0,0);
        // Clockwise, viewed from above: left wheels forward; right backward.
        settle(300,-300);
        if(left!==300 || right!==-300 || in1!==4'b1010 || in2!==4'b0101)
            $fatal(1,"clockwise side mapping/polarity incorrect");
        all_pwm_present;
        // Counterclockwise: reverse both sides, with output blanking/deadtime.
        command(1,-300,300);cycles(3);
        if(pwm!==0)$fatal(1,"reversal must blank all four outputs");
        cycles(110);settle(-300,300);
        if(left!==-300 || right!==300 || in1!==4'b0101 || in2!==4'b1010)
            $fatal(1,"counterclockwise side mapping/polarity incorrect");
        all_pwm_present;
        command(1,0,0);
        if(pwm!==0 || in1!==0 || in2!==0 || left!==0 || right!==0)
            $fatal(1,"armed PWM=0 did not stop immediately");
        settle(300,300);all_pwm_present;
        if(in1!==4'b1111 || in2!==0)$fatal(1,"forward direction incorrect");
        command(0,0,0);
        if(pwm!==0 || in1!==0 || in2!==0 || flags[0])
            $fatal(1,"unarmed STOP failed");
        settle(300,-300);cycles(8100);
        if(pwm!==0 || in1!==0 || in2!==0 || !flags[1] || flags[0] || flags[2])
            $fatal(1,"watchdog did not stop all motors");
        $display("PASS final M1/M3 right M2/M4 left, invert=0, clockwise/counterclockwise, zero/STOP and independent watchdog");
        $finish;
    end
    initial begin #1000000000;$fatal(1,"timeout");end
endmodule
