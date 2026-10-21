`timescale 1ns/1ps
module tb_motor_remote;
    reg clk=0,rst_n=0,command_valid=0;reg [127:0] command_data=0;
    wire [3:0] pwm,in1,in2,flags;wire status_valid;wire [127:0] status;
    wire [31:0] last_sequence;wire signed[15:0] left,right;
    motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(20),
        .RAMP_STEP_MS(1),.REVERSAL_MS(1),.STATUS_MS(5)) dut(
        .clk(clk),.rst_n(rst_n),.command_valid(command_valid),.command_data(command_data),
        .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),
        .status_valid(status_valid),.status_data(status),.last_sequence(last_sequence),
        .applied_left(left),.applied_right(right));
    always #5000 clk=~clk;
    reg [31:0] seqno=100;integer i,j,highs[0:3];
    task command;
        input bit arm;input signed [15:0] l,r;input [3:0] mask,inv;
        begin
            @(negedge clk);command_data={32'h4d4f5431,8'd1,7'd0,arm,seqno,l,r,4'd0,mask,4'd0,inv};command_valid=1;
            @(negedge clk);command_valid=0;repeat(3)@(negedge clk);
        end
    endtask
    task fresh;
        input bit arm;input signed[15:0] l,r;
        begin seqno=seqno+1;command(arm,l,r,3,12);end
    endtask
    task cycles;input integer count;begin repeat(count)@(negedge clk);end endtask
    initial begin
        cycles(3);rst_n=1;fresh(1,600,600);cycles(300);
        if(pwm!=0 || flags[2] || flags[0])$fatal(1,"armed packet bypassed first STOP handshake");
        fresh(0,0,0);if(flags!==4'b0100)$fatal(1,"STOP configuration handshake failed");
        for(i=0;i<16;i=i+1)begin fresh(1,1000,700);cycles(400);end
        if(left!=600 || right!=600 || in1!=3 || in2!=12)$fatal(1,"clamp/ramp/polarity wrong");
        for(i=0;i<4;i=i+1)highs[i]=0;
        for(j=0;j<100;j=j+1)begin cycles(1);for(i=0;i<4;i=i+1)highs[i]=highs[i]+pwm[i];end
        for(i=0;i<4;i=i+1)if(highs[i]!=60)$fatal(1,"wrong steady PWM duty");
        // Runtime remapping cannot be smuggled into an armed command.
        seqno=seqno+1;command(1,400,400,5,0);
        if(last_sequence==seqno)$fatal(1,"armed remap accepted");
        fresh(1,-300,300);cycles(3);
        if(pwm[1:0]!=0)$fatal(1,"reversal was not blanked");
        cycles(90);if(dut.direction[0]!=0)$fatal(1,"direction changed before 1ms deadtime");
        cycles(30);if(dut.direction[0]!=1)$fatal(1,"direction did not change after deadtime");
        for(i=0;i<8;i=i+1)begin fresh(1,-300,300);cycles(400);end
        if(left!=-300 || right!=300 || in1!=0 || in2!=15)$fatal(1,"signed reversal ramp wrong");
        fresh(1,0,300);if(pwm[1:0]!=0 || left!=0)$fatal(1,"zero side must stop immediately");
        fresh(0,0,0);if(pwm!=0 || left!=0 || right!=0)$fatal(1,"STOP must be immediate");
        fresh(1,400,400);cycles(400);
        // Duplicate packets do not refresh a stale motion command.
        for(i=0;i<7;i=i+1)begin command(1,400,400,3,12);cycles(400);end
        if(pwm!=0 || !flags[1] || flags[2])$fatal(1,"watchdog failed on duplicate-only input");
        seqno=1;command(1,400,400,3,12);if(flags[0])$fatal(1,"armed restart bypassed recovery STOP");
        command(0,0,0,3,12);if(!flags[2] || flags[1])$fatal(1,"new host STOP cannot recover sequence");
        seqno=32'hfffffffe;command(0,0,0,3,12); // old sequence: ignored
        if(last_sequence!=1)$fatal(1,"old sequence accepted");
        // Explicit restart after another timeout permits a sequence close to wrap.
        cycles(2100);seqno=32'hfffffffe;command(0,0,0,3,12);
        seqno=32'hffffffff;command(1,300,300,3,12);
        seqno=0;command(1,300,300,3,12);
        if(last_sequence!=0 || !flags[0])$fatal(1,"32-bit sequence wrap rejected");
        $display("PASS startup STOP handshake, signed PWM clamp/ramp, inversion, remap rejection, deadtime, immediate zero, freshness watchdog and sequence recovery/wrap");$finish;
    end
    initial begin #300000000;$fatal(1,"timeout");end
endmodule
