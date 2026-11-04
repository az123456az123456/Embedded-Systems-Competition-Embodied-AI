`timescale 1ns/1ps
module tb_motor_angle;
    reg clk=0,rst_n=0,valid=0;reg [127:0] data=0;
    wire [3:0] pwm,in1,in2,flags;wire [127:0] status;wire [31:0] sequence_out;
    wire signed [15:0] left,right;
    motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(20),
        .RAMP_STEP_MS(1),.REVERSAL_MS(1),.STATUS_MS(5)) dut(
        .clk(clk),.rst_n(rst_n),.command_valid(valid),.command_data(data),
        .encoder_valid(1'b0),.encoder_deltas(128'b0),.encoder_illegal(4'b0),
        .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),
        .status_valid(),.status_data(status),.last_sequence(sequence_out),
        .applied_left(left),.applied_right(right));
    always #5 clk=~clk;
    integer seqno=0,checks=0,a,f,s,expected_l,expected_r,i;
    task cycles(input integer n); repeat(n)@(negedge clk);endtask
    task command(input integer mode,arm,x,y,mask,inv);
        begin
            seqno=seqno+1;
            @(negedge clk);
            data={mode==2 ? 32'h4d4f5432 : 32'h4d4f5431,mode==2 ? 8'd2 : 8'd1,
                  7'b0,arm[0],seqno[31:0],x[15:0],y[15:0],4'b0,mask[3:0],4'b0,inv[3:0]};
            valid=1;@(negedge clk);valid=0;cycles(3);
        end
    endtask
    task check_target(input integer l,r);
        begin
            checks=checks+1;
            if($signed(dut.target[0])!=l || $signed(dut.target[1])!=r)
                $fatal(1,"target angle=%d expected=(%d,%d) got=(%d,%d)",a,l,r,$signed(dut.target[0]),$signed(dut.target[1]));
            if(status[127:96]!=32'h41434b32 || status[95:88]!=2)$fatal(1,"ACK2 tag");
        end
    endtask
    task reject(input integer x,y,arm,mask,inv);
        integer old;
        begin old=sequence_out;command(2,arm,x,y,mask,inv);checks=checks+1;
            if(sequence_out!=old)$fatal(1,"invalid MOT2 accepted x=%d y=%d arm=%d",x,y,arm);
        end
    endtask
    initial begin
        cycles(3);rst_n=1;
        reject(1000,200,1,10,15); // STOP handshake required
        command(2,0,0,0,10,15);check_target(0,0);
        for(a=-9000;a<=9000;a=a+1)begin
            f=220;s=a/25;
            if(a>=-200 && a<=200)s=0;
            if(s>220)s=220;if(s< -220)s=-220;
            if(a<=-2500 || a>=2500)f=0;
            expected_l=f+s;expected_r=f-s;
            command(2,1,a,220,10,15);check_target(expected_l,expected_r);
        end
        reject(-9001,0,1,10,15);reject(9001,0,1,10,15);
        reject(0,-1,1,10,15);reject(0,221,1,10,15);
        reject(1,0,0,10,15);reject(0,1,0,10,15);
        reject(0,100,1,5,15);reject(0,100,1,10,0);
        reject(0,0,0,0,15);reject(0,0,0,15,15);
        // A mode switch shares sequence freshness: no replayed MOT1 acceptance.
        command(1,1,200,200,10,15);
        if(status[127:96]!=32'h41434b31 || status[95:88]!=1)$fatal(1,"ACK1 restoration");
        seqno=seqno-1;command(2,1,-9000,0,10,15);
        if(status[127:96]!=32'h41434b31 || dut.target[0]!=200)$fatal(1,"cross-mode duplicate accepted");
        for(i=0;i<7;i=i+1)begin command(2,1,0,220,10,15);cycles(400);end
        if(left!=220 || right!=220)$fatal(1,"MOT2 forward ramp");
        command(2,1,0,0,10,15);check_target(0,0);
        if(left!=0 || right!=0 || pwm!=0)$fatal(1,"armed MOT2 zero not immediate");
        for(i=0;i<8;i=i+1)begin command(2,1,9000,0,10,15);cycles(400);end
        if(left!=220 || right!=-220 || in1!=5 || in2!=10)$fatal(1,"CW polarity/ramp");
        command(2,1,-9000,0,10,15);cycles(3);
        if(pwm!=0)$fatal(1,"MOT2 reversal blanking");
        command(2,0,0,0,10,15);
        if(left!=0 || right!=0 || pwm!=0)$fatal(1,"MOT2 STOP not immediate");
        command(2,1,0,220,10,15);cycles(400);
        for(i=0;i<7;i=i+1)begin seqno=seqno-1;command(2,1,0,220,10,15);cycles(400);end
        if(pwm!=0 || !flags[1] || flags[2])$fatal(1,"MOT2 replay watchdog");
        command(2,1,0,100,10,15);if(flags[0])$fatal(1,"MOT2 watchdog rearm bypass");
        command(1,0,0,0,10,15);if(status[127:96]!=32'h41434b31)$fatal(1,"MOT1 recovery STOP");
        $display("PASS MOT2 %0d checked cases; exhaustive signed -90..90 deg, limits, zero, mode freshness, ramp/reversal/watchdog",checks);$finish;
    end
    initial begin #2000000;$fatal(1,"timeout");end
endmodule
