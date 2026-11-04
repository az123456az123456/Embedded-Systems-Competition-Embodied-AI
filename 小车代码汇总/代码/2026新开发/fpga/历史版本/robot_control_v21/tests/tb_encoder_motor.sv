`timescale 1ns/1ps
// Whole feedback path: actual PWM pins -> unequal wheel plants -> quadrature
// pins -> existing encoder filter/SNS1 capture -> motor feedback -> PWM pins.
module tb_encoder_motor;
    reg clk=0,rst_n=0,command_valid=0;
    reg [127:0] command_data=0;
    reg [3:0] encoder_a=0,encoder_b=0,blocked=0;
    wire sensor_sample_valid;
    wire [511:0] sensor_sample_packet;
    wire [3:0] pwm,in1,in2,flags;
    wire signed [15:0] left,right;
    wire [127:0] status;
    always #5000 clk=~clk;
    robot_sensor_capture #(.CLK_FREQ(100000),.BAUD(1000),.ENCODER_FILTER_CYCLES(2)) capture(
        .clk(clk),.rst_n(rst_n),.encoder_a(encoder_a),.encoder_b(encoder_b),.sonar_rx(3'b111),
        .sensor_valid(sensor_sample_valid),.sensor_packet(sensor_sample_packet));
    motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.RAMP_STEP_MS(1),.ENCODER_SYNC_ENABLE(1)) dut(
        .clk(clk),.rst_n(rst_n),.command_valid(command_valid),.command_data(command_data),
        .encoder_valid(sensor_sample_valid && sensor_sample_packet[31:16]==16'd50),
        .encoder_deltas({sensor_sample_packet[159:128],sensor_sample_packet[191:160],
                         sensor_sample_packet[223:192],sensor_sample_packet[255:224]}),
        .encoder_illegal(sensor_sample_packet[11:8]),
        .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),.status_valid(),
        .status_data(status),.last_sequence(),.applied_left(left),.applied_right(right));
    integer gain[0:3],phase[0:3],fraction[0:3],i;
    reg [1:0] ab;
    always @(posedge clk)begin
        if(!rst_n)begin
            encoder_a<=0;encoder_b<=0;
            for(i=0;i<4;i=i+1)begin phase[i]=0;fraction[i]=0;end
        end else for(i=0;i<4;i=i+1)if(pwm[i] && !blocked[i])begin
            fraction[i]=fraction[i]+gain[i];
            if(fraction[i]>=5000)begin
                fraction[i]=fraction[i]-5000;
                if(in2[i] ^ (i==1 || i==3))phase[i]=(phase[i]+1)%4;
                else phase[i]=(phase[i]+3)%4;
                case(phase[i])0:ab=0;1:ab=2;2:ab=3;3:ab=1;endcase
                encoder_a[i]<=ab[1];encoder_b[i]<=ab[0];
            end
        end
    end
    task tick(input integer n);repeat(n)@(negedge clk);endtask
    integer seqno=0,k,j,count[0:3],rate[0:3],lo,hi;
    task command(input bit armed,input integer l,r);
        begin
            seqno=seqno+1;command_data={32'h4d4f5431,8'd1,7'd0,armed,32'(seqno),16'(l),16'(r),8'h0a,8'h0f};
            command_valid=1;tick(1);command_valid=0;tick(2);
        end
    endtask
    task run_motion(input integer l,r,windows);
        begin for(k=0;k<windows;k=k+1)begin command(1,l,r);tick(4997);end end
    endtask
    task check_balance;
        begin
            for(j=0;j<4;j=j+1)begin
                rate[j]=$signed(sensor_sample_packet[255-32*j-:32]);
                if(rate[j]<0)rate[j]=-rate[j];
            end
            lo=rate[0];hi=rate[0];
            for(j=1;j<4;j=j+1)begin if(rate[j]<lo)lo=rate[j];if(rate[j]>hi)hi=rate[j];end
            if(!dut.sync_active || dut.sync_fault || hi-lo>3 || lo<30)
                $fatal(1,"feedback path failed balance: %0d %0d %0d %0d active=%0d",rate[0],rate[1],rate[2],rate[3],dut.sync_active);
            if(dut.threshold[1]>=dut.threshold[0] || dut.threshold[3]>=dut.threshold[2])
                $fatal(1,"fast left wheels were not reduced");
            $display("PASS actual PWM -> encoder A/B -> SNS1 -> correction, left=%0d: delta_abs=(%0d,%0d,%0d,%0d)",left,rate[0],rate[1],rate[2],rate[3]);
        end
    endtask
    initial begin
        gain[0]=100;gain[1]=120;gain[2]=105;gain[3]=115;
        tick(4);rst_n=1;tick(4);
        if(status[15:0]!=16'h0a0f || pwm!=0)$fatal(1,"v15 defaults/startup incorrect");
        command(1,400,400);tick(200);if(pwm!=0)$fatal(1,"motion without STOP handshake");
        command(0,0,0);run_motion(400,400,160);check_balance;
        if(in1!=0 || in2!=15)$fatal(1,"confirmed forward polarity changed");
        command(1,-400,-400);tick(5);if(pwm!=0)$fatal(1,"reversal blanking failed");
        run_motion(-400,-400,160);check_balance;
        if(in1!=15 || in2!=0)$fatal(1,"reverse polarity wrong");
        blocked=8;run_motion(-400,-400,10);
        if(dut.sync_active || !dut.sync_fault || dut.wheel_trims!=0 || left!=-400)$fatal(1,"missing M4 failed fallback");
        blocked=0;run_motion(-400,-400,16);if(!dut.sync_active)$fatal(1,"recovery failed");
        run_motion(400,200,5);
        if(dut.sync_active || dut.wheel_trims!=0)$fatal(1,"feedback fights intended turn");
        for(j=0;j<4;j=j+1)count[j]=0;
        repeat(100)begin tick(1);for(j=0;j<4;j=j+1)count[j]=count[j]+pwm[j];end
        if(count[0]!=20 || count[1]!=40 || count[2]!=20 || count[3]!=40)$fatal(1,"arc mapping/duty changed");
        run_motion(300,-300,5);if(in1!=5 || in2!=10)$fatal(1,"CW mapping wrong");
        run_motion(-300,300,5);if(in1!=10 || in2!=5)$fatal(1,"CCW mapping wrong");
        command(1,0,0);tick(3);if({pwm,in1,in2}!=0)$fatal(1,"zero must stop");
        run_motion(400,400,16);if(!dut.sync_active)$fatal(1,"feedback did not re-engage");
        tick(50100);if({pwm,in1,in2}!=0 || !flags[1] || dut.wheel_trims!=0)$fatal(1,"watchdog must stop even with valid feedback");
        command(0,0,0);run_motion(400,400,16);command(0,0,0);tick(3);
        if({pwm,in1,in2}!=0 || dut.wheel_trims!=0)$fatal(1,"disarm failed");
        $display("PASS real encoder acquisition, M4 loss/recovery, arc/rotation bypass, confirmed mask10/invert15, startup/reversal/zero/watchdog/disarm");
        $finish;
    end
    initial begin #25000000000;$fatal(1,"integration timeout");end
endmodule
