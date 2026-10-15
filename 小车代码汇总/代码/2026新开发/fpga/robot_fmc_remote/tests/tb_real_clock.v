`timescale 1ns/1ps
module tb_real_clock;
    reg clk=0,rst_n=0,valid=0;reg [127:0] data=0;
    wire [3:0] pwm,in1,in2;wire signed [15:0] l,r;
    motor_remote_control #(.RAMP_STEP_MS(1)) dut(.clk(clk),.rst_n(rst_n),
        .command_valid(valid),.command_data(data),.motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),
        .status_flags(),.status_valid(),.status_data(),.last_sequence(),.applied_left(l),.applied_right(r));
    always #10 clk=~clk;
    integer i,j,highs[0:3],rising[0:3];reg [3:0] previous;
    task command;input [31:0] seqno;input bit arm;input [15:0] left,right;
        begin
            @(negedge clk);data={32'h4d4f5431,8'd1,7'd0,arm,seqno,left,right,8'd3,8'd12};valid=1;
            @(negedge clk);valid=0;repeat(5)@(negedge clk);
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst_n=1;
        command(0,0,0,0);command(1,1,600,600);
        #65000000;
        if(l!=600 || r!=600 || in1!=3 || in2!=12)$fatal(1,"real clock ramp/polarity wrong");
        for(i=0;i<4;i=i+1)begin highs[i]=0;rising[i]=-1;end
        previous=pwm;
        for(j=0;j<2500;j=j+1)begin
            @(negedge clk);
            for(i=0;i<4;i=i+1)begin
                highs[i]=highs[i]+pwm[i];
                if(pwm[i]&&!previous[i]&&rising[i]<0)rising[i]=j;
            end
            previous=pwm;
        end
        for(i=0;i<4;i=i+1)if(highs[i]!=1500 || rising[i]<0)$fatal(1,"real PWM must be1500/2500 cycles");
        for(i=0;i<3;i=i+1)
            if((rising[i]-rising[i+1]+2500)%2500!=625)$fatal(1,"carrier spacing must be625 clocks");
        command(2,1,0,0);if(pwm!=0 || l!=0 || r!=0)$fatal(1,"real clock zero did not stop immediately");
        $display("PASS real 50MHz clock, 20kHz carrier,1500/2500 high cycles,625-cycle offsets and PWM=0 stop");$finish;
    end
    initial begin #80000000;$fatal(1,"timeout");end
endmodule
