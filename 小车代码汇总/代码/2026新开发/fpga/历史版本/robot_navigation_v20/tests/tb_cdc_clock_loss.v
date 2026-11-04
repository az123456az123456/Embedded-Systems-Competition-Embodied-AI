`timescale 1ns/1ps
module tb_cdc_clock_loss;
    reg src_clk=0,dst_clk=0,run_rx_clock=1,rst_n=0,src_valid=0;
    reg [127:0] src_data=0;wire ready,dst_valid;wire [127:0] dst_data;
    wire [3:0] pwm,in1,in2,flags;wire signed[15:0] l,r;
    always #4000 if(run_rx_clock)src_clk=~src_clk;
    always #5000 dst_clk=~dst_clk;
    motor_command_cdc cdc(.src_clk(src_clk),.src_rst_n(rst_n),.src_valid(src_valid),
        .src_data(src_data),.src_ready(ready),.dst_clk(dst_clk),.dst_rst_n(rst_n),
        .dst_valid(dst_valid),.dst_data(dst_data));
    motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000),.WATCHDOG_MS(20),.RAMP_STEP_MS(1)) motors(
        .clk(dst_clk),.rst_n(rst_n),.command_valid(dst_valid),.command_data(dst_data),
        .motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),.status_flags(flags),
        .status_valid(),.status_data(),.last_sequence(),.applied_left(l),.applied_right(r));
    task send;input [31:0] seqno;input bit arm;
        begin
            wait(ready);@(negedge src_clk);
            src_data={32'h4d4f5431,8'd1,7'd0,arm,seqno,arm?16'd200:16'd0,arm?16'd200:16'd0,8'd3,8'd12};
            src_valid=1;@(negedge src_clk);src_valid=0;
            // Input changes before the destination has sampled: held data must stay coherent.
            src_data=128'hdeaddeaddeaddeaddeaddeaddeaddead;
            wait(!ready);wait(ready);repeat(5)@(negedge dst_clk);
        end
    endtask
    initial begin
        repeat(4)@(negedge dst_clk);rst_n=1;
        send(1,0);if(!flags[2] || flags[0])$fatal(1,"CDC STOP payload corrupt");
        send(2,1);repeat(600)@(negedge dst_clk);
        if(l==0 || r==0 || !flags[0])$fatal(1,"CDC armed command failed");
        run_rx_clock=0;repeat(2200)@(negedge dst_clk);
        if(pwm!=0 || !flags[1] || flags[2])$fatal(1,"RX clock loss prevented watchdog stop");
        run_rx_clock=1;send(1,0);send(2,1);
        if(!flags[0] || flags[1])$fatal(1,"restart STOP handshake after RX recovery failed");
        $display("PASS coherent command CDC, stopped RX clock watchdog and recovered host sequence");$finish;
    end
    initial begin #100000000;$fatal(1,"timeout");end
endmodule
