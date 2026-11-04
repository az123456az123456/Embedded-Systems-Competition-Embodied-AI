`timescale 1ns/1ps
module tb_direction_sequence;
    reg clk=0,rst_n=0;
    always #5000 clk=~clk; // 100kHz scaled clock; same millisecond/state logic.
    wire [3:0] pwm,in1,in2,led,step,event_id;
    wire event_valid,done,tx,overflow;
    motor_direction_sequence #(.CLK_HZ(100000),.PWM_HZ(1000),.WAIT_MS(40),
        .PREP_MS(1),.HOLD_MS(10),.STOP_MS(3),.RAMP_STEP_MS(1)) dut(
        .clk(clk),.rst_n(rst_n),.motor_pwm(pwm),.motor_in1(in1),.motor_in2(in2),
        .led(led),.event_valid(event_valid),.event_id(event_id),.step_number(step),.done(done));
    diag_uart_logger #(.CLK_HZ(100000),.BAUD(50000)) logger(
        .clk(clk),.rst_n(rst_n),.event_valid(event_valid),.event_id(event_id),.tx(tx),.overflow(overflow));
    // Baseline mask: assert that only rear direction pins change, never PWM/timing.
    wire [3:0] old_pwm,old_in1,old_in2,old_led,old_step,old_event_id;
    wire old_event_valid,old_done;
    motor_direction_sequence #(.CLK_HZ(100000),.PWM_HZ(1000),.WAIT_MS(40),
        .PREP_MS(1),.HOLD_MS(10),.STOP_MS(3),.RAMP_STEP_MS(1),.INVERT_MASK(4'b1001)) baseline(
        .clk(clk),.rst_n(rst_n),.motor_pwm(old_pwm),.motor_in1(old_in1),.motor_in2(old_in2),
        .led(old_led),.event_valid(old_event_valid),.event_id(old_event_id),.step_number(old_step),.done(old_done));
    integer events=0,cycles=0,previous_step=0,gap_cycles=0;
    reg [2:0] previous_state=0;
    always @(negedge clk) if(rst_n) begin
        cycles=cycles+1;
        if({pwm,led,step,event_valid,event_id,done}!=={old_pwm,old_led,old_step,old_event_valid,old_event_id,old_done})
            $fatal(1,"v2 changed PWM/timing/event behavior");
        if((in1^old_in1)!==((in1|in2)&4'b0110) || (in2^old_in2)!==((in1|in2)&4'b0110))
            $fatal(1,"v2 direction delta is not limited to M2/M3");
        if((in1&in2)!=0 || (^{pwm,in1,in2})===1'bx) $fatal(1,"Unknown or conflicting outputs");
        if(!dut.running && {pwm,in1,in2}!=0) $fatal(1,"Outputs active in stopped state");
        if(dut.state==5) gap_cycles=gap_cycles+1;
        if(previous_state==5 && dut.state!=5) begin
            if(gap_cycles!=300) $fatal(1,"Stop gap too short: %0d",gap_cycles);
            gap_cycles=0;
        end
        if(event_valid) begin
            events=events+1;
            if(event_id!=events) $fatal(1,"Wrong event order %0d expected%0d",event_id,events);
            if(events<=12 && step!=events) $fatal(1,"Wrong stage number");
        end
        if(step!=0 && led!=step) $fatal(1,"Wrong LED index");
        if(overflow) $fatal(1,"UART message lost");
        previous_state=dut.state;
        previous_step=step;
    end
    integer uart_file,uart_bytes=0,bit_index;
    reg [7:0] byte_value;
    initial begin
        uart_file=$fopen("reports/uart_capture.hex","w");
        wait(rst_n);
        forever begin
            @(negedge tx);
            #30000; // 1.5 bits at DIV=2 clocks.
            for(bit_index=0;bit_index<8;bit_index=bit_index+1) begin
                byte_value[bit_index]=tx;
                #20000;
            end
            if(tx!==1) $fatal(1,"UART invalid stop bit");
            $fwrite(uart_file,"%02x\n",byte_value);uart_bytes=uart_bytes+1;
        end
    end
    integer s,n,c,expected_duty[0:3],count[0:3];
    reg [3:0] expected_in1,expected_in2;
    initial begin
        repeat(4)@(negedge clk);rst_n=1;
        for(s=1;s<=12;s=s+1) begin
            wait(step==s && dut.state==3);
            // Skip two complete carrier periods; then count four whole periods.
            repeat(200)@(negedge clk);
            for(n=0;n<4;n=n+1)begin expected_duty[n]=0;count[n]=0;end
            expected_in1=0;expected_in2=0;
            case(s)
             1:begin expected_duty[0]=40;expected_in2=1;end
             2:begin expected_duty[1]=40;expected_in2=2;end
             3:begin expected_duty[2]=40;expected_in2=4;end
             4:begin expected_duty[3]=40;expected_in2=8;end
             5,6,7,8:begin
                 for(n=0;n<4;n=n+1)expected_duty[n]=40;
                 case(s)
                   5:begin expected_in1=0;expected_in2=15;end
                   6:begin expected_in1=15;expected_in2=0;end
                   7:begin expected_in1=10;expected_in2=5;end
                   8:begin expected_in1=5;expected_in2=10;end
                 endcase
             end
             9,11:begin expected_duty[0]=40;expected_duty[1]=20;expected_duty[2]=40;expected_duty[3]=20;
                 expected_in1=(s==9 ? 0 : 15);expected_in2=(s==9 ? 15 : 0);end
             10,12:begin expected_duty[0]=20;expected_duty[1]=40;expected_duty[2]=20;expected_duty[3]=40;
                 expected_in1=(s==10 ? 0 : 15);expected_in2=(s==10 ? 15 : 0);end
            endcase
            for(c=0;c<400;c=c+1)begin
                @(negedge clk);
                for(n=0;n<4;n=n+1)count[n]=count[n]+pwm[n];
                if(in1!=expected_in1 || in2!=expected_in2) $fatal(1,"Wrong direction stage%0d in=%h/%h",s,in1,in2);
            end
            for(n=0;n<4;n=n+1)
                if(count[n]!=expected_duty[n]*4) $fatal(1,"Stage%0d M%0d duty count%0d expected%0d",s,n+1,count[n],expected_duty[n]*4);
            $display("PASS step%02d PWM=(%0d,%0d,%0d,%0d) IN1=%h IN2=%h",s,count[0],count[1],count[2],count[3],in1,in2);
            wait(dut.state!=3);
        end
        wait(done);wait(logger.sending && logger.message==13);wait(!logger.sending);
        repeat(1000)@(negedge clk);
        if(events!=13 || uart_bytes!=14*128 || led!=15 || {pwm,in1,in2}!=0) $fatal(1,"Incomplete sequence or final stop");
        $fclose(uart_file);
        $display("PASS one-shot final stop, exact stop gaps, 13 ordered events, 14 UART messages");
        $display("PASS v1/v2 equivalence: same PWM, timing, LEDs/events; ONLY M2/M3 direction pins flipped");
        $finish;
    end
    initial begin #1000000000;$fatal(1,"Test timeout");end
endmodule
