`timescale 1ns/1ps
module tb_encoder_sync;
    reg clk=0,rst_n=0,enable=0,reverse=0,sample_valid=0;
    reg [9:0] base_duty=400;
    reg [127:0] deltas=0;
    reg [3:0] illegal=0;
    wire [47:0] trims;
    wire active,fault;
    always #5 clk=~clk;
    motor_encoder_sync #(.CLK_HZ(1000)) dut(.*);
    task tick(input integer n);repeat(n)@(negedge clk);endtask
    task sample(input integer a,b,c,d);
        begin
            deltas={32'(d),32'(c),32'(b),32'(a)};sample_valid=1;tick(1);sample_valid=0;tick(49);
        end
    endtask
    task restart;
        begin enable=0;tick(2);enable=1;tick(2);end
    endtask
    integer i,k,rate[0:3],gain[0:3],lo,hi;
    task plant(input bit backward);
        begin
            reverse=backward;restart;
            gain[0]=100;gain[1]=120;gain[2]=105;gain[3]=115;
            for(k=0;k<240;k=k+1)begin
                for(i=0;i<4;i=i+1)begin
                    rate[i]=(400+$signed(trims[i*12+:12]))*gain[i]/1000;
                    if($signed(trims[i*12+:12])>100 || $signed(trims[i*12+:12])< -100)
                        $fatal(1,"trim exceeds bound");
                end
                // Encoder wiring may have mixed signs, independent of motor polarity.
                if(backward)sample(-rate[0],rate[1],-rate[2],rate[3]);
                else sample(rate[0],-rate[1],rate[2],-rate[3]);
            end
            lo=rate[0];hi=rate[0];
            for(i=1;i<4;i=i+1)begin if(rate[i]<lo)lo=rate[i];if(rate[i]>hi)hi=rate[i];end
            if(!active || fault || hi-lo>2)$fatal(1,"plant failed to converge: %0d %0d %0d %0d",rate[0],rate[1],rate[2],rate[3]);
            $display("PASS encoder feedback backward=%0d: initial=(40,48,42,46) settled=(%0d,%0d,%0d,%0d)",backward,rate[0],rate[1],rate[2],rate[3]);
        end
    endtask
    initial begin
        tick(3);rst_n=1;plant(0);plant(1);
        // Changed throttle, direction, disable all discard learned corrections.
        base_duty=410;tick(2);if(trims!=0 || active)$fatal(1,"throttle change retained trim");
        base_duty=400;restart;
        for(k=0;k<24;k=k+1)sample(40,80,40,80);
        if(trims==0 || !active)$fatal(1,"feedback failed to engage");
        reverse=~reverse;tick(2);if(trims!=0 || active)$fatal(1,"reversal retained trim");
        for(k=0;k<24;k=k+1)sample(40,80,40,80);
        enable=0;tick(2);if(trims!=0 || active)$fatal(1,"turn/disable retained trim");
        restart;for(k=0;k<24;k=k+1)sample(40,80,40,80);
        tick(301);if(trims!=0 || active || !fault)$fatal(1,"stale feedback not cleared");
        restart;for(k=0;k<12;k=k+1)sample(40,80,40,80);
        // A disconnected or stalled channel must never be driven to maximum trim.
        for(k=0;k<8;k=k+1)sample(40,80,40,0);
        if(trims!=0 || active || !fault)$fatal(1,"zero channel failed safe fallback");
        for(k=0;k<12;k=k+1)sample(40,80,40,80);
        if(!active || fault)$fatal(1,"valid feedback failed recovery");
        illegal=8;sample(40,80,40,80);
        if(trims!=0 || active || !fault)$fatal(1,"illegal quadrature accepted");
        illegal=0;sample(-2147483648,40,40,40);
        if(trims!=0 || active || !fault)$fatal(1,"abs overflow accepted");
        sample(65536,40,40,40);
        if(trims!=0 || active || !fault)$fatal(1,"implausible count accepted");
        restart;for(k=0;k<12;k=k+1)sample(1,1,1,1);
        if(trims!=0 || active)$fatal(1,"low-resolution feedback not bypassed");
        base_duty=200;restart;for(k=0;k<160;k=k+1)sample(10,100,10,100);
        for(i=0;i<4;i=i+1)if($signed(trims[i*12+:12])>50 || $signed(trims[i*12+:12])< -50)$fatal(1,"relative limit failed");
        $display("PASS limits, deadband, mixed encoder signs, stale/lost/illegal feedback, recovery, throttle/reversal/disable reset");
        $finish;
    end
endmodule
