`timescale 1ns/1ps
module tb_encoder_prearm_relocation;
    reg clk=0,rst_n=0,manual_armed=0,nav_owned=0;
    wire motor_armed=manual_armed||nav_owned;
    reg [3:0] encoder_a=0,encoder_b=0;
    wire [127:0] positions;
    wire [3:0] illegal;
    always #10 clk=~clk;
    encoder_4ch #(.FILTER_CYCLES(2)) dut(
        .clk(clk),.rst_n(rst_n),.motor_armed(motor_armed),
        .encoder_a(encoder_a),.encoder_b(encoder_b),
        .positions(positions),.illegal_seen(illegal));
    task settle(input bit a,input bit b);
        begin
            encoder_a[0]=a;encoder_b[0]=b;
            repeat(8)@(posedge clk);
            #1;
        end
    endtask
    initial begin
        repeat(3)@(posedge clk);rst_n=1;
        settle(0,0);
        // Illegal Gray transition while disarmed must not latch.
        settle(1,1);
        if(illegal[0])$fatal(1,"pre-arm illegal transition latched");
        manual_armed=1;
        repeat(3)@(posedge clk);#1;
        if(illegal[0])$fatal(1,"old pre-arm transition appeared after arm");
        // A new illegal transition while armed is still observable.
        settle(0,0);
        if(!illegal[0])$fatal(1,"armed illegal transition was hidden");
        manual_armed=0;
        repeat(3)@(posedge clk);#1;
        if(illegal[0])$fatal(1,"fault did not clear after disarm");
        manual_armed=1;
        repeat(3)@(posedge clk);#1;
        if(illegal[0])$fatal(1,"stale illegal bit blocked next arm");
        manual_armed=0;
        repeat(3)@(posedge clk);#1;
        nav_owned=1;
        settle(1,1);
        if(!illegal[0])$fatal(1,"navigation-owned illegal transition was hidden");
        nav_owned=0;
        repeat(3)@(posedge clk);#1;
        if(illegal[0])$fatal(1,"navigation fault did not clear after release");
        $display("PASS prearm relocation: idle move ignored, manual/navigation faults seen, release clears, rearm clean");
        $finish;
    end
endmodule
