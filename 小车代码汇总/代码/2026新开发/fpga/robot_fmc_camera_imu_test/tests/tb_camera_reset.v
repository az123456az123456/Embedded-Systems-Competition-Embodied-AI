`timescale 1ns/1ps
module tb_camera_reset;
    reg a=0,b=0,run_b=1,source_n=0,fifo1_reset=1,fifo2_reset=1;
    wire reset_a,reset_b;wire rdclk,rd1,rd2,wrclk,aclr,wrreq;wire [7:0] data;wire fault;
    camera_reset_release sa(.clk(a),.async_reset_n(source_n),.reset_n(reset_a));
    camera_reset_release sb(.clk(b),.async_reset_n(source_n),.reset_n(reset_b));
    controller_top controller(.pair_fault(fault),.clk(a),.rst_n(reset_a),
        .wrclk1(a),.fifo1_aclr(fifo1_reset),.wrdata1(8'b0),.wrreq1(1'b0),
        .wrclk2(b),.fifo2_aclr(fifo2_reset),.wrdata2(8'b0),.wrreq2(1'b0),
        .wrclk(wrclk),.fifo_aclr(aclr),.fifo_wrreq(wrreq),.fifo_wrdata(data));
    always #4 a=~a;
    always #7 if(run_b)b=~b;
    initial begin
        #3;source_n=1;
        @(posedge a);#1;if(reset_a)$fatal(1,"reset A released on first edge");
        @(posedge a);#1;if(!reset_a)$fatal(1,"reset A not released on second edge");
        #1;fifo1_reset=0;fifo2_reset=0;
        @(posedge a);#1;if(!aclr)$fatal(1,"controller released on first edge");
        @(posedge a);#1;if(aclr)$fatal(1,"controller not released on second local edge");
        #2;fifo2_reset=1;#1;if(!aclr)$fatal(1,"controller did not asynchronously assert");
        #2;fifo2_reset=0;
        @(posedge a);#1;if(!aclr)$fatal(1,"controller re-released too early");
        @(posedge a);#1;if(aclr)$fatal(1,"controller re-release failed");
        run_b=0;#1;source_n=0;#1;
        if(reset_a || reset_b || !aclr)$fatal(1,"async reset must assert even with stopped PCLK");
        source_n=1;repeat(5)@(posedge a);#1;
        if(reset_b)$fatal(1,"stopped PCLK must hold its reset");
        run_b=1;repeat(3)@(posedge b);#1;if(!reset_b)$fatal(1,"PCLK reset did not recover");
        $display("PASS camera local reset release, combined FIFO reset synchronization and stopped PCLK assertion");$finish;
    end
endmodule
// FIFO stub is used only for reset/controller boundary testing, not image data.
module cache(input wrclk,aclr,wrreq,input [7:0]wrdata,input rdclk,rdreq,
    output [7:0]rddata,output [13:0]rdusedw);
    assign rddata=0;assign rdusedw=0;
endmodule
