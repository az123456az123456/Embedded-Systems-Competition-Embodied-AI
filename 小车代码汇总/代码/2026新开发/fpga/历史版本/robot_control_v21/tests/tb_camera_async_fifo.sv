`timescale 1ns/1ps
module tb_camera_async_fifo #(parameter AW=6);
    localparam DEPTH=1<<AW;
    reg wrclk=0,rdclk=0,aclr=1,wrreq=0,rdreq=0;
    reg [7:0] wrdata=0;
    wire [7:0] rddata;
    wire [AW-1:0] rdusedw,wrusedw;
    wire empty,full;
    reg [7:0] reference[0:200000];
    integer pushed=0,popped=0,total_push=0,total_pop=0,phase=0,i;
    integer rejected_write=0,rejected_read=0;
    reg [31:0] ws=32'h44316d32,rs=32'h99853acf;
    real whalf=4.1,rhalf=6.3;
    always #(whalf) wrclk=~wrclk;
    always #(rhalf) rdclk=~rdclk;
    camera_async_fifo #(.ADDR_WIDTH(AW)) dut(.*);
    always @(posedge wrclk) if(!aclr && dut.wr_reset_n)begin
        if(wrreq && !full)begin
            reference[pushed]=wrdata;pushed=pushed+1;total_push=total_push+1;
            if(pushed-popped>DEPTH)$fatal(1,"overflow");
        end else if(wrreq && full) rejected_write=rejected_write+1;
    end
    always @(posedge rdclk) if(!aclr && dut.rd_reset_n)begin
        if(dut.rbin_plus_one!==((dut.rbin+1)&(2*DEPTH-1)))
            $fatal(1,"prefetch address invariant broken");
        if(dut.read_count_next>DEPTH)
            $fatal(1,"registered write decode lag caused count underflow");
        if(!empty && dut.decoded_write_bin==dut.rbin)
            $fatal(1,"empty flag outran registered decoded writer pointer");
        if(rdreq && !empty)begin
            if(popped>=pushed)$fatal(1,"underflow");
            if(rddata!==reference[popped])$fatal(1,"FWFT byte mismatch %0d got %02h expected %02h phase %0d",popped,rddata,reference[popped],phase);
            popped=popped+1;total_pop=total_pop+1;
        end else if(rdreq && empty) rejected_read=rejected_read+1;
        if(rdusedw>pushed-popped+1)$fatal(1,"occupancy overestimates live data");
    end
    always @(negedge wrclk)begin
        ws={ws[30:0],ws[31]^ws[21]^ws[1]^ws[0]};
        wrdata=ws[15:8];
        case(phase)
        1:wrreq=1;
        3,4:wrreq=ws[0]||ws[2];
        default:wrreq=0;
        endcase
    end
    always @(negedge rdclk)begin
        rs={rs[30:0],rs[31]^rs[21]^rs[1]^rs[0]};
        case(phase)
        2,5:rdreq=1;
        3,4:rdreq=rs[0]||rs[3];
        default:rdreq=0;
        endcase
    end
    task reset_fifo;
    begin
        aclr=1;#137;pushed=0;popped=0;aclr=0;
        repeat(12)@(posedge rdclk);
        if(!empty || full || rdusedw!=0)$fatal(1,"reset state invalid");
    end endtask
    initial begin
        reset_fifo();
        phase=1;repeat(DEPTH*3)@(posedge wrclk);#1;
        if(!full || pushed!=DEPTH || wrusedw!=DEPTH-1)$fatal(1,"full protection/count failed %0d",pushed);
        phase=0;repeat(10)@(posedge rdclk);
        if(rdusedw!=DEPTH-1)$fatal(1,"full count saturation incorrect %0d",rdusedw);
        phase=2;repeat(DEPTH*3)@(posedge rdclk);#1;
        if(!empty || popped!=DEPTH)$fatal(1,"empty protection failed");
        phase=3;repeat(40000)@(posedge wrclk);
        // Swap which clock is faster, preserving unrelated edges.
        whalf=7.7;rhalf=3.9;phase=4;
        repeat(30000)@(posedge rdclk);
        phase=0;repeat(10)@(posedge rdclk);
        reset_fifo(); // Throw away unread data on a common asynchronous reset.
        phase=4;repeat(20000)@(posedge rdclk);
        phase=0;repeat(10)@(posedge rdclk);phase=5;
        repeat(DEPTH*3)@(posedge rdclk);
        if(pushed!=popped)$fatal(1,"final drain mismatch");
        if(total_pop<20000 || rejected_write<10 || rejected_read<10)$fatal(1,"insufficient coverage");
        $display("PASS camera async FIFO: %0d pushes %0d pops, full/empty protection, pointer wrap, both clock ratios, reset, FWFT",total_push,total_pop);
        $finish;
    end
    initial begin #3000000;$fatal(1,"timeout");end
endmodule
