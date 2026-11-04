`timescale 1ns/1ps
// Independent four-state check of the finite sender input stages. The source
// words remain stable until popped; availability, reset and epoch vary.
module tb_rgb_head_cache;
    reg clk=0;always #4 clk=~clk;
    reg rst_n=0;reg [15:0] epoch=1;
    reg tx_allow=1;
    wire tx_busy,tx_pending,gmii_valid,pair_fault;
    wire [7:0] gmii_data;
    wire [31:0] pair_fault_count,packets_sent,pairs_started;
    reg [35:0] fifo0_data=0,fifo1_data=0;
    reg fifo0_empty=1,fifo1_empty=1;
    wire fifo0_pop,fifo1_pop;
    rgb_packet_tx #(.IMAGE_HEIGHT(4),.WAIT_CYCLES(1000)) dut(.*);
    reg [35:0] expected0=0,expected1=0;
    reg valid0=0,valid1=0,old_pop0,old_pop1,old_take0,old_take1,old_epoch_clear;
    reg refill_pending0=0,refill_pending1=0;
    reg [31:0] random_bits=32'h12e3f491;
    integer cycle=0,word0=0,word1=0,checks=0,refills=0,holds=0,invalidations=0,resets=0;
    reg [15:0] tag_coverage=0;
    function [4:0] classes;
        input [3:0] tag;
        begin classes={tag==1,tag==2,tag==0,tag==3,tag==15};end
    endfunction
    always @(posedge clk)begin
        old_pop0=fifo0_pop;old_pop1=fifo1_pop;
        old_take0=dut.head0_take;old_take1=dut.head1_take;
        old_epoch_clear=(epoch!=dut.current_epoch && !dut.sending);
        if(rst_n)begin
            if(old_pop0 && fifo0_empty || old_pop1 && fifo1_empty)$fatal(1,"pop on empty source");
            if(old_take0 && !dut.head0_valid || old_take1 && !dut.head1_valid)$fatal(1,"take invalid cache");
            if(old_pop0 && old_take0 || old_pop1 && old_take1)$fatal(1,"late consume-to-refill combinational path returned");
            if(old_pop0 && refill_pending0)begin refills=refills+1;refill_pending0=0;end
            if(old_pop1 && refill_pending1)begin refills=refills+1;refill_pending1=0;end
            if(old_take0)refill_pending0=1;
            if(old_take1)refill_pending1=1;
            if(valid0 && !old_take0 || valid1 && !old_take1)holds=holds+1;
            if(old_pop0)begin expected0=fifo0_data;valid0=1;end else if(old_take0)valid0=0;
            if(old_pop1)begin expected1=fifo1_data;valid1=1;end else if(old_take1)valid1=0;
            if(old_epoch_clear)begin valid0=0;valid1=0;invalidations=invalidations+1;end
        end else begin valid0=0;valid1=0;refill_pending0=0;refill_pending1=0;resets=resets+1;end
        #1;
        if(dut.head0_valid!==valid0 || dut.head1_valid!==valid1)$fatal(1,"cache occupancy mismatch at %0d",cycle);
        if(valid0)begin
            if(dut.head0_data!==expected0)$fatal(1,"head0 duplicate/lost word");
            if({dut.head0_sof,dut.head0_stamp,dut.head0_pixel,dut.head0_eof,dut.head0_abort}!==classes(expected0[35:32]))$fatal(1,"head0 classification/stale tag");
            tag_coverage[expected0[35:32]]=1;
        end
        if(valid1)begin
            if(dut.head1_data!==expected1)$fatal(1,"head1 duplicate/lost word");
            if({dut.head1_sof,dut.head1_stamp,dut.head1_pixel,dut.head1_eof,dut.head1_abort}!==classes(expected1[35:32]))$fatal(1,"head1 classification/stale tag");
            tag_coverage[expected1[35:32]]=1;
        end
        checks=checks+1;
    end
    always @(negedge clk)begin
        cycle=cycle+1;
        random_bits={random_bits[30:0],random_bits[31]^random_bits[21]^random_bits[1]^random_bits[0]};
        rst_n=(cycle>5 && cycle%2357!=0 && cycle%2357!=1);
        if(cycle%5003==1000)epoch=epoch+1'b1;
        tx_allow=random_bits[7];
        if(old_pop0)begin word0=word0+1;fifo0_data={word0[3:0],word0[15:0],16'h1357};end
        if(old_pop1)begin word1=word1+1;fifo1_data={word1[3:0],word1[15:0],16'h2468};end
        fifo0_empty=!random_bits[0] || !random_bits[2];
        fifo1_empty=!random_bits[1] || !random_bits[3];
        if(cycle==50000)begin
            if(tag_coverage!==16'hffff || refills<1000 || holds<1000 || invalidations<5 || resets<5)$fatal(1,"insufficient boundary coverage tags=%h refill=%0d hold=%0d epochs=%0d reset=%0d",tag_coverage,refills,holds,invalidations,resets);
            $display("PASS checks=%0d refills=%0d holds=%0d epoch_invalidations=%0d reset_edges=%0d tag_coverage=%h",checks,refills,holds,invalidations,resets,tag_coverage);
            $finish;
        end
    end
endmodule
