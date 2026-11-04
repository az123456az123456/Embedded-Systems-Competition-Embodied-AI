`timescale 1ns/1ps
module tb_rgb_packet_tx;
    parameter HEIGHT=4,WORDS0=2563,WORDS1=2563,MAX_WORDS=100000;
    parameter FRAME_CYCLES=0,PIXEL_GAP=0,CHANGE_EPOCH=0,MAX_CYCLES=1000000,INITIAL_EPOCH=1;
    reg clk=0;always #4 clk=~clk;
    reg rst_n=0;reg [15:0] epoch=INITIAL_EPOCH[15:0];
    wire tx_busy,tx_pending,gmii_valid,pair_fault;
    wire [7:0] gmii_data;wire [31:0] pair_fault_count,packets_sent,pairs_started;
    reg [35:0] words0[0:MAX_WORDS-1],words1[0:MAX_WORDS-1];
    integer index0=0,index1=0,cycle=0,quiet=0,capture,packet_bytes=0,last_end=-10000;
    integer allow_cycles=0,blocked_cycles=0,ifg_min=100000,first_start=-1;
    reg previous_valid=0;reg changed=0;
    reg [1023:0] input0,input1,output_path;
    wire fifo0_pop,fifo1_pop;
    wire [35:0] fifo0_data=words0[index0],fifo1_data=words1[index1];
    function integer release_cycle;
        input integer index;
        integer per_frame,frame,offset;
        begin
            per_frame=640*HEIGHT+3;frame=index/per_frame;offset=index%per_frame;
            if(FRAME_CYCLES==0)release_cycle=0;
            else if(offset<2)release_cycle=100+frame*FRAME_CYCLES+offset;
            else release_cycle=200+frame*FRAME_CYCLES+(offset-2)*PIXEL_GAP;
        end
    endfunction
    wire fifo0_empty=index0>=WORDS0 || cycle<release_cycle(index0);
    wire fifo1_empty=index1>=WORDS1 || cycle<release_cycle(index1);
    // Emulate priority for independent IMU/motor traffic for 160 clocks in
    // every 5000-clock window. Never revoke ownership inside an active frame.
    wire tx_allow=(cycle%5000>=160);
    rgb_packet_tx #(.IMAGE_HEIGHT(HEIGHT),.WAIT_CYCLES(100000)) dut(.*);
    always @(posedge clk)begin
        if(rst_n)begin
            if(fifo0_pop && !fifo0_empty)index0<=index0+1;
            if(fifo1_pop && !fifo1_empty)index1<=index1+1;
            if(CHANGE_EPOCH!=0 && !changed && cycle>=CHANGE_EPOCH)begin
                epoch<=2;index0<=0;index1<=0;changed<=1;
            end
        end
        #1;
        if(rst_n)begin
            if(tx_pending && !tx_busy && !tx_allow)blocked_cycles=blocked_cycles+1;
            if(tx_pending && !tx_busy && tx_allow)allow_cycles=allow_cycles+1;
            if(gmii_valid)begin
                if(!previous_valid)begin
                    if(cycle-last_end-1<12)$fatal(1,"IFG under 12");
                    if(cycle-last_end-1<ifg_min)ifg_min=cycle-last_end-1;
                    if(first_start<0)first_start=cycle;
                end
                $fwrite(capture,"%c",gmii_data);packet_bytes=packet_bytes+1;
            end else if(previous_valid)begin
                if(packet_bytes!=1018)$fatal(1,"Wrong packet length %0d",packet_bytes);
                last_end=cycle-1;packet_bytes=0;
            end
            previous_valid=gmii_valid;
            if(index0>=WORDS0 && index1>=WORDS1 && !tx_busy && !tx_pending &&
               !dut.pair_active && dut.load_state==0 && dut.meta0==0 && dut.meta1==0)quiet=quiet+1;else quiet=0;
            if(quiet>=200)begin
                $display("RESULT packets=%0d faults=%0d pairs=%0d cycles=%0d ifg_min=%0d arbiter_blocked=%0d epoch=%0d",packets_sent,pair_fault_count,pairs_started,cycle,ifg_min,blocked_cycles,epoch);
                $fclose(capture);$finish;
            end
            if(cycle>MAX_CYCLES)$fatal(1,"simulation timeout idx=%0d/%0d idx1=%0d/%0d state=%0d pair=%b meta=%0d/%0d",index0,WORDS0,index1,WORDS1,dut.load_state,dut.pair_active,dut.meta0,dut.meta1);
        end
        cycle=cycle+1;
    end
    initial begin
        if(!$value$plusargs("fifo0=%s",input0) || !$value$plusargs("fifo1=%s",input1) || !$value$plusargs("capture=%s",output_path))$fatal(1,"paths missing");
        $readmemh(input0,words0);$readmemh(input1,words1);capture=$fopen(output_path,"wb");
        if(!capture)$fatal(1,"capture open failed");
        repeat(5)@(negedge clk);rst_n=1;
    end
endmodule
