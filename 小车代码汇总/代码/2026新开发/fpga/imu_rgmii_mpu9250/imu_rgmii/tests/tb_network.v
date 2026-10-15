`timescale 1ns/1ps
module tb_network;
    reg src_clk=0,dst_clk=0,rst_n=0,run_dst=1;
    always #10 src_clk=~src_clk;
    always #4 if (run_dst) dst_clk=~dst_clk;
    reg valid=0;
    reg [511:0] data=0;
    wire ready,net_valid,net_ready;
    wire [511:0] net_data;
    wire [7:0] gmii;
    wire dv;
    imu_snapshot_cdc cdc (.src_clk(src_clk),.src_rst_n(rst_n),.src_valid(valid),
        .src_data(data),.src_ready(ready),.dst_clk(dst_clk),.dst_rst_n(rst_n),
        .dst_ready(net_ready),.dst_valid(net_valid),.dst_data(net_data));
    imu_ethernet_tx tx (.clk(dst_clk),.rst_n(rst_n),.valid(net_valid),.payload(net_data),
        .ready(net_ready),.gmii_data(gmii),.gmii_valid(dv));
    integer f, count=0, frames=0, gap=1000;
    reg was_dv=0;
    always @(negedge dst_clk) begin
        if (dv) begin
            if (!was_dv && gap<12) $fatal(1,"Short interframe gap");
            $fwrite(f,"%02x\n",gmii); count=count+1; gap=0;
        end else begin
            gap=gap+1;
            if (was_dv) begin
                if(count!=90) $fatal(1,"Frame size %d",count);
                $fwrite(f,"END\n"); count=0; frames=frames+1;
            end
        end
        was_dv=dv;
    end
    task send;
        input [511:0] payload;
        begin
            wait(ready); @(negedge src_clk); data=payload; valid=1;
            @(negedge src_clk); valid=0;
        end
    endtask
    reg [511:0] fixture;
    initial begin
        f=$fopen("build/frames.hex","w");
        // Full protocol fixture: signed boundaries and distinct ASA factors.
        fixture={32'h494d5539,8'd1,8'h92,16'h000f,32'd1,32'd1234,
          16'h4000,16'hc000,16'h0000,16'h0000,16'h0083,16'hff7d,16'h0000,
          16'd100,16'hff9c,16'd200,24'h80a060,8'h71,8'h48,8'h68,
          16'd2,32'd3,32'd4,32'd1236,16'd2,16'd100,32'd7};
        #50; rst_n=1;
        send(fixture);
        wait(frames==1);
        // Stall destination clock, publish once, then change source data.
        @(negedge dst_clk); run_dst=0;
        send(fixture ^ 512'd1);
        repeat(10) @(negedge src_clk);
        if (ready) $fatal(1,"Mailbox acknowledged without destination clock");
        data=0; valid=1; repeat(10) @(negedge src_clk); valid=0;
        run_dst=1;
        wait(frames==2);
        send(fixture ^ 512'd2);
        wait(frames==3); #100;
        $fclose(f); $display("PASS network: CDC backpressure, stopped clock, frame length and IFG"); $finish;
    end
    initial begin #100000; $fatal(1,"Network timeout"); end
endmodule
