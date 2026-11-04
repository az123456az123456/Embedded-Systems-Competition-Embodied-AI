`timescale 1ns/1ps
// Compare the optimized and original mailbox on every visible handshake,
// including destination stalls, unrelated clocks and changing idle inputs.
module tb_mailbox_idle_capture;
    reg src_clk=0,dst_clk=0,rst_n=0,src_valid=0,dst_ready=0;
    reg [511:0] src_data=0;
    wire a_ready,b_ready,a_valid,b_valid;
    wire [511:0] a_data,b_data;
    integer i,accepted=0,delivered=0;
    reg [31:0] prng=32'h5e33c001;
    reg [511:0] expected;
    always #4 src_clk=~src_clk;
    always #10 dst_clk=~dst_clk;
    imu_snapshot_cdc original(.src_clk(src_clk),.src_rst_n(rst_n),
        .src_valid(src_valid),.src_data(src_data),.src_ready(a_ready),
        .dst_clk(dst_clk),.dst_rst_n(rst_n),.dst_ready(dst_ready),
        .dst_valid(a_valid),.dst_data(a_data));
    imu_snapshot_cdc #(.CAPTURE_WHEN_IDLE(1)) optimized(
        .src_clk(src_clk),.src_rst_n(rst_n),.src_valid(src_valid),
        .src_data(src_data),.src_ready(b_ready),.dst_clk(dst_clk),
        .dst_rst_n(rst_n),.dst_ready(dst_ready),.dst_valid(b_valid),.dst_data(b_data));
    always @(posedge src_clk) if(rst_n) begin
        if(a_ready!==b_ready) $fatal(1,"ready changed");
        if(src_valid && a_ready) begin expected=src_data;accepted=accepted+1;end
    end
    always @(posedge dst_clk) if(rst_n) begin
        if(a_valid!==b_valid) $fatal(1,"valid latency changed");
        if(a_valid && (a_data!==b_data || b_data!==expected))
            $fatal(1,"published payload incoherent");
        if(a_valid && dst_ready) delivered=delivered+1;
    end
    initial begin
        #27;rst_n=1;
        for(i=0;i<20000;i=i+1)begin
            @(negedge src_clk);
            prng={prng[30:0],prng[31]^prng[21]^prng[1]^prng[0]};
            src_data={16{prng}};src_valid=prng[0];
            // Keep frequent long enough stalls to exercise held stability.
            dst_ready=prng[3] && (i%157>35);
        end
        @(negedge src_clk);src_valid=0;dst_ready=1;
        repeat(30)@(posedge dst_clk);
        if(accepted<500 || delivered!=accepted) $fatal(1,"lost transactions %0d/%0d",delivered,accepted);
        $display("PASS idle-capture mailbox: %0d coherent transactions, unchanged latency/backpressure",delivered);
        $finish;
    end
endmodule
