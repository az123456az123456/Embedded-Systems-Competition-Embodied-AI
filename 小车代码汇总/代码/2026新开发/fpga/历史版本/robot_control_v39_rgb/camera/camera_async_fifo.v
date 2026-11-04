`timescale 1ns/1ps
// Camera byte FIFO, independent write/read clocks, synchronous BRAM FWFT.
// Binary pointers address RAM; only Gray pointers cross the clock boundary.
// Registered empty/full flags do not feed a next-pointer comparison into
// the memory clock enable. The read port preloads the next head every cycle.
// Reset must cover both domains together; release is synchronized locally.
module camera_async_fifo #(
    parameter ADDR_WIDTH=14
)(
    input wire wrclk,rdclk,aclr,
    input wire wrreq,rdreq,
    input wire [7:0] wrdata,
    output reg [7:0] rddata,
    output wire [ADDR_WIDTH-1:0] rdusedw,
    output wire [ADDR_WIDTH-1:0] wrusedw,
    output reg empty,full
);
    localparam PTR_WIDTH=ADDR_WIDTH+1;
    (* syn_ramstyle="block_ram" *) reg [7:0] memory[0:(1<<ADDR_WIDTH)-1];
    reg [PTR_WIDTH-1:0] wbin,wgray,rbin,rgray,rbin_plus_one;
    (* syn_async_reg=1 *) reg [PTR_WIDTH-1:0] rq1_wgray,rq2_wgray,wq1_rgray,wq2_rgray;
    (* syn_async_reg=1 *) reg [1:0] wr_reset_sync=0,rd_reset_sync=0;
    wire wr_reset_n=wr_reset_sync[1],rd_reset_n=rd_reset_sync[1];
    wire write_take=wrreq && !full && wr_reset_n;
    wire read_take=rdreq && !empty && rd_reset_n;
    wire [PTR_WIDTH-1:0] wbin_next=wbin+write_take;
    // Keep the next address ready in a register: the BRAM prefetch port sees
    // only a mux, rather than reset/enable logic followed by a carry chain.
    wire [PTR_WIDTH-1:0] rbin_next=read_take ? rbin_plus_one : rbin;
    wire [PTR_WIDTH-1:0] wgray_next=(wbin_next>>1)^wbin_next;
    wire [PTR_WIDTH-1:0] rgray_next=(rbin_next>>1)^rbin_next;
    wire full_next=(wgray_next=={~wq2_rgray[PTR_WIDTH-1:PTR_WIDTH-2],wq2_rgray[PTR_WIDTH-3:0]});
    wire empty_next=(rgray_next==rq2_wgray);
    function [PTR_WIDTH-1:0] gray_to_binary;
        input [PTR_WIDTH-1:0] gray;
        integer k;
        begin
            // Independent reduction XORs synthesize as balanced trees rather
            // than the serial bit-to-bit XOR chain on the old count path.
            for(k=0;k<PTR_WIDTH;k=k+1)
                gray_to_binary[k]=^(gray>>k);
        end
    endfunction
    // Decode synchronized writes in their own stage before subtraction. The
    // empty flag already uses the same old rq2_wgray value on this edge, so
    // !empty implies decoded_write_bin is ahead of rbin before any read.
    // Local reads decrement immediately; new writes take two additional
    // cycles to reach the public count, keeping it conservative.
    reg [PTR_WIDTH-1:0] decoded_write_bin,read_count;
    wire [PTR_WIDTH-1:0] read_count_next=decoded_write_bin-rbin_next;
    // Optional legacy write-domain count. Synchronized reads make this an
    // upper bound on occupied space; unused instances optimize it away.
    wire [PTR_WIDTH-1:0] write_count=wbin-gray_to_binary(wq2_rgray);
    // Saturate full depth because the legacy count port has no extra MSB.
    assign rdusedw=!rd_reset_n ? {ADDR_WIDTH{1'b0}} :
                   read_count[ADDR_WIDTH] ? {ADDR_WIDTH{1'b1}} : read_count[ADDR_WIDTH-1:0];
    assign wrusedw=!wr_reset_n ? {ADDR_WIDTH{1'b0}} :
                   write_count[ADDR_WIDTH] ? {ADDR_WIDTH{1'b1}} : write_count[ADDR_WIDTH-1:0];
    always @(posedge wrclk or posedge aclr)
        if(aclr) wr_reset_sync<=0; else wr_reset_sync<={wr_reset_sync[0],1'b1};
    always @(posedge rdclk or posedge aclr)
        if(aclr) rd_reset_sync<=0; else rd_reset_sync<={rd_reset_sync[0],1'b1};
    always @(posedge wrclk or negedge wr_reset_n)begin
        if(!wr_reset_n)begin wbin<=0;wgray<=0;full<=0;wq1_rgray<=0;wq2_rgray<=0;end
        else begin
            wq1_rgray<=rgray;wq2_rgray<=wq1_rgray;
            wbin<=wbin_next;wgray<=wgray_next;full<=full_next;
        end
    end
    always @(posedge rdclk or negedge rd_reset_n)begin
        if(!rd_reset_n)begin
            rbin<=0;rgray<=0;rbin_plus_one<=1;empty<=1;
            rq1_wgray<=0;rq2_wgray<=0;decoded_write_bin<=0;read_count<=0;
        end
        else begin
            rq1_wgray<=wgray;rq2_wgray<=rq1_wgray;
            rbin<=rbin_next;rgray<=rgray_next;empty<=empty_next;
            rbin_plus_one<=rbin_plus_one+read_take;
            decoded_write_bin<=gray_to_binary(rq2_wgray);
            read_count<=read_count_next;
        end
    end
    // No reset on RAM data ports: inferred dual-clock BRAM needs only
    // pointer/valid resets. rddata is unspecified while empty/reset.
    always @(posedge wrclk)
        if(write_take) memory[wbin[ADDR_WIDTH-1:0]]<=wrdata;
    always @(posedge rdclk)
        rddata<=memory[rbin_next[ADDR_WIDTH-1:0]];
endmodule
