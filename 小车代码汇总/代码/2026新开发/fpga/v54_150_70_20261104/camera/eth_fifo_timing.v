`timescale 1ns/1ps
// The row controller's wrclk and GMII RdClk are BOTH clk_125m. Use a
// synchronous FIFO here; the camera PCLK crossings remain asynchronous.
// Preserve the 8KiB BRAM FWFT interface and saturating 13-bit counts.
module eth_dcfifo(
    input wire [7:0] Data,
    input wire Reset,WrClk,RdClk,WrEn,RdEn,
    output wire [12:0] Wnum,Rnum,
    output wire [7:0] Q,
    output wire Empty,Full
);
    eth_sync_fifo #(.ADDR_WIDTH(13)) fifo(
        .clk(RdClk),.aclr(Reset),
        .wrreq(WrEn),.rdreq(RdEn),.wrdata(Data),
        .rddata(Q),.rdusedw(Rnum),.wrusedw(Wnum),
        .empty(Empty),.full(Full)
    );
endmodule

module eth_sync_fifo #(parameter ADDR_WIDTH=13)(
    input wire clk,aclr,wrreq,rdreq,
    input wire [7:0] wrdata,
    output reg [7:0] rddata,
    output wire [ADDR_WIDTH-1:0] rdusedw,wrusedw,
    output wire empty,full
);
    localparam DEPTH=1<<ADDR_WIDTH;
    (* syn_ramstyle="block_ram" *) reg [7:0] memory[0:DEPTH-1];
    (* syn_async_reg=1 *) reg [1:0] rd_reset_sync=0;
    wire reset_n=rd_reset_sync[1];
    reg [ADDR_WIDTH-1:0] waddr,raddr,raddr_plus_one;
    // One local occupancy counter; no Gray decode or pointer subtraction.
    reg [ADDR_WIDTH:0] used;
    reg empty_q,full_q;
    assign empty=!reset_n || empty_q;
    assign full=!reset_n || full_q;
    wire write_take=wrreq&&!full,read_take=rdreq&&!empty;
    wire [ADDR_WIDTH-1:0] read_next=read_take?raddr_plus_one:raddr;
    assign rdusedw=used[ADDR_WIDTH]?{ADDR_WIDTH{1'b1}}:used[ADDR_WIDTH-1:0];
    assign wrusedw=rdusedw;
    always @(posedge clk or posedge aclr)
        if(aclr)rd_reset_sync<=0;else rd_reset_sync<={rd_reset_sync[0],1'b1};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            waddr<=0;raddr<=0;raddr_plus_one<=1;used<=0;
            empty_q<=1'b1;full_q<=1'b0;
        end
        else begin
            if(write_take)waddr<=waddr+1'b1;
            raddr<=read_next;
            raddr_plus_one<=raddr_plus_one+read_take;
            case({write_take,read_take})
                // Compare the CURRENT count against constants. Register the
                // next flags at the same edge as used, so the flags are exact
                // without a counter carry chain followed by a comparator.
                2'b10:begin
                    used<=used+1'b1;
                    empty_q<=1'b0;full_q<=used==DEPTH-1;
                end
                2'b01:begin
                    used<=used-1'b1;
                    empty_q<=used==1;full_q<=1'b0;
                end
                default:begin end
            endcase
        end
    end
    // Write-through forwarding handles empty->nonempty and a simultaneous
    // write/read of the final old byte without relying on BRAM collision mode.
    always @(posedge clk)begin
        if(write_take)memory[waddr]<=wrdata;
        if(write_take && waddr==read_next)rddata<=wrdata;
        else rddata<=memory[read_next];
    end
endmodule
