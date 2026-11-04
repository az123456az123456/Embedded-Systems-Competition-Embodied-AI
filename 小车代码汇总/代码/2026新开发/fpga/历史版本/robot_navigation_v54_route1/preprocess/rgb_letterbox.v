// Sparse 569x640 RGB -> 640x640 RGB, exact U8 114 border (35 left / 36 right).
// The fixed downscale supplies fewer than one RGB pixel/clock. A small FIFO
// absorbs border emission; no frame RAM and no camera backpressure. A sender
// must discard the whole current tensor on out_abort (FIFO/protocol fault).
module rgb_letterbox #(
    parameter integer ACTIVE_WIDTH=569,
    parameter integer ACTIVE_HEIGHT=640,
    parameter integer PAD_LEFT=35,
    parameter integer PAD_RIGHT=36,
    parameter integer FIFO_AW=6
)(
    input wire clk,input wire rst_n,
    input wire in_valid,input wire in_sof,input wire in_eol,input wire in_eof,
    input wire [9:0] in_x,input wire [9:0] in_y,input wire [23:0] in_rgb,
    input wire in_abort,
    output reg out_valid,output reg out_sof,output reg out_eol,output reg out_eof,
    output reg [9:0] out_x,output reg [9:0] out_y,output reg [23:0] out_rgb,
    output reg out_abort,output reg [FIFO_AW:0] fifo_peak
);
    localparam integer OUTPUT_WIDTH=PAD_LEFT+ACTIVE_WIDTH+PAD_RIGHT;
    localparam integer DATA_END=PAD_LEFT+ACTIVE_WIDTH;
    localparam [9:0] LAST_ACTIVE_X=ACTIVE_WIDTH[9:0]-10'd1;
    localparam [9:0] LAST_Y=ACTIVE_HEIGHT[9:0]-10'd1;
    localparam [9:0] LAST_X=OUTPUT_WIDTH[9:0]-10'd1;
    localparam integer DEPTH=(1<<FIFO_AW);
    // Synchronous BRAM read with a forwarding head. All queued pixels remain
    // included in count (including the prefetched/forwarded head), so the
    // advertised capacity is exactly DEPTH, not DEPTH plus a hidden skid.
    (* syn_ramstyle="block_ram" *)reg [23:0] fifo[0:DEPTH-1];
    reg [23:0] memory_head,forward_head;
    reg head_forward,head_valid;
    reg [FIFO_AW-1:0] rd,wr;reg [FIFO_AW:0] count;
    reg input_active,output_active;reg [9:0] expected_x,expected_y,column,row;
    wire begin_frame=in_valid&&in_sof&&in_x==0&&in_y==0&&in_eol==(ACTIVE_WIDTH==1);
    wire ordered=in_valid&&input_active&&!in_sof&&in_x==expected_x&&in_y==expected_y&&
        in_eol==(in_x==LAST_ACTIVE_X)&&(!in_eof||(in_y==LAST_Y&&in_eol));
    wire malformed=in_valid&&!begin_frame&&(in_sof||input_active&&!ordered);
    wire border=column<PAD_LEFT[9:0]||column>=DATA_END[9:0];
    wire pop=output_active&&!border&&count!=0&&head_valid;
    wire push=ordered;
    wire full=count[FIFO_AW];
    wire abort_now=in_abort||malformed||(push&&full&&!pop);
    wire store=rst_n&&!abort_now&&(begin_frame||(push&&(!full||pop)));
    wire [FIFO_AW-1:0] store_address=begin_frame?{FIFO_AW{1'b0}}:wr;
    wire [FIFO_AW-1:0] read_address=pop?rd+1'b1:rd;
    // If an old head is consumed, prefetch the next previously stored word.
    // If the next head is this cycle's new input, forward it instead. This
    // avoids every same-address RAM read/write and still sustains 1 px/clk.
    wire [FIFO_AW:0] consumed={{FIFO_AW{1'b0}},pop};
    wire read_enable=rst_n&&!abort_now&&!begin_frame&&(count>consumed);
    wire forward_enable=store&&(begin_frame||(count==consumed));
    wire [23:0] queued_head=head_forward?forward_head:memory_head;
    always @(posedge clk)begin
        if(store)fifo[store_address]<=in_rgb;
        if(read_enable)memory_head<=fifo[read_address];
    end
    always @(posedge clk)begin
        if(!rst_n)begin
            rd<=0;wr<=0;count<=0;fifo_peak<=0;input_active<=0;output_active<=0;
            expected_x<=0;expected_y<=0;column<=0;row<=0;
            out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_abort<=0;
            out_x<=0;out_y<=0;out_rgb<=0;
            head_forward<=0;head_valid<=0;forward_head<=0;
        end else begin
            out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_abort<=0;
            head_valid<=read_enable||forward_enable;
            head_forward<=forward_enable;
            if(forward_enable)forward_head<=in_rgb;
            if(output_active&&(border||pop))begin
                out_valid<=1;out_x<=column;out_y<=row;
                out_rgb<=border?24'h727272:queued_head;
                out_sof<=column==0&&row==0;out_eol<=column==LAST_X;
                out_eof<=column==LAST_X&&row==LAST_Y;
                if(column==LAST_X)begin
                    column<=0;
                    if(row==LAST_Y)begin output_active<=0;row<=0;end
                    else row<=row+1'b1;
                end else column<=column+1'b1;
            end
            if(pop)rd<=rd+1'b1;
            if(push&&(!full||pop))wr<=wr+1'b1;
            case({push,pop})
                2'b10:begin count<=count+1'b1;if(count+1'b1>fifo_peak)fifo_peak<=count+1'b1;end
                2'b01:count<=count-1'b1;
                default:begin end
            endcase
            if(push)begin
                if(in_eol)begin expected_x<=0;
                    if(in_y==LAST_Y)begin input_active<=0;expected_y<=0;end
                    else expected_y<=expected_y+1'b1;
                end else expected_x<=expected_x+1'b1;
            end
            if(begin_frame)begin
                // A new frame supersedes an incomplete one. The very first
                // pixel enters an empty FIFO; border starts next cycle.
                out_abort<=output_active||input_active;
                rd<=0;wr<=1;count<=1;
                if(fifo_peak==0)fifo_peak<=1;
                column<=0;row<=0;input_active<=1;output_active<=1;
                expected_x<=ACTIVE_WIDTH==1?10'd0:10'd1;expected_y<=ACTIVE_WIDTH==1?10'd1:10'd0;
                out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;
            end
            if(in_abort||malformed||(push&&full&&!pop))begin
                rd<=0;wr<=0;count<=0;input_active<=0;output_active<=0;
                expected_x<=0;expected_y<=0;column<=0;row<=0;
                out_valid<=0;out_sof<=0;out_eol<=0;out_eof<=0;out_abort<=1;
            end
        end
    end
endmodule
