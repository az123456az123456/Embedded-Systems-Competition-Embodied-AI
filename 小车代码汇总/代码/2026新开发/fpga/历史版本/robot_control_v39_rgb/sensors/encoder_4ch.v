`timescale 1ns/1ps
// Independent read-only four-wheel 4x quadrature capture. {A,B}: A-leading
// 00->10->11->01->00 is positive before INVERT_MASK. Initial stable level is
// learned without counting. Count arithmetic wraps modulo 2^32.
module encoder_4ch #(
    parameter integer FILTER_CYCLES=16,
    parameter [3:0] INVERT_MASK=4'b0000
)(
    input wire clk,rst_n,
    input wire [3:0] encoder_a,encoder_b,
    output wire [127:0] positions,
    output wire [3:0] illegal_seen
);
    genvar i;
    generate for(i=0;i<4;i=i+1)begin: wheel
        (* syn_async_reg=1 *) reg [1:0] a_sync,b_sync;
        reg [1:0] candidate,previous;
        reg [15:0] stable;
        reg initialized;
        reg [31:0] position;
        reg illegal;
        assign positions[i*32 +: 32]=position;
        assign illegal_seen[i]=illegal;
        wire [1:0] current={a_sync[1],b_sync[1]};
        wire positive=({previous,candidate}==4'b0010)||({previous,candidate}==4'b1011)||
                      ({previous,candidate}==4'b1101)||({previous,candidate}==4'b0100);
        wire negative=({previous,candidate}==4'b0001)||({previous,candidate}==4'b0111)||
                      ({previous,candidate}==4'b1110)||({previous,candidate}==4'b1000);
        always @(posedge clk)begin
            if(!rst_n)begin
                a_sync<=0;b_sync<=0;candidate<=0;previous<=0;stable<=0;
                initialized<=0;position<=0;illegal<=0;
            end else begin
                a_sync<={a_sync[0],encoder_a[i]};b_sync<={b_sync[0],encoder_b[i]};
                if(current!=candidate)begin candidate<=current;stable<=0;end
                else if(stable<FILTER_CYCLES-1)stable<=stable+1'b1;
                else if(!initialized)begin initialized<=1;previous<=candidate;end
                else if(previous!=candidate)begin
                    previous<=candidate;
                    if(positive||negative)begin
                        if(positive ^ INVERT_MASK[i])position<=position+32'd1;
                        else position<=position-32'd1;
                    end else illegal<=1;
                end
            end
        end
    end endgenerate
endmodule
