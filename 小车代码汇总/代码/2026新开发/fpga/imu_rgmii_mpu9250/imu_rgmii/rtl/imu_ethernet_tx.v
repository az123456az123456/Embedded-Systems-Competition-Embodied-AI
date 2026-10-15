`timescale 1ns/1ps
// Coherent bundled-data mailbox. Both resets must be asserted together at
// power-up. Data is held from request until the synchronized acknowledgement.
module imu_snapshot_cdc (
    input wire src_clk, src_rst_n, src_valid,
    input wire [511:0] src_data,
    output wire src_ready,
    input wire dst_clk, dst_rst_n, dst_ready,
    output wire dst_valid,
    output wire [511:0] dst_data
);
    reg [511:0] held;
    reg request, acknowledge;
    (* syn_async_reg=1 *) reg [1:0] ack_sync, req_sync;
    assign src_ready=(request==ack_sync[1]);
    assign dst_valid=(req_sync[1]!=acknowledge);
    assign dst_data=held;
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin held<=0; request<=0; ack_sync<=0; end
        else begin
            ack_sync<={ack_sync[0],acknowledge};
            if (src_valid && src_ready) begin held<=src_data; request<=!request; end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin acknowledge<=0; req_sync<=0; end
        else begin
            req_sync<={req_sync[0],request};
            if (dst_valid && dst_ready) acknowledge<=req_sync[1];
        end
    end
endmodule

// 1 Gb/s GMII transmitter, raw Ethernet EtherType 88B5. Fixed 64-byte payload.
// Generates preamble, SFD, CRC32 FCS and at least 12 byte-times of IFG.
module imu_ethernet_tx (
    input wire clk, rst_n, valid,
    input wire [511:0] payload,
    output wire ready,
    output reg [7:0] gmii_data,
    output reg gmii_valid
);
    localparam IDLE=0, PREAMBLE=1, BODY=2, FCS=3, GAP=4;
    reg [2:0] state;
    reg [6:0] index;
    reg [511:0] saved;
    reg [31:0] crc, fcs;
    reg [7:0] body_byte;
    function [31:0] crc_byte;
        input [31:0] previous;
        input [7:0] value;
        reg [31:0] c;
        integer k;
        begin
            c=previous^{24'b0,value};
            for (k=0;k<8;k=k+1) c=c[0] ? (c>>1)^32'hedb88320 : c>>1;
            crc_byte=c;
        end
    endfunction
    always @* begin
        case (index)
            0,1,2,3,4,5: body_byte=8'hff;
            6: body_byte=8'h02;
            7: body_byte=8'hac;
            8: body_byte=8'h13;
            9: body_byte=8'h80;
            10: body_byte=8'h00;
            11: body_byte=8'h01;
            12: body_byte=8'h88;
            13: body_byte=8'hb5;
            default: body_byte=saved[511-(index-14)*8 -: 8];
        endcase
    end
    assign ready=(state==IDLE);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; index<=0; saved<=0; crc<=32'hffffffff;
            fcs<=0; gmii_data<=0; gmii_valid<=0;
        end else begin
            gmii_valid<=0; gmii_data<=0;
            case (state)
                IDLE: if (valid) begin
                    saved<=payload; index<=0; crc<=32'hffffffff; state<=PREAMBLE;
                end
                PREAMBLE: begin
                    gmii_valid<=1; gmii_data<=(index==7) ? 8'hd5 : 8'h55;
                    if (index==7) begin index<=0; state<=BODY; end
                    else index<=index+1'b1;
                end
                BODY: begin
                    gmii_valid<=1; gmii_data<=body_byte; crc<=crc_byte(crc,body_byte);
                    if (index==77) begin
                        fcs<=~crc_byte(crc,body_byte); index<=0; state<=FCS;
                    end else index<=index+1'b1;
                end
                FCS: begin
                    gmii_valid<=1; gmii_data<=fcs[7:0]; fcs<=fcs>>8;
                    if (index==3) begin index<=0; state<=GAP; end
                    else index<=index+1'b1;
                end
                GAP: if (index==11) begin state<=IDLE; index<=0; end
                     else index<=index+1'b1;
                default: state<=IDLE;
            endcase
        end
    end
endmodule
