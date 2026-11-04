// Exact pre-CE-timing-change sender retained only for cycle equivalence.
`timescale 1ns/1ps
// ACK1 uses EtherType 88B6, a 16-byte body plus big-endian CCITT CRC16.
// Broadcast destination permits status discovery before a command is accepted.
// NAV_ENABLE emits a separate 0x88B7 NAV1 frame AFTER the byte-identical ACK1.
// The sender retains ownership through both frames and each mandatory IFG.
module motor_status_tx_pre_ce #(parameter NAV_ENABLE=0)(
    input wire clk,rst_n,valid,
    input wire [127:0] payload,
    input wire [207:0] nav_payload,
    output wire ready,
    output reg [7:0] gmii_data,
    output reg gmii_valid
);
    localparam IDLE=0,PREAMBLE=1,BODY=2,FCS=3,GAP=4;
    reg [2:0] state;
    reg [6:0] index;
    reg [127:0] saved;
    reg [207:0] saved_nav;
    reg nav_frame;
    reg [31:0] crc32,fcs;
    reg [15:0] crc16;
    reg [7:0] body_byte;
    function [31:0] crc32_byte;
        input [31:0] previous;input [7:0] value;
        reg [31:0] c;integer k;
        begin
            c=previous^{24'b0,value};
            for(k=0;k<8;k=k+1)c=c[0] ? (c>>1)^32'hedb88320 : c>>1;
            crc32_byte=c;
        end
    endfunction
    function [15:0] crc16_byte;
        input [15:0] previous;input [7:0] value;
        reg [15:0] c;integer k;
        begin
            c=previous^{value,8'b0};
            for(k=0;k<8;k=k+1)c=c[15] ? (c<<1)^16'h1021 : c<<1;
            crc16_byte=c;
        end
    endfunction
    wire [31:0] last_crc=~crc32_byte(crc32,gmii_data);
    always @* begin
        case(index)
            0,1,2,3,4,5:body_byte=8'hff;
            6:body_byte=8'h02;7:body_byte=8'hac;8:body_byte=8'h13;
            9:body_byte=8'h80;10:body_byte=8'h00;11:body_byte=8'h01;
            12:body_byte=8'h88;13:body_byte=nav_frame ? 8'hb7 : 8'hb6;
            default:begin
                body_byte=0;
                if(nav_frame)begin
                    if(index>=14 && index<40)body_byte=saved_nav[207:200];
                    if(index==40)body_byte=crc16[15:8];
                    if(index==41)body_byte=crc16[7:0];
                end else begin
                    if(index>=14 && index<30)body_byte=saved[127:120];
                    if(index==30)body_byte=crc16[15:8];
                    if(index==31)body_byte=crc16[7:0];
                end
            end
        endcase
    end
    assign ready=(state==IDLE);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            state<=IDLE;index<=0;saved<=0;crc32<=32'hffffffff;
            crc16<=16'hffff;fcs<=0;gmii_data<=0;gmii_valid<=0;
            saved_nav<=0;nav_frame<=0;
        end else begin
            gmii_data<=0;gmii_valid<=0;
            case(state)
                IDLE:if(valid)begin
                    saved<=payload;index<=0;crc32<=32'hffffffff;
                    saved_nav<=nav_payload;nav_frame<=0;
                    crc16<=16'hffff;state<=PREAMBLE;
                end
                PREAMBLE:begin
                    gmii_valid<=1;gmii_data<=index==7 ? 8'hd5 : 8'h55;
                    if(index==7)begin index<=0;state<=BODY;end else index<=index+1'b1;
                end
                BODY:begin
                    gmii_valid<=1;gmii_data<=body_byte;
                    if(index!=0)crc32<=crc32_byte(crc32,gmii_data);
                    if(!nav_frame && index>=14 && index<30)begin
                        crc16<=crc16_byte(crc16,saved[127:120]);
                        saved<={saved[119:0],8'b0};
                    end
                    if(nav_frame && index>=14 && index<40)begin
                        crc16<=crc16_byte(crc16,saved_nav[207:200]);
                        saved_nav<={saved_nav[199:0],8'b0};
                    end
                    if(index==59)begin index<=0;state<=FCS;end else index<=index+1'b1;
                end
                FCS:begin
                    gmii_valid<=1;
                    if(index==0)begin gmii_data<=last_crc[7:0];fcs<=last_crc>>8;end
                    else begin gmii_data<=fcs[7:0];fcs<=fcs>>8;end
                    if(index==3)begin index<=0;state<=GAP;end else index<=index+1'b1;
                end
                GAP:if(index==11)begin
                    index<=0;
                    if(NAV_ENABLE && !nav_frame)begin
                        nav_frame<=1;state<=PREAMBLE;
                        crc32<=32'hffffffff;crc16<=16'hffff;
                    end else state<=IDLE;
                end else index<=index+1'b1;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
