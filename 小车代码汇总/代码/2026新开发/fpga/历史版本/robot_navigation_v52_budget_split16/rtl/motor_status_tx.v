`timescale 1ns/1ps
// ACK1 uses EtherType 88B6, a 16-byte body plus big-endian CCITT CRC16.
// Broadcast destination permits status discovery before a command is accepted.
// NAV_ENABLE emits a separate 0x88B7 NAV1 frame AFTER the byte-identical ACK1.
// The sender retains ownership through both frames and each mandatory IFG.
module motor_status_tx #(parameter NAV_ENABLE=0)(
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
    reg shift_ack_payload,shift_nav_payload;
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
    // Preload while idle, including the actual valid/ready handshake edge.
    // The mailbox keeps this bundle stable until that edge acknowledges it.
    // Neither valid nor the shared-bus grant drives the 336-bit capture CE.
    // Decode the shift CEs one cycle ahead: old BODY indices 14..29/39 use
    // enables computed at indices 13..28/38, preserving every emitted byte.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            saved<=0;saved_nav<=0;shift_ack_payload<=0;shift_nav_payload<=0;
        end else begin
            shift_ack_payload<=state==BODY && !nav_frame && index>=13 && index<29;
            shift_nav_payload<=state==BODY && nav_frame && index>=13 && index<39;
            if(state==IDLE)begin saved<=payload;saved_nav<=nav_payload;end
            else begin
                if(shift_ack_payload)saved<={saved[119:0],8'b0};
                if(shift_nav_payload)saved_nav<={saved_nav[199:0],8'b0};
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            state<=IDLE;index<=0;crc32<=32'hffffffff;
            crc16<=16'hffff;fcs<=0;gmii_data<=0;gmii_valid<=0;
            nav_frame<=0;
        end else begin
            gmii_data<=0;gmii_valid<=0;
            case(state)
                IDLE:if(valid)begin
                    index<=0;crc32<=32'hffffffff;nav_frame<=0;
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
                    end
                    if(nav_frame && index>=14 && index<40)begin
                        crc16<=crc16_byte(crc16,saved_nav[207:200]);
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

module motor_status_gmii #(parameter NAV_ENABLE=0)(
    input wire source_clk,source_rst_n,status_valid,
    input wire [127:0] status_data,
    input wire [207:0] nav_data,
    input wire tx_clk,tx_rst_n,tx_allow,
    output wire tx_pending,tx_busy,gmii_valid,
    output wire [7:0] gmii_data
);
    wire [511:0] network_data;
    wire network_valid,network_ready;
    imu_snapshot_cdc mailbox(
        .src_clk(source_clk),.src_rst_n(source_rst_n),.src_valid(status_valid),
        .src_data({status_data,nav_data,176'b0}),.src_ready(),
        .dst_clk(tx_clk),.dst_rst_n(tx_rst_n),.dst_ready(network_ready && tx_allow),
        .dst_valid(network_valid),.dst_data(network_data));
    assign tx_pending=network_valid;
    assign tx_busy=!network_ready;
    motor_status_tx #(.NAV_ENABLE(NAV_ENABLE)) sender(.clk(tx_clk),.rst_n(tx_rst_n),.valid(network_valid && tx_allow),
        .payload(network_data[511:384]),.nav_payload(network_data[383:176]),.ready(network_ready),
        .gmii_data(gmii_data),.gmii_valid(gmii_valid));
endmodule

// Each source includes IFG in busy and consults allow only before frame start.
module gmii_three_source_share (
    input camera_busy,camera_valid,input [7:0] camera_data,
    input imu_pending,imu_busy,imu_valid,input [7:0] imu_data,
    input motor_pending,motor_busy,motor_valid,input [7:0] motor_data,
    output camera_allow,imu_allow,motor_allow,
    output [7:0] gmii_data,output gmii_valid
);
    assign motor_allow=!camera_busy && !camera_valid && !imu_busy && !imu_valid;
    assign imu_allow=!camera_busy && !camera_valid && !motor_pending && !motor_busy && !motor_valid;
    assign camera_allow=!imu_pending && !imu_busy && !imu_valid &&
                        !motor_pending && !motor_busy && !motor_valid;
    assign gmii_valid=camera_valid || imu_valid || motor_valid;
    assign gmii_data=motor_valid ? motor_data : (imu_valid ? imu_data : camera_data);
endmodule
