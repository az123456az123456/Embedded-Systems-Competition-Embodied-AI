`timescale 1ns/1ps
// Fixed raw Ethernet NAV2 requests; 64-byte body + CRC16. No command is published before valid FCS.
// Bytes on GMII include preamble/SFD and the four received FCS bytes.
module nav_packet_rx (
    input wire clk, rst_n,
    input wire [7:0] data,
    input wire valid, error,
    output reg command_valid,
    output reg [511:0] command_data,
    output reg [31:0] accepted_frames, rejected_frames
);
    reg in_frame, sfd_seen, bad;
    reg [3:0] preamble;
    reg [11:0] count;
    reg [31:0] crc32;
    reg [15:0] crc16, received_crc16;
    reg [511:0] payload;
    reg shift_payload;
    reg [47:0] destination;
    reg [15:0] ethertype;
    function [31:0] crc32_byte;
        input [31:0] previous;
        input [7:0] value;
        reg [31:0] c;
        integer k;
        begin
            c=previous^{24'b0,value};
            for(k=0;k<8;k=k+1) c=c[0] ? (c>>1)^32'hedb88320 : c>>1;
            crc32_byte=c;
        end
    endfunction
    function [15:0] crc16_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] c;
        integer k;
        begin
            c=previous^{value,8'b0};
            for(k=0;k<8;k=k+1) c=c[15] ? (c<<1)^16'h1021 : c<<1;
            crc16_byte=c;
        end
    endfunction
    wire payload_ok=(payload[511:480]==32'h4e415632 && payload[479:472]==1 &&
                     payload[471:464]>=1 && payload[471:464]<=6 &&
                     payload[463:456]==0 && payload[287:256]==0);
    // Decode payload shift one byte early; keep CRC/destination validation
    // out of the 512-bit data-register enable path at125MHz.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin payload<=0;shift_payload<=0;end
        else begin
            shift_payload<=valid && sfd_seen && count>=13 && count<77;
            if(!valid)payload<=0;
            else if(shift_payload)payload<={payload[503:0],data};
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            in_frame<=0;sfd_seen<=0;bad<=0;preamble<=0;count<=0;
            crc32<=32'hffffffff;crc16<=16'hffff;received_crc16<=0;
            destination<=0;ethertype<=0;
            command_valid<=0;command_data<=0;accepted_frames<=0;rejected_frames<=0;
        end else begin
            command_valid<=0;
            if(valid) begin
                in_frame<=1;
                if(error) bad<=1;
                if(!sfd_seen) begin
                    if(data==8'h55 && preamble<8) preamble<=preamble+1'b1;
                    else if(data==8'hd5 && preamble>=7) begin
                        sfd_seen<=1;count<=0;crc32<=32'hffffffff;
                        crc16<=16'hffff;received_crc16<=0;
                        destination<=0;ethertype<=0;
                    end else bad<=1;
                end else begin
                    if(count<2047) count<=count+1'b1; else bad<=1;
                    crc32<=crc32_byte(crc32,data);
                    if(count<6) destination<={destination[39:0],data};
                    if(count==12 || count==13) ethertype<={ethertype[7:0],data};
                    if(count>=14 && count<78) begin
                        crc16<=crc16_byte(crc16,data);
                    end
                    if(count==78 || count==79) received_crc16<={received_crc16[7:0],data};

                end
            end else if(in_frame) begin
                command_data<=payload;
                if(sfd_seen && !bad && count==84 && crc32==32'hdebb20e3 &&
                   destination==48'h02ac13800001 && ethertype==16'h88b9 &&
                   crc16==received_crc16 && payload_ok) begin
                    command_valid<=1;
                    accepted_frames<=accepted_frames+1'b1;
                end else rejected_frames<=rejected_frames+1'b1;
                in_frame<=0;sfd_seen<=0;bad<=0;preamble<=0;count<=0;
            end
        end
    end
endmodule

