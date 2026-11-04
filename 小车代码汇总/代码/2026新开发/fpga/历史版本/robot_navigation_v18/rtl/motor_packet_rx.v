`timescale 1ns/1ps
// Fixed raw Ethernet MOT1 wheel commands / MOT2 target-bearing commands. No command is published before valid FCS.
// Bytes on GMII include preamble/SFD and the four received FCS bytes.
module motor_packet_rx (
    input wire clk, rst_n,
    input wire [7:0] data,
    input wire valid, error,
    output reg command_valid,
    output reg [127:0] command_data,
    output reg [31:0] accepted_frames, rejected_frames
);
    reg in_frame, sfd_seen, bad;
    reg [3:0] preamble;
    reg [11:0] count;
    reg [31:0] crc32;
    reg [15:0] crc16, received_crc16;
    reg [127:0] payload,checked_payload;
    reg check_pending,checked_frame_good;
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
    wire format_ok=(checked_payload[127:96]==32'h4d4f5431 && checked_payload[95:88]==1) ||
                   (checked_payload[127:96]==32'h4d4f5432 && checked_payload[95:88]==2);
    wire payload_ok=(format_ok &&
                     checked_payload[87:81]==0 && checked_payload[15:12]==0 &&
                     checked_payload[11:8]!=0 && checked_payload[11:8]!=15 && checked_payload[7:4]==0);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            in_frame<=0;sfd_seen<=0;bad<=0;preamble<=0;count<=0;
            crc32<=32'hffffffff;crc16<=16'hffff;received_crc16<=0;
            payload<=0;destination<=0;ethertype<=0;
            command_valid<=0;command_data<=0;accepted_frames<=0;rejected_frames<=0;
            checked_payload<=0;check_pending<=0;checked_frame_good<=0;
        end else begin
            command_valid<=0;check_pending<=0;
            // Commit on a second clock. RX_CTL/FCS fan-in does not drive the
            // 128-bit output enable; command_valid remains atomic with data.
            if(check_pending)begin
                command_data<=checked_payload;
                if(checked_frame_good && payload_ok)begin
                    command_valid<=1;accepted_frames<=accepted_frames+1'b1;
                end else rejected_frames<=rejected_frames+1'b1;
            end
            if(valid) begin
                in_frame<=1;
                if(error) bad<=1;
                if(!sfd_seen) begin
                    if(data==8'h55 && preamble<8) preamble<=preamble+1'b1;
                    else if(data==8'hd5 && preamble>=7) begin
                        sfd_seen<=1;count<=0;crc32<=32'hffffffff;
                        crc16<=16'hffff;received_crc16<=0;
                        payload<=0;destination<=0;ethertype<=0;
                    end else bad<=1;
                end else begin
                    if(count<2047) count<=count+1'b1; else bad<=1;
                    crc32<=crc32_byte(crc32,data);
                    if(count<6) destination<={destination[39:0],data};
                    if(count==12 || count==13) ethertype<={ethertype[7:0],data};
                    if(count>=14 && count<30) begin
                        payload<={payload[119:0],data};
                        crc16<=crc16_byte(crc16,data);
                    end
                    if(count==30 || count==31) received_crc16<={received_crc16[7:0],data};
                    // The specified wire format is a padded minimum Ethernet frame.
                    if(count>=32 && count<60 && data!=0) bad<=1;
                end
            end else if(in_frame) begin
                checked_payload<=payload;check_pending<=1;
                checked_frame_good<=sfd_seen && !bad && count==64 && crc32==32'hdebb20e3 &&
                   destination==48'h02ac13800001 && ethertype==16'h88b6 && crc16==received_crc16;
                in_frame<=0;sfd_seen<=0;bad<=0;preamble<=0;count<=0;
            end
        end
    end
endmodule

// Bundled-data handshake; held stays constant until destination acknowledgement.
// The motor clock never relies on the receive clock to expire its watchdog.
module motor_command_cdc (
    input wire src_clk, src_rst_n, src_valid,
    input wire [127:0] src_data,
    output wire src_ready,
    input wire dst_clk, dst_rst_n,
    output reg dst_valid,
    output reg [127:0] dst_data
);
    reg [127:0] held;
    reg request, acknowledge;
    (* syn_async_reg=1 *) reg [1:0] ack_sync, req_sync;
    assign src_ready=(request==ack_sync[1]);
    always @(posedge src_clk or negedge src_rst_n) begin
        if(!src_rst_n) begin held<=0;request<=0;ack_sync<=0;end
        else begin
            ack_sync<={ack_sync[0],acknowledge};
            if(src_valid && src_ready) begin held<=src_data;request<=!request;end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if(!dst_rst_n) begin acknowledge<=0;req_sync<=0;dst_valid<=0;dst_data<=0;end
        else begin
            req_sync<={req_sync[0],request};dst_valid<=0;
            if(req_sync[1]!=acknowledge) begin
                dst_data<=held;dst_valid<=1;acknowledge<=req_sync[1];
            end
        end
    end
endmodule

module motor_rgmii_rx (
    input wire reset_n, rgmii_rx_clk,
    input wire [3:0] rgmii_rxd,
    input wire rgmii_rx_ctl,
    output wire command_valid,
    output wire [127:0] command_data,
    output wire receive_reset_n
);
    wire [3:0] q0,q1;
    wire ctl0,ctl1;
    (* syn_async_reg=1 *) reg [1:0] reset_sync=0;
    always @(posedge rgmii_rx_clk or negedge reset_n)
        if(!reset_n) reset_sync<=0; else reset_sync<={reset_sync[0],1'b1};
    assign receive_reset_n=reset_sync[1];
    genvar bitno;
    generate for(bitno=0;bitno<4;bitno=bitno+1) begin:g_data
        IDDR iddr(.Q0(q0[bitno]),.Q1(q1[bitno]),.D(rgmii_rxd[bitno]),.CLK(rgmii_rx_clk));
    end endgenerate
    IDDR iddr_ctl(.Q0(ctl0),.Q1(ctl1),.D(rgmii_rx_ctl),.CLK(rgmii_rx_clk));
    motor_packet_rx parser(.clk(rgmii_rx_clk),.rst_n(receive_reset_n),
        .data({q1,q0}),.valid(ctl0),.error(ctl0^ctl1),
        .command_valid(command_valid),.command_data(command_data),
        .accepted_frames(),.rejected_frames());
endmodule
