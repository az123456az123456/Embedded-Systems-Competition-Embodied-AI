`timescale 1ns/1ps
// 0x88B8 / SNS1: 64 payload bytes, CCITT-FALSE CRC16, Ethernet FCS.
module sensor_status_tx(
    input wire clk,rst_n,valid,input wire [511:0] payload,
    output wire ready,output reg active,output reg [7:0] gmii_data,output reg gmii_valid
);
    localparam IDLE=0,PREAMBLE=1,BODY=2,FCS=3,GAP=4;
    reg [2:0] state;
    reg [6:0] index;
    reg [511:0] saved;
    reg shift_payload;
    reg [31:0] crc32,fcs;
    reg [15:0] crc16;
    reg [7:0] body_byte;
    function [31:0] crc32_byte;
        input [31:0] previous;input [7:0] value;
        reg [31:0] c;integer k;
        begin c=previous^{24'b0,value};for(k=0;k<8;k=k+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc32_byte=c;end
    endfunction
    function [15:0] crc16_byte;
        input [15:0] previous;input [7:0] value;
        reg [15:0] c;integer k;
        begin c=previous^{value,8'b0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc16_byte=c;end
    endfunction
    wire [31:0] last_crc=~crc32_byte(crc32,gmii_data);
    always @*begin
        case(index)
            0,1,2,3,4,5:body_byte=8'hff;
            6:body_byte=8'h02;7:body_byte=8'hac;8:body_byte=8'h13;
            9:body_byte=8'h80;10:body_byte=8'h00;11:body_byte=8'h01;
            12:body_byte=8'h88;13:body_byte=8'hb8;
            78:body_byte=crc16[15:8];79:body_byte=crc16[7:0];
            default:body_byte=(index>=14&&index<78)?saved[511:504]:8'd0;
        endcase
    end
    // A separate ownership flip-flop avoids state-decode logic on the
    // four-source arbiter -> existing ACK/NAV launch fanout path.
    assign ready=!active;
    // Predecode the wide shift enable one cycle ahead, as in the validated
    // camera/IMU pipeline. This removes state/decode fanout from 512 D inputs.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin saved<=0;shift_payload<=0;end
        else begin
            shift_payload<=state==BODY && index>=13 && index<77;
            if(state==IDLE)saved<=payload;
            else if(shift_payload)saved<={saved[503:0],8'd0};
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=IDLE;active<=0;index<=0;crc32<=32'hffffffff;crc16<=16'hffff;fcs<=0;gmii_data<=0;gmii_valid<=0;end
        else begin
            gmii_data<=0;gmii_valid<=0;
            case(state)
                IDLE:if(valid)begin active<=1;index<=0;crc32<=32'hffffffff;crc16<=16'hffff;state<=PREAMBLE;end
                PREAMBLE:begin gmii_valid<=1;gmii_data<=index==7?8'hd5:8'h55;if(index==7)begin index<=0;state<=BODY;end else index<=index+1'b1;end
                BODY:begin
                    gmii_valid<=1;gmii_data<=body_byte;
                    if(index!=0)crc32<=crc32_byte(crc32,gmii_data);
                    if(index>=14&&index<78)begin crc16<=crc16_byte(crc16,saved[511:504]);end
                    if(index==79)begin index<=0;state<=FCS;end else index<=index+1'b1;
                end
                FCS:begin
                    gmii_valid<=1;
                    if(index==0)begin gmii_data<=last_crc[7:0];fcs<=last_crc>>8;end else begin gmii_data<=fcs[7:0];fcs<=fcs>>8;end
                    if(index==3)begin index<=0;state<=GAP;end else index<=index+1'b1;
                end
                GAP:if(index==11)begin index<=0;state<=IDLE;active<=0;end else index<=index+1'b1;
                default:begin state<=IDLE;active<=0;end
            endcase
        end
    end
endmodule
module sensor_status_gmii(
    input wire source_clk,source_rst_n,sensor_valid,input wire [511:0] sensor_data,
    input wire tx_clk,tx_rst_n,tx_allow,
    output wire tx_pending,tx_busy,gmii_valid,output wire [7:0] gmii_data
);
    wire [511:0] network_data;wire network_valid,network_ready,sender_active;
    imu_snapshot_cdc mailbox(.src_clk(source_clk),.src_rst_n(source_rst_n),.src_valid(sensor_valid),
        .src_data(sensor_data),.src_ready(),.dst_clk(tx_clk),.dst_rst_n(tx_rst_n),
        .dst_ready(network_ready&&tx_allow),.dst_valid(network_valid),.dst_data(network_data));
    assign tx_pending=network_valid;assign tx_busy=sender_active;
    sensor_status_tx sender(.clk(tx_clk),.rst_n(tx_rst_n),.valid(network_valid&&tx_allow),
        .payload(network_data),.ready(network_ready),.active(sender_active),.gmii_data(gmii_data),.gmii_valid(gmii_valid));
endmodule
// Fixed priority on new frame starts: motor ACK/NAV, SNS1, IMU, camera.
// busy includes mandatory IFG, so an existing owner always finishes first.
module gmii_four_source_share(
    input camera_busy,camera_valid,input [7:0] camera_data,
    input imu_pending,imu_busy,imu_valid,input [7:0] imu_data,
    input motor_pending,motor_busy,motor_valid,input [7:0] motor_data,
    input sensor_pending,sensor_busy,sensor_valid,input [7:0] sensor_data,
    output camera_allow,imu_allow,motor_allow,sensor_allow,
    output [7:0] gmii_data,output gmii_valid
);
    assign motor_allow=!camera_busy&&!camera_valid&&!imu_busy&&!imu_valid&&!sensor_busy&&!sensor_valid;
    assign sensor_allow=!camera_busy&&!camera_valid&&!imu_busy&&!imu_valid&&!motor_pending&&!motor_busy&&!motor_valid;
    assign imu_allow=!camera_busy&&!camera_valid&&!motor_pending&&!motor_busy&&!motor_valid&&!sensor_pending&&!sensor_busy&&!sensor_valid;
    assign camera_allow=!imu_pending&&!imu_busy&&!imu_valid&&!motor_pending&&!motor_busy&&!motor_valid&&!sensor_pending&&!sensor_busy&&!sensor_valid;
    assign gmii_valid=camera_valid||imu_valid||motor_valid||sensor_valid;
    assign gmii_data=motor_valid?motor_data:sensor_valid?sensor_data:imu_valid?imu_data:camera_data;
endmodule
