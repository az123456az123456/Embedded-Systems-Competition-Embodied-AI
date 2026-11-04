`timescale 1ns/1ps
// Optional bench image: no camera or IMU. Same motor/RGMII pins and protocol.
module fmc_remote_motor_only_top (
    input wire clk50m,rgmii_rx_clk,
    input wire [3:0] rgmii_rxd,
    input wire rgmii_rx_ctl,
    output wire rgmii_tx_clk,rgmii_tx_ctl,
    output wire [3:0] rgmii_txd,
    output wire [3:0] motor_pwm,motor_in1,motor_in2,
    output wire [3:0] led
);
    reg [20:0] startup=0;
    always @(posedge clk50m)if(!startup[20])startup<=startup+1'b1;
    wire reset_n=startup[20];
    wire rx_valid,rx_reset_n,command_valid,status_valid;
    wire [127:0] rx_data,command_data,status_data;
    wire [3:0] flags;
    motor_rgmii_rx receiver(.reset_n(reset_n),.rgmii_rx_clk(rgmii_rx_clk),
        .rgmii_rxd(rgmii_rxd),.rgmii_rx_ctl(rgmii_rx_ctl),
        .command_valid(rx_valid),.command_data(rx_data),.receive_reset_n(rx_reset_n));
    motor_command_cdc command_mailbox(.src_clk(rgmii_rx_clk),.src_rst_n(rx_reset_n),
        .src_valid(rx_valid),.src_data(rx_data),.src_ready(),
        .dst_clk(clk50m),.dst_rst_n(reset_n),.dst_valid(command_valid),.dst_data(command_data));
    motor_remote_control motors(.clk(clk50m),.rst_n(reset_n),
        .command_valid(command_valid),.command_data(command_data),
        .motor_pwm(motor_pwm),.motor_in1(motor_in1),.motor_in2(motor_in2),
        .status_flags(flags),.status_valid(status_valid),.status_data(status_data),
        .last_sequence(),.applied_left(),.applied_right());
    wire [7:0] data;wire valid;
    motor_status_gmii status(.source_clk(clk50m),.source_rst_n(reset_n),
        .status_valid(status_valid),.status_data(status_data),
        .tx_clk(rgmii_rx_clk),.tx_rst_n(rx_reset_n),.tx_allow(1'b1),
        .tx_pending(),.tx_busy(),.gmii_valid(valid),.gmii_data(data));
    gmii_to_rgmii serializer(.reset_n(reset_n),.gmii_tx_clk(rgmii_rx_clk),
        .gmii_txd(data),.gmii_txen(valid),.gmii_txer(1'b0),
        .rgmii_tx_clk(rgmii_tx_clk),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_tx_ctl));
    reg [25:0] heartbeat=0;
    always @(posedge clk50m)heartbeat<=heartbeat+1'b1;
    assign led={flags[1],flags[0],flags[2],heartbeat[24]};
endmodule
