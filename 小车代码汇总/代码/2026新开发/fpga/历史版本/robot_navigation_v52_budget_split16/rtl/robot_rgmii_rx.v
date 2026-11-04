`timescale 1ns/1ps
module robot_rgmii_rx (
    input wire reset_n, rgmii_rx_clk,
    input wire [3:0] rgmii_rxd,
    input wire rgmii_rx_ctl,
    output wire command_valid,
    output wire [127:0] command_data,
    output wire nav_valid,
    output wire [511:0] nav_data,
    output wire rgb_valid,
    output wire [15:0] rgb_epoch,
    output wire receive_reset_n,
    output wire [7:0] exec_gmii_data,
    output wire exec_gmii_valid,exec_gmii_error
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
    // Keep IDDR outputs on a short local path. Data, valid and error advance
    // together, so packet/FCS semantics are unchanged by this one-cycle stage.
    reg [7:0] gmii_data;
    reg gmii_valid,gmii_error;
    assign exec_gmii_data=gmii_data;
    assign exec_gmii_valid=gmii_valid;
    assign exec_gmii_error=gmii_error;
    always @(posedge rgmii_rx_clk or negedge receive_reset_n)
        if(!receive_reset_n)begin gmii_data<=0;gmii_valid<=0;gmii_error<=0;end
        else begin gmii_data<={q1,q0};gmii_valid<=ctl0;gmii_error<=ctl0^ctl1;end
    // RGB/RGP1 session commands were removed with the FPGA image preprocessor.
    // Keep the legacy ports for source compatibility; RAW camera transport does
    // not consume a session request and therefore never asserts these outputs.
    assign rgb_valid = 1'b0;
    assign rgb_epoch = 16'd0;
    motor_packet_rx parser(.clk(rgmii_rx_clk),.rst_n(receive_reset_n),
        .data(gmii_data),.valid(gmii_valid),.error(gmii_error),
        .command_valid(command_valid),.command_data(command_data),
        .accepted_frames(),.rejected_frames());
    // NAV3 owns planning; retained NAV2 observation ports are inert.
    assign nav_valid=1'b0;assign nav_data=512'b0;
endmodule
