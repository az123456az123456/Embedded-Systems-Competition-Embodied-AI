`timescale 1ns/1ps
module imu_rgmii_top #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000
)(
    input wire clk_50m,
    inout wire imu_scl, imu_sda,
    output wire [3:0] led,
    input wire rgmii_rx_clk,
    output wire [3:0] rgmii_txd,
    output wire rgmii_tx_ctl, rgmii_tx_clk
);
    // Gowin supports initialized registers for power-on reset; no external
    // reset pin required. These counters also reset after reconfiguration.
    reg [7:0] por=0, net_por=0;
    always @(posedge clk_50m) if (!(&por)) por<=por+1'b1;
    always @(posedge rgmii_rx_clk) if (!(&net_por)) net_por<=net_por+1'b1;
    wire rst_n=&por;
    wire net_rst_n=&net_por;
    reg [31:0] ms_div, uptime_ms;
    reg ms_tick;
    reg [9:0] second_ms;
    reg heartbeat;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ms_div<=0; ms_tick<=0; uptime_ms<=0; second_ms<=0; heartbeat<=0;
        end else begin
            ms_tick<=0; heartbeat<=0;
            if (ms_div==CLK_HZ/1000-1) begin ms_div<=0; ms_tick<=1; end
            else ms_div<=ms_div+1'b1;
            if (ms_tick) begin
                uptime_ms<=uptime_ms+1'b1;
                if (second_ms==999) begin second_ms<=0; heartbeat<=1; end
                else second_ms<=second_ms+1'b1;
            end
        end
    end
    wire [6:0] address;
    wire [7:0] mpu_id, mag_id;
    wire mpu_init, mag_init, fault, overflow, ag_valid, mag_valid;
    wire [15:0] ax,ay,az,temp,gx,gy,gz,mx,my,mz;
    wire [23:0] asa;
    wire [31:0] errors;
    mpu9250_reader #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) sensor (
        .clk(clk_50m),.rst_n(rst_n),.ms_tick(ms_tick),.scl(imu_scl),.sda(imu_sda),
        .mpu_address(address),.mpu_id(mpu_id),.mag_id(mag_id),
        .mpu_initialized(mpu_init),.mag_initialized(mag_init),.fault(fault),
        .mag_overflow(overflow),.ag_valid(ag_valid),.mag_valid(mag_valid),
        .ax(ax),.ay(ay),.az(az),.temperature(temp),.gx(gx),.gy(gy),.gz(gz),
        .mx(mx),.my(my),.mz(mz),.asa(asa),.error_count(errors)
    );
    reg [31:0] sequence_ag, sequence_mag, sample_ms, drops;
    reg [15:0] age_ag, age_mag;
    reg seen_ag, seen_mag, publish;
    wire ag_fresh=seen_ag && mpu_init && age_ag<500;
    wire mag_fresh=seen_mag && mag_init && age_mag<500 && !overflow;
    wire [15:0] flags={10'b0,overflow,fault,mag_init,mpu_init,mag_fresh,ag_fresh};
    wire source_ready;
    wire [511:0] packet={32'h494d5539,8'd1,8'h92,flags,
        sequence_ag,sample_ms,ax,ay,az,temp,gx,gy,gz,mx,my,mz,
        asa,mpu_id,mag_id,1'b0,address,age_mag,errors,drops,
        uptime_ms,age_ag,16'd100,sequence_mag};
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sequence_ag<=0; sequence_mag<=0; sample_ms<=0; drops<=0;
            age_ag<=16'hffff; age_mag<=16'hffff; seen_ag<=0; seen_mag<=0; publish<=0;
        end else begin
            // One-cycle delay makes sequence / timestamp / age updates part of
            // the same snapshot as the sensor words.
            publish<=ag_valid || mag_valid || heartbeat;
            if (publish && !source_ready) drops<=drops+1'b1;
            if (ms_tick) begin
                if (age_ag!=16'hffff) age_ag<=age_ag+1'b1;
                if (age_mag!=16'hffff) age_mag<=age_mag+1'b1;
            end
            if (ag_valid) begin
                sequence_ag<=sequence_ag+1'b1; sample_ms<=uptime_ms; age_ag<=0; seen_ag<=1;
            end
            if (mag_valid) begin sequence_mag<=sequence_mag+1'b1; age_mag<=0; seen_mag<=1; end
        end
    end
    assign led[0]=(second_ms<500);
    assign led[1]=ag_fresh;
    assign led[2]=mag_fresh;
    assign led[3]=fault || overflow || (mpu_init && !ag_fresh) || (mag_init && !mag_fresh);
    wire [511:0] net_packet;
    wire net_valid, net_ready;
    imu_snapshot_cdc mailbox (
        .src_clk(clk_50m),.src_rst_n(rst_n),.src_valid(publish),.src_data(packet),
        .src_ready(source_ready),.dst_clk(rgmii_rx_clk),.dst_rst_n(net_rst_n),
        .dst_ready(net_ready),.dst_valid(net_valid),.dst_data(net_packet)
    );
    wire [7:0] gmii_data;
    wire gmii_valid;
    imu_ethernet_tx sender (
        .clk(rgmii_rx_clk),.rst_n(net_rst_n),.valid(net_valid),.payload(net_packet),
        .ready(net_ready),.gmii_data(gmii_data),.gmii_valid(gmii_valid)
    );
    genvar i;
    generate for (i=0;i<4;i=i+1) begin: ddr_data
        ODDR out_data (.Q0(rgmii_txd[i]),.Q1(),.D0(gmii_data[i]),
            .D1(gmii_data[i+4]),.TX(1'b1),.CLK(rgmii_rx_clk));
    end endgenerate
    ODDR out_ctl (.Q0(rgmii_tx_ctl),.Q1(),.D0(gmii_valid),.D1(gmii_valid),
        .TX(1'b1),.CLK(rgmii_rx_clk));
    ODDR out_clk (.Q0(rgmii_tx_clk),.Q1(),.D0(1'b1),.D1(1'b0),
        .TX(1'b1),.CLK(rgmii_rx_clk));
endmodule
