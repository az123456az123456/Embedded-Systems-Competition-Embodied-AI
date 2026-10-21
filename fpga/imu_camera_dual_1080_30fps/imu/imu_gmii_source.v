`timescale 1ns/1ps
module imu_gmii_source #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000
)(
    input wire clk_50m,
    inout wire imu_scl, imu_sda,
    output wire [3:0] led,
    input wire tx_clk, reset_n, tx_allow,
    output wire tx_pending, tx_busy,
    output wire [7:0] gmii_data,
    output wire gmii_valid
);
    // Common asynchronous assertion on PLL loss, synchronous release per domain.
    (* syn_async_reg=1 *) reg [1:0] src_reset=0,net_reset=0;
    always @(posedge clk_50m or negedge reset_n)
        if(!reset_n) src_reset<=0; else src_reset<={src_reset[0],1'b1};
    always @(posedge tx_clk or negedge reset_n)
        if(!reset_n) net_reset<=0; else net_reset<={net_reset[0],1'b1};
    wire rst_n=src_reset[1];
    wire net_rst_n=net_reset[1];
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
    wire mpu_init, mag_init, fault, overflow, ag_valid, mag_valid, raw_ag_valid;
    wire [15:0] raw_ax,raw_ay,raw_az,raw_temp,raw_gx,raw_gy,raw_gz;
    wire [15:0] ax,ay,az,temp,gx,gy,gz,mx,my,mz;
    wire [23:0] asa;
    wire [31:0] errors;
    mpu9250_reader #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) sensor (
        .clk(clk_50m),.rst_n(rst_n),.ms_tick(ms_tick),.scl(imu_scl),.sda(imu_sda),
        .mpu_address(address),.mpu_id(mpu_id),.mag_id(mag_id),
        .mpu_initialized(mpu_init),.mag_initialized(mag_init),.fault(fault),
        .mag_overflow(overflow),.ag_valid(raw_ag_valid),.mag_valid(mag_valid),
        .ax(raw_ax),.ay(raw_ay),.az(raw_az),.temperature(raw_temp),.gx(raw_gx),.gy(raw_gy),.gz(raw_gz),
        .mx(mx),.my(my),.mz(mz),.asa(asa),.error_count(errors)
    );
    // Filter once per actual new sensor sample, before packet snapshot/CDC.
    // All clients receive identical filtered physical codes and original units.
    imu_sample_filter filter (
        .clk(clk_50m),.rst_n(rst_n),.clear(!mpu_init),.in_valid(raw_ag_valid),
        .ax_in(raw_ax),.ay_in(raw_ay),.az_in(raw_az),.temp_in(raw_temp),
        .gx_in(raw_gx),.gy_in(raw_gy),.gz_in(raw_gz),
        .out_valid(ag_valid),.ax_out(ax),.ay_out(ay),.az_out(az),.temp_out(temp),
        .gx_out(gx),.gy_out(gy),.gz_out(gz)
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
        .src_ready(source_ready),.dst_clk(tx_clk),.dst_rst_n(net_rst_n),
        .dst_ready(net_ready && tx_allow),.dst_valid(net_valid),.dst_data(net_packet)
    );
    assign tx_pending=net_valid;
    assign tx_busy=!net_ready;
    imu_ethernet_tx sender (
        .clk(tx_clk),.rst_n(net_rst_n),.valid(net_valid && tx_allow),.payload(net_packet),
        .ready(net_ready),.gmii_data(gmii_data),.gmii_valid(gmii_valid)
    );
endmodule
