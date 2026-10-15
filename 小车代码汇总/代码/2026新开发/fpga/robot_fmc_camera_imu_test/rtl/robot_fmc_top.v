// ACG138 compatibility wrapper. Original example core retained.
module robot_fmc_top(
    inout imu_scl,imu_sda,
    output [3:0] motor_pwm,motor_in1,motor_in2,
    output [3:0] led,
    input clk50m,
    output uart_tx,
    output rgmii_tx_clk,
    output [3:0] rgmii_txd,
    output rgmii_tx_ctl,
    output cam1_pwdn,
    output cam1_rst_n,
    output cam1_xclk,
    output cam1_scl,
    inout cam1_sda,
    input cam1_pclk,
    input cam1_href,
    input cam1_vsync,
    input [7:0] cam1_data,
    output cam2_pwdn,
    output cam2_rst_n,
    output cam2_xclk,
    output cam2_scl,
    inout cam2_sda,
    input cam2_pclk,
    input cam2_href,
    input cam2_vsync,
    input [7:0] cam2_data
);
reg [20:0] startup = 0;
always @(posedge clk50m) if (!startup[20]) startup <= startup + 1'b1;
wire reset_n = startup[20];
wire core_reset1, core_reset2;
wire init_done;
wire [15:0] af_status1,af_status2;
wire [2:0] init_status;
assign cam1_rst_n = reset_n & core_reset1;
assign cam2_rst_n = reset_n & core_reset2;
wire tx_clk,diag_clk,pll_ready,cam_busy,cam_allow,cam_valid;
wire [7:0] cam_data,imu_data,gmii_data;
wire imu_pending,imu_busy,imu_allow,imu_valid,gmii_valid;
camera_dual_gmii core (
.tx_allow(cam_allow),.tx_busy(cam_busy),.tx_clk(tx_clk),.tx_data(cam_data),.tx_valid(cam_valid),.pll_ready(pll_ready),.diag_clk(diag_clk),
.af_status1(af_status1),.af_status2(af_status2),
.camera_status(init_status),.clk(clk50m),.rst_n(reset_n),.eth_gtxc(),.eth_txd(),.eth_txen(),
.eth_mdc(),.eth_mdio(),.eth_rst_n(),.led(init_done),
.camera1_sclk(cam1_scl),.camera1_sdat(cam1_sda),.camera1_pclk(cam1_pclk),.camera1_href(cam1_href),.camera1_vsync(cam1_vsync),.camera1_data(cam1_data),.camera1_xclk(cam1_xclk),.camera1_pwdn(cam1_pwdn),.camera1_rst_n(core_reset1),
.camera2_sclk(cam2_scl),.camera2_sdat(cam2_sda),.camera2_pclk(cam2_pclk),.camera2_href(cam2_href),.camera2_vsync(cam2_vsync),.camera2_data(cam2_data),.camera2_xclk(cam2_xclk),.camera2_pwdn(cam2_pwdn),.camera2_rst_n(core_reset2)
);
camera_diag diag(.af1(af_status1),.af2(af_status2),.clk(diag_clk),.reset_n(reset_n),.init_done(init_status),
.pclk1(cam1_pclk),.href1(cam1_href),.vsync1(cam1_vsync),
.pclk2(cam2_pclk),.href2(cam2_href),.vsync2(cam2_vsync),.uart_tx(uart_tx));
imu_gmii_source imu (
    .clk_50m(clk50m),.tx_clk(tx_clk),.reset_n(reset_n && pll_ready),
    .imu_scl(imu_scl),.imu_sda(imu_sda),.led(led),
    .tx_allow(imu_allow),.tx_pending(imu_pending),.tx_busy(imu_busy),
    .gmii_data(imu_data),.gmii_valid(imu_valid)
);
gmii_frame_share share (
    .camera_busy(cam_busy),.camera_valid(cam_valid),.camera_data(cam_data),
    .imu_pending(imu_pending),.imu_busy(imu_busy),.imu_valid(imu_valid),.imu_data(imu_data),
    .camera_allow(cam_allow),.imu_allow(imu_allow),.gmii_data(gmii_data),.gmii_valid(gmii_valid)
);
gmii_to_rgmii serializer (
    .reset_n(reset_n && pll_ready),.gmii_tx_clk(tx_clk),.gmii_txd(gmii_data),
    .gmii_txen(gmii_valid),.gmii_txer(1'b0),
    .rgmii_tx_clk(rgmii_tx_clk),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_tx_ctl)
);
// Automatic motor bench test; external TB6612 STBY remains high.
four_motor_ramp_test motors (
    .clk(clk50m),.rst_n(reset_n),.motor_pwm(motor_pwm),
    .motor_in1(motor_in1),.motor_in2(motor_in2),
    .busy(),.done(),.reverse_dir(),.selected(),.duty_percent()
);
endmodule
