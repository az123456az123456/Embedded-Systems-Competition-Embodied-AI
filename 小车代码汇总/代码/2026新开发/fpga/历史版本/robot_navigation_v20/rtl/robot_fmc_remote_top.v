// ACG138 compatibility wrapper. Original example core retained.
module robot_fmc_remote_top(
    inout imu_scl,imu_sda,
    input [3:0] encoder_a,encoder_b,
    input [2:0] sonar_rx,
    output [3:0] motor_pwm,motor_in1,motor_in2,
    output [3:0] led,
    input clk50m,
    output uart_tx,
    input rgmii_rx_clk,
    input [3:0] rgmii_rxd,
    input rgmii_rx_ctl,
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
wire imu_monitor_valid;
wire [511:0] imu_monitor_packet;
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
    .gmii_data(imu_data),.gmii_valid(imu_valid),
    .monitor_valid(imu_monitor_valid),.monitor_packet(imu_monitor_packet)
);
wire ack_pending,ack_busy,ack_valid,ack_allow;
wire [7:0] ack_data;
wire sensor_pending,sensor_busy,sensor_tx_valid,sensor_allow;
wire [7:0] sensor_tx_data;
wire sonar_pending,sonar_busy,sonar_valid,sonar_allow;
wire [7:0] sonar_data;
wire plan_pending,plan_busy,plan_valid,plan_allow;
wire [7:0] plan_data;
// Share the old sensor slot. Sonar packets retain priority at frame boundaries.
assign sensor_pending=sonar_pending || plan_pending;
assign sensor_busy=sonar_busy || plan_busy;
assign sensor_tx_valid=sonar_valid || plan_valid;
assign sensor_tx_data=sonar_valid?sonar_data:plan_data;
wire sensor_sample_valid;
wire [511:0] sensor_sample_packet;
robot_sensor_capture sensor_capture(.clk(clk50m),.rst_n(reset_n),
    .encoder_a(encoder_a),.encoder_b(encoder_b),.sonar_rx(sonar_rx),
    .sensor_valid(sensor_sample_valid),.sensor_packet(sensor_sample_packet));
gmii_navigation_share share (.clk(tx_clk),.rst_n(ack_network_reset[1]),
    .camera_busy(cam_busy),.camera_valid(cam_valid),.camera_data(cam_data),
    .imu_pending(imu_pending),.imu_busy(imu_busy),.imu_valid(imu_valid),.imu_data(imu_data),
    .motor_pending(ack_pending),.motor_busy(ack_busy),.motor_valid(ack_valid),.motor_data(ack_data),
    .sensor_pending(sonar_pending),.sensor_busy(sonar_busy),.sensor_valid(sonar_valid),.sensor_data(sonar_data),
    .planner_pending(plan_pending),.planner_busy(plan_busy),.planner_valid(plan_valid),.planner_data(plan_data),
    .planner_allow(plan_allow),.sensor_allow(sonar_allow),.camera_allow(cam_allow),.imu_allow(imu_allow),.motor_allow(ack_allow),.gmii_data(gmii_data),.gmii_valid(gmii_valid)
);
gmii_to_rgmii serializer (
    .reset_n(reset_n && pll_ready),.gmii_tx_clk(tx_clk),.gmii_txd(gmii_data),
    .gmii_txen(gmii_valid),.gmii_txer(1'b0),
    .rgmii_tx_clk(rgmii_tx_clk),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_tx_ctl)
);
// New commands are accepted only after an unarmed zero handshake.
wire rx_command_valid,rx_reset_n,command_valid,command_ready;
wire [127:0] rx_command_data,command_data,status_data;
wire motor_status_valid;
wire [3:0] motor_flags;
wire [207:0] nav_status;
wire nav_trap;
wire nav_rx_valid,nav_request_valid,nav_request_ready,nav_reply_valid,nav_reply_ready;
wire [511:0] nav_rx_data,nav_request_data,nav_reply_data;
robot_nav_soc nav_cpu(.clk(clk50m),.reset_n(reset_n),
    .imu_valid(imu_monitor_valid),.imu_packet(imu_monitor_packet),
    .motor_valid(motor_status_valid),.motor_packet(status_data),
    .sensor_valid(sensor_sample_valid),.sensor_packet(sensor_sample_packet),
    .nav_request_valid(nav_request_valid),.nav_request_packet(nav_request_data),.nav_request_ready(nav_request_ready),
    .nav_reply_valid(nav_reply_valid),.nav_reply_packet(nav_reply_data),.nav_reply_ready(nav_reply_ready),
    .nav_status(nav_status),.cpu_trap(nav_trap));
robot_rgmii_rx receiver(.reset_n(reset_n),.rgmii_rx_clk(rgmii_rx_clk),
    .rgmii_rxd(rgmii_rxd),.rgmii_rx_ctl(rgmii_rx_ctl),
    .command_valid(rx_command_valid),.command_data(rx_command_data),.receive_reset_n(rx_reset_n),
    .nav_valid(nav_rx_valid),.nav_data(nav_rx_data));
imu_snapshot_cdc #(.CAPTURE_WHEN_IDLE(1)) nav_request_mailbox(.src_clk(rgmii_rx_clk),.src_rst_n(rx_reset_n),
    .src_valid(nav_rx_valid),.src_data(nav_rx_data),.src_ready(),
    .dst_clk(clk50m),.dst_rst_n(reset_n),.dst_ready(nav_request_ready),
    .dst_valid(nav_request_valid),.dst_data(nav_request_data));
motor_command_cdc command_mailbox(.src_clk(rgmii_rx_clk),.src_rst_n(rx_reset_n),
    .src_valid(rx_command_valid),.src_data(rx_command_data),.src_ready(command_ready),
    .dst_clk(clk50m),.dst_rst_n(reset_n),.dst_valid(command_valid),.dst_data(command_data));
motor_remote_control #(.ENCODER_SYNC_ENABLE(1)) motors(.clk(clk50m),.rst_n(reset_n),
    .command_valid(command_valid),.command_data(command_data),
    .encoder_valid(sensor_sample_valid && sensor_sample_packet[31:16]==16'd50),
    .encoder_deltas({sensor_sample_packet[159:128],sensor_sample_packet[191:160],
                     sensor_sample_packet[223:192],sensor_sample_packet[255:224]}),
    .encoder_illegal(sensor_sample_packet[11:8]),
    .motor_pwm(motor_pwm),.motor_in1(motor_in1),.motor_in2(motor_in2),
    .status_flags(motor_flags),.status_valid(motor_status_valid),.status_data(status_data),
    .last_sequence(),.applied_left(),.applied_right());
wire ack_reset=reset_n && pll_ready;
(* syn_async_reg=1 *) reg [1:0] ack_source_reset=0,ack_network_reset=0;
always @(posedge clk50m or negedge ack_reset)
    if(!ack_reset)ack_source_reset<=0;else ack_source_reset<={ack_source_reset[0],1'b1};
always @(posedge tx_clk or negedge ack_reset)
    if(!ack_reset)ack_network_reset<=0;else ack_network_reset<={ack_network_reset[0],1'b1};
motor_status_gmii #(.NAV_ENABLE(1)) motor_status(.source_clk(clk50m),.source_rst_n(ack_source_reset[1]),
    .status_valid(motor_status_valid),.status_data(status_data),.nav_data(nav_status),.tx_clk(tx_clk),
    .tx_rst_n(ack_network_reset[1]),.tx_allow(ack_allow),.tx_pending(ack_pending),
    .tx_busy(ack_busy),.gmii_valid(ack_valid),.gmii_data(ack_data));
sensor_status_gmii sensor_network(.source_clk(clk50m),.source_rst_n(ack_source_reset[1]),
    .sensor_valid(sensor_sample_valid),.sensor_data(sensor_sample_packet),.tx_clk(tx_clk),
    .tx_rst_n(ack_network_reset[1]),.tx_allow(sonar_allow),.tx_pending(sonar_pending),
    .tx_busy(sonar_busy),.gmii_valid(sonar_valid),.gmii_data(sonar_data));
nav_message_gmii planner_network(.source_clk(clk50m),.source_rst_n(ack_source_reset[1]),
    .message_valid(nav_reply_valid),.message_data(nav_reply_data),.message_ready(nav_reply_ready),
    .tx_clk(tx_clk),.tx_rst_n(ack_network_reset[1]),.tx_allow(plan_allow),.tx_pending(plan_pending),
    .tx_busy(plan_busy),.gmii_valid(plan_valid),.gmii_data(plan_data));
endmodule
