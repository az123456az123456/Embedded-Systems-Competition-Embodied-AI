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
wire tx_clk,pll_ready,cam_busy,cam_allow,cam_valid;
(* syn_async_reg=1 *) reg [1:0] ack_source_reset=0,ack_network_reset=0;
wire rx_reset_n;
wire [7:0] cam_data,imu_data,gmii_data;
wire imu_pending,imu_busy,imu_allow,imu_valid,gmii_valid;
wire imu_monitor_valid;
wire [511:0] imu_monitor_packet;
camera_dual_gmii core (
.tx_allow(cam_allow),.tx_busy(cam_busy),.tx_clk(tx_clk),.tx_data(cam_data),.tx_valid(cam_valid),.pll_ready(pll_ready),
.af_status1(af_status1),.af_status2(af_status2),
.camera_status(init_status),.clk(clk50m),.rst_n(reset_n),.eth_gtxc(),.eth_txd(),.eth_txen(),
.eth_mdc(),.eth_mdio(),.eth_rst_n(),.led(init_done),
.camera1_sclk(cam1_scl),.camera1_sdat(cam1_sda),.camera1_pclk(cam1_pclk),.camera1_href(cam1_href),.camera1_vsync(cam1_vsync),.camera1_data(cam1_data),.camera1_xclk(cam1_xclk),.camera1_pwdn(cam1_pwdn),.camera1_rst_n(core_reset1),
.camera2_sclk(cam2_scl),.camera2_sdat(cam2_sda),.camera2_pclk(cam2_pclk),.camera2_href(cam2_href),.camera2_vsync(cam2_vsync),.camera2_data(cam2_data),.camera2_xclk(cam2_xclk),.camera2_pwdn(cam2_pwdn),.camera2_rst_n(core_reset2)
);
// Camera text diagnostics are non-safety telemetry and are consumed on RK.
// Keep the legacy UART port at its idle level without instantiating the
// formatter/serializer in the FPGA fabric.
assign uart_tx = 1'b1;
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
wire [127:0] local_encoder_positions;
wire [3:0] local_encoder_illegal;
wire [47:0] local_sonar_mm,local_filter_mm;
wire [95:0] local_sonar_seq,local_sonar_stamp,local_filter_seq;
wire [2:0] local_sonar_online,local_filter_valid;
wire [63:0] local_control_pwm;
wire local_control_ready;
wire [4:0] local_control_flags;
wire [31:0] local_control_epoch,local_control_sequence,local_control_motion;
wire [7:0] local_control_masks;
wire [127:0] local_control_diagnostics;
robot_sensor_capture #(.CPU_FILTER_ENABLE(1)) sensor_capture(.clk(clk50m),.rst_n(reset_n),
    .encoder_a(encoder_a),.encoder_b(encoder_b),.sonar_rx(sonar_rx),
    .sensor_valid(sensor_sample_valid),.sensor_packet(sensor_sample_packet),
    .filter_mm(local_filter_mm),.filter_seq(local_filter_seq),.filter_valid(local_filter_valid),.filter_ready(local_control_ready),
    .encoder_positions(local_encoder_positions),.encoder_illegal(local_encoder_illegal),
    .raw_sonar_mm(local_sonar_mm),.raw_sonar_seq(local_sonar_seq),.raw_sonar_stamp(local_sonar_stamp),.raw_sonar_online(local_sonar_online));
robot_gmii_share share (.clk(tx_clk),.rst_n(ack_network_reset[1]),
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
wire rx_command_valid,command_valid,command_ready;
wire [127:0] rx_command_data,command_data,status_data;
wire motor_status_valid;
wire [3:0] motor_flags;
wire [207:0] nav_status;
wire nav_trap;
wire exec_bus_req,exec_bus_rsp_valid;
wire [11:0] exec_bus_offset;
wire [31:0] exec_bus_wdata,exec_bus_rsp_data,exec_device_ms;
wire [3:0] exec_bus_wstrb;
wire [7:0] exec_rx_data;
wire exec_rx_valid,exec_rx_error;
wire exec_owned,exec_live,accepted_external_mot,manual_quiet;
wire [63:0] exec_pwm,exec_epoch;
wire [15:0] exec_linear,exec_angular;
wire [31:0] exec_generation;
// Status ACK and its attached NAV1 use one coherent, registered snapshot.
// Only read-only telemetry consumers use this stage; all control feedback is direct.
wire buffered_motor_status_valid;
wire [127:0] buffered_motor_status_data;
wire [207:0] buffered_motor_nav_data;
motor_status_align status_align(.clk(clk50m),.reset_n(reset_n),
    .in_valid(motor_status_valid),.in_status(status_data),.in_nav(nav_status),
    .out_valid(buffered_motor_status_valid),.out_status(buffered_motor_status_data),.out_nav(buffered_motor_nav_data));
robot_nav_soc nav_cpu(.clk(clk50m),.reset_n(reset_n),
    .imu_valid(imu_monitor_valid),.imu_packet(imu_monitor_packet),
    .motor_valid(buffered_motor_status_valid),.motor_packet(buffered_motor_status_data),
    .sensor_valid(sensor_sample_valid),.sensor_packet(sensor_sample_packet),
    // NAV2 planning remains in the retained rollback project. No old request
    // producer or old executor runs alongside this NAV3 owner.
    .nav_request_valid(1'b0),.nav_request_packet(512'b0),.nav_request_ready(),
    .nav_reply_valid(),.nav_reply_packet(),.nav_reply_ready(1'b1),
    .nav_status(nav_status),.cpu_trap(nav_trap),
    .control_flags(local_control_flags),.control_epoch(local_control_epoch),.control_sequence(local_control_sequence),
    .control_motion(local_control_motion),.control_masks(local_control_masks),
    .encoder_positions(local_encoder_positions),.encoder_illegal(local_encoder_illegal),
    .sonar_mm(local_sonar_mm),.sonar_seq(local_sonar_seq),.sonar_stamp(local_sonar_stamp),.sonar_online(local_sonar_online),
    .control_pwm(local_control_pwm),.control_ready(local_control_ready),
    .filter_mm(local_filter_mm),.filter_seq(local_filter_seq),.filter_valid(local_filter_valid),.control_diagnostics(local_control_diagnostics),
    .exec_bus_req(exec_bus_req),.exec_bus_offset(exec_bus_offset),.exec_bus_wdata(exec_bus_wdata),.exec_bus_wstrb(exec_bus_wstrb),
    .exec_bus_rsp_valid(exec_bus_rsp_valid),.exec_bus_rsp_data(exec_bus_rsp_data),.device_ms(exec_device_ms));
robot_rgmii_rx receiver(.reset_n(reset_n),.rgmii_rx_clk(rgmii_rx_clk),
    .rgmii_rxd(rgmii_rxd),.rgmii_rx_ctl(rgmii_rx_ctl),
    .command_valid(rx_command_valid),.command_data(rx_command_data),.receive_reset_n(rx_reset_n),
    .nav_valid(),.nav_data(),.rgb_valid(),.rgb_epoch(),
    .exec_gmii_data(exec_rx_data),.exec_gmii_valid(exec_rx_valid),.exec_gmii_error(exec_rx_error));
nav_exec_io navigation_io(
    .cpu_clk(clk50m),.cpu_reset_n(reset_n),.rx_clk(rgmii_rx_clk),.rx_reset_n(rx_reset_n),
    .tx_clk(tx_clk),.tx_reset_n(ack_network_reset[1]),.device_ms(exec_device_ms),
    .rx_data(exec_rx_data),.rx_valid(exec_rx_valid),.rx_error(exec_rx_error),
    .bus_req(exec_bus_req),.bus_offset(exec_bus_offset),.bus_wdata(exec_bus_wdata),.bus_wstrb(exec_bus_wstrb),
    .bus_rsp_valid(exec_bus_rsp_valid),.bus_rsp_data(exec_bus_rsp_data),
    .cpu_trap(nav_trap),.mot_accept(accepted_external_mot),.manual_quiet(manual_quiet),
    .raw_sonar_mm(local_sonar_mm),.raw_sonar_online(local_sonar_online),
    .nav_owned(exec_owned),.nav_live(exec_live),.nav_pwm(exec_pwm),.nav_linear(exec_linear),.nav_angular(exec_angular),
    .nav_epoch(exec_epoch),.context_generation(exec_generation),
    .tx_allow(plan_allow),.tx_pending(plan_pending),.tx_busy(plan_busy),.gmii_valid(plan_valid),.gmii_data(plan_data),.reserved_access());
motor_command_cdc command_mailbox(.src_clk(rgmii_rx_clk),.src_rst_n(rx_reset_n),
    .src_valid(rx_command_valid),.src_data(rx_command_data),.src_ready(command_ready),
    .dst_clk(clk50m),.dst_rst_n(reset_n),.dst_valid(command_valid),.dst_data(command_data));
motor_remote_control #(.ENCODER_SYNC_ENABLE(1),.NAV_EXEC_ENABLE(1)) motors(.clk(clk50m),.rst_n(reset_n),
    .command_valid(command_valid),.command_data(command_data),
    .encoder_valid(sensor_sample_valid && sensor_sample_packet[31:16]==16'd50),
    .encoder_deltas({sensor_sample_packet[159:128],sensor_sample_packet[191:160],
                     sensor_sample_packet[223:192],sensor_sample_packet[255:224]}),
    .encoder_illegal(sensor_sample_packet[11:8]),
    .motor_pwm(motor_pwm),.motor_in1(motor_in1),.motor_in2(motor_in2),
    .status_flags(motor_flags),.status_valid(motor_status_valid),.status_data(status_data),
    .last_sequence(),.applied_left(),.applied_right(),
    .cpu_pwm(local_control_pwm),.cpu_ready(local_control_ready),.cpu_trap(nav_trap),
    .raw_sonar_mm(local_sonar_mm),.raw_sonar_online(local_sonar_online),
    .control_flags(local_control_flags),.control_epoch(local_control_epoch),.control_sequence(local_control_sequence),
    .control_motion(local_control_motion),.control_masks(local_control_masks),
    .nav_owned(exec_owned),.nav_live(exec_live),.nav_pwm(exec_pwm),.nav_linear(exec_linear),.nav_angular(exec_angular),
    .accepted_external_mot(accepted_external_mot),.manual_quiet(manual_quiet));
wire ack_reset=reset_n && pll_ready;
always @(posedge clk50m or negedge ack_reset)
    if(!ack_reset)ack_source_reset<=0;else ack_source_reset<={ack_source_reset[0],1'b1};
always @(posedge tx_clk or negedge ack_reset)
    if(!ack_reset)ack_network_reset<=0;else ack_network_reset<={ack_network_reset[0],1'b1};
motor_status_gmii #(.NAV_ENABLE(1)) motor_status(.source_clk(clk50m),.source_rst_n(ack_source_reset[1]),
    .status_valid(buffered_motor_status_valid),.status_data(buffered_motor_status_data),.nav_data(buffered_motor_nav_data),.tx_clk(tx_clk),
    .tx_rst_n(ack_network_reset[1]),.tx_allow(ack_allow),.tx_pending(ack_pending),
    .tx_busy(ack_busy),.gmii_valid(ack_valid),.gmii_data(ack_data));
sensor_status_gmii sensor_network(.source_clk(clk50m),.source_rst_n(ack_source_reset[1]),
    .sensor_valid(sensor_sample_valid),.sensor_data(sensor_sample_packet),.tx_clk(tx_clk),
    .tx_rst_n(ack_network_reset[1]),.tx_allow(sonar_allow),.tx_pending(sonar_pending),
    .tx_busy(sonar_busy),.gmii_valid(sonar_valid),.gmii_data(sonar_data));
endmodule
