create_clock -name clk50m -period 20.000 [get_ports {clk50m}]
create_clock -name cam1_pclk -period 10.000 [get_ports {cam1_pclk}]
create_clock -name cam2_pclk -period 10.000 [get_ports {cam2_pclk}]
// Preliminary clocks only; external I/O timing is not signed off.

# Camera PLL supplies 125MHz for BOTH packet sources and the single ODDR.
# Retain source camera constraints; external I/O and CDC require STA review.
# IMU coherent mailbox: request crosses two synchronizer stages before data
# capture; held payload remains constant until the synchronized ack returns.
# Bound this bundled-data path rather than cutting the entire 50/125MHz domains.
set_max_delay 12.000 -from [get_pins {imu/mailbox/held*/Q}] -to [get_pins {imu/sender/saved*/D}]
set_min_delay 0.000 -from [get_pins {imu/mailbox/held*/Q}] -to [get_pins {imu/sender/saved*/D}]
set_false_path -to [get_pins {imu/mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {imu/mailbox/ack_sync_0_s0/D}]

# Additional motor receiver. Board-tested RX skew is supplied by RK GMAC delay.
create_clock -name rgmii_rx_clk -period 8.000 [get_ports {rgmii_rx_clk}]
# Coherent motor command mailbox, two request stages before capture.
set_max_delay 12.000 -from [get_pins {command_mailbox/held*/Q}] -to [get_pins {command_mailbox/dst_data*/D}]
set_min_delay 0.000 -from [get_pins {command_mailbox/held*/Q}] -to [get_pins {command_mailbox/dst_data*/D}]
set_false_path -to [get_pins {command_mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {command_mailbox/ack_sync_0_s0/D}]
set_max_delay 12.000 -from [get_pins {motor_status/mailbox/held*/Q}] -to [get_pins {motor_status/sender/saved*/D}]
set_min_delay 0.000 -from [get_pins {motor_status/mailbox/held*/Q}] -to [get_pins {motor_status/sender/saved*/D}]
set_false_path -to [get_pins {motor_status/mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {motor_status/mailbox/ack_sync_0_s0/D}]

# Asynchronous handshake data is held until the returning two-stage ACK.
# Capture occurs only after the synchronized request; same-edge hold checks
# between unrelated clocks do not model this path. Keep the 12ns max bound.
set_false_path -hold -from [get_pins {command_mailbox/held*/Q}] -to [get_pins {command_mailbox/dst_data*/D}]
set_false_path -hold -from [get_pins {motor_status/mailbox/held*/Q}] -to [get_pins {motor_status/sender/saved*/D}]
# Only the dedicated asynchronous-assert / synchronous-release reset chain.
set_false_path -to [get_pins {receiver/reset_sync*/CLEAR}]
set_false_path -to [get_pins {ack_source_reset*/CLEAR}]
set_false_path -to [get_pins {ack_network_reset*/CLEAR}]

# The dedicated reset-release synchronizers intentionally accept async assertion.
set_false_path -to [get_pins {core/*reset/release_sync*/CLEAR}]
set_false_path -to [get_pins {core/pclk1_init_sync_0_s0/D}]
set_false_path -to [get_pins {core/pclk2_init_sync_0_s0/D}]
# Inspect every negative endpoint instead of just the default 25 paths.
report_timing -setup -max_paths 1000
report_timing -recovery -max_paths 1000
report_timing -hold -max_paths 1000
report_timing -removal -max_paths 1000



# v14 SNS1 uses the same held-until-ACK bundled-data mailbox as IMU/ACK.
# Only first request/ack synchronizer stages are asynchronous. Bound all held
# data propagation to 12ns; never cut the full 50/125MHz domains.
set_max_delay 12.000 -from [get_pins {sensor_network/mailbox/held*/Q}] -to [get_pins {sensor_network/sender/saved*/D}]
set_min_delay 0.000 -from [get_pins {sensor_network/mailbox/held*/Q}] -to [get_pins {sensor_network/sender/saved*/D}]
set_false_path -hold -from [get_pins {sensor_network/mailbox/held*/Q}] -to [get_pins {sensor_network/sender/saved*/D}]
set_false_path -to [get_pins {sensor_network/mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {sensor_network/mailbox/ack_sync_0_s0/D}]

# v16 NAV2: same held-until-returning-ACK handshake, not arbitrary bus cuts.
# Source payload is stable before the two request synchronizers reach capture.
set_max_delay 12.000 -from [get_pins {planner_network/mailbox/held*/Q}] -to [get_pins {planner_network/sender/saved*/D}]
set_min_delay 0.000 -from [get_pins {planner_network/mailbox/held*/Q}] -to [get_pins {planner_network/sender/saved*/D}]
set_false_path -hold -from [get_pins {planner_network/mailbox/held*/Q}] -to [get_pins {planner_network/sender/saved*/D}]
set_false_path -to [get_pins {planner_network/mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {planner_network/mailbox/ack_sync_0_s0/D}]
set_max_delay 12.000 -from [get_pins {nav_request_mailbox/held*/Q}] -to [get_pins {nav_cpu/request_packet*/D}]
set_min_delay 0.000 -from [get_pins {nav_request_mailbox/held*/Q}] -to [get_pins {nav_cpu/request_packet*/D}]
set_false_path -hold -from [get_pins {nav_request_mailbox/held*/Q}] -to [get_pins {nav_cpu/request_packet*/D}]
set_false_path -to [get_pins {nav_request_mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {nav_request_mailbox/ack_sync_0_s0/D}]
report_timing -setup -from [get_clocks {rgmii_rx_clk}] -to [get_clocks {rgmii_rx_clk}] -max_paths 1000
report_timing -setup -from [get_pins {motor_status/sender/*/Q}] -to [get_pins {motor_status/sender/*/D}] -max_paths 1000

# v35 RGB FIFOs: registered Gray pointer crossings, bounded within one TX
# period. Only local asynchronous reset-release chains are excluded.
# The write Gray MSB aliases wbin[11] after synthesis.
# The pipelined read pointer retains rgray[11], already covered by rgray*.
set_max_delay 8.000 -from [get_pins {core/fifo*/wgray*/Q}] -to [get_pins {core/fifo*/rq1_wgray*/D}]
set_max_delay 8.000 -from [get_pins {core/fifo*/rgray*/Q}] -to [get_pins {core/fifo*/wq1_rgray*/D}]
set_max_delay 8.000 -from [get_pins {core/fifo*/wbin_11*/Q}] -to [get_pins {core/fifo*/rq1_wgray_11*/D}]
set_false_path -to [get_pins {core/fifo*/wr_reset_sync*/CLEAR}]
set_false_path -to [get_pins {core/fifo*/rd_reset_sync*/CLEAR}]
report_timing -setup -from [get_pins {core/fifo*/wgray*/Q}] -to [get_pins {core/fifo*/rq1_wgray*/D}] -max_paths 1000
report_timing -setup -from [get_pins {core/fifo*/rgray*/Q}] -to [get_pins {core/fifo*/wq1_rgray*/D}] -max_paths 1000
report_timing -setup -from [get_pins {core/fifo*/wbin_11*/Q}] -to [get_pins {core/fifo*/rq1_wgray_11*/D}] -max_paths 1000

# v35 coherent session request and per-camera mailboxes.
# Only the local sampling register receives the held async epoch bus;
# epoch compare and abort fanout are fully registered in the camera domain.
set_max_delay 12.000 -from [get_pins {rgb_request_mailbox/held*/Q}] -to [get_pins {core/pending_epoch*/D}]
set_false_path -hold -from [get_pins {rgb_request_mailbox/held*/Q}] -to [get_pins {core/pending_epoch*/D}]
set_false_path -to [get_pins {rgb_request_mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {rgb_request_mailbox/ack_sync_0_s0/D}]
set_max_delay 12.000 -from [get_pins {core/rgb_session1_mailbox/held*/Q}] -to [get_pins {core/bridge1/sampled_session_epoch*/D}]
set_false_path -hold -from [get_pins {core/rgb_session1_mailbox/held*/Q}] -to [get_pins {core/bridge1/sampled_session_epoch*/D}]
set_false_path -to [get_pins {core/rgb_session1_mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {core/rgb_session1_mailbox/ack_sync_0_s0/D}]
set_max_delay 12.000 -from [get_pins {core/rgb_session2_mailbox/held*/Q}] -to [get_pins {core/bridge2/sampled_session_epoch*/D}]
set_false_path -hold -from [get_pins {core/rgb_session2_mailbox/held*/Q}] -to [get_pins {core/bridge2/sampled_session_epoch*/D}]
set_false_path -to [get_pins {core/rgb_session2_mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {core/rgb_session2_mailbox/ack_sync_0_s0/D}]
# Gray millisecond count: only the first destination synchronizer accepts async arrival.
set_false_path -to [get_pins {core/bridge*/time_sync0*/D}]

# Expose local critical paths separately from deliberately bounded CDC paths.
report_timing -setup -from [get_clocks {clk50m}] -to [get_clocks {clk50m}] -max_paths 1000
report_timing -setup -from [get_clocks {cam1_pclk}] -to [get_clocks {cam1_pclk}] -max_paths 1000
report_timing -setup -from [get_clocks {cam2_pclk}] -to [get_clocks {cam2_pclk}] -max_paths 1000
report_timing -setup -from [get_clocks {rgmii_rx_clk}] -to [get_clocks {rgmii_rx_clk}] -max_paths 1000
report_timing -setup -from [get_pins {core/sender/*/Q}] -to [get_pins {core/sender/*/D}] -max_paths 1000
# Include enable pins explicitly; a /D-only subset cannot sign off shared CEs.
report_timing -setup -to [get_pins {core/sender/*/CE}] -max_paths 1000
report_timing -setup -to [get_pins {motors/encoder_feedback.balance/*/CE}] -max_paths 1000
