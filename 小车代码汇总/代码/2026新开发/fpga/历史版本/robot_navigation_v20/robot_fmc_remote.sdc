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
set_false_path -to [get_pins {core/init_done_tx_0_s0/D}]

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
set_false_path -to [get_pins {core/controller_top/controller_reset/release_sync*/CLEAR}]
set_false_path -to [get_pins {core/pclk1_init_sync_0_s0/D}]
set_false_path -to [get_pins {core/pclk2_init_sync_0_s0/D}]
# Inspect every negative endpoint instead of just the default 25 paths.
report_timing -setup -max_paths 100
report_timing -recovery -max_paths 100
report_timing -hold -max_paths 100
report_timing -removal -max_paths 100



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
report_timing -setup -from [get_clocks {rgmii_rx_clk}] -to [get_clocks {rgmii_rx_clk}] -max_paths 20
report_timing -setup -from [get_pins {motor_status/sender/*/Q}] -to [get_pins {motor_status/sender/*/D}] -max_paths 20

# v20 camera FIFO: only registered Gray pointers cross domains. Keep the
# same <=8ns propagation bound (8ns reader / 10ns constrained writer).
# Stage2 synchronizers, full/empty logic and memory enables stay timed.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache*/fifo/wgray*/Q}] -to [get_pins {core/controller_top/cache*/fifo/rq1_wgray*/D*}]
set_max_delay 8.000 -from [get_pins {core/controller_top/cache*/fifo/rgray*/Q}] -to [get_pins {core/controller_top/cache*/fifo/wq1_rgray*/D*}]
# Async assertion enters only the local reset-release chain; its synchronized
# stage2 release and every downstream functional register remain timed.
set_false_path -to [get_pins {core/controller_top/cache*/fifo/wr_reset_sync*/CLEAR}]
set_false_path -to [get_pins {core/controller_top/cache*/fifo/rd_reset_sync*/CLEAR}]
report_timing -setup -from [get_pins {core/controller_top/cache*/fifo/wgray*/Q}] -to [get_pins {core/controller_top/cache*/fifo/rq1_wgray*/D*}] -max_paths 100
report_timing -setup -from [get_pins {core/controller_top/cache*/fifo/rgray*/Q}] -to [get_pins {core/controller_top/cache*/fifo/wq1_rgray*/D*}] -max_paths 100

# Gray bit14 is identical to binary bit14; synthesis merges those registers.
# Bound that actual registered source too, instead of leaving an alias untimed.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache*/fifo/wbin_14*/Q}] -to [get_pins {core/controller_top/cache*/fifo/rq1_wgray_14*/D*}]
set_max_delay 8.000 -from [get_pins {core/controller_top/cache*/fifo/rbin_14*/Q}] -to [get_pins {core/controller_top/cache*/fifo/wq1_rgray_14*/D*}]
report_timing -setup -from [get_pins {core/controller_top/cache*/fifo/wbin_14*/Q}] -to [get_pins {core/controller_top/cache*/fifo/rq1_wgray_14*/D*}] -max_paths 100
report_timing -setup -from [get_pins {core/controller_top/cache*/fifo/rbin_14*/Q}] -to [get_pins {core/controller_top/cache*/fifo/wq1_rgray_14*/D*}] -max_paths 100

# v20 TX row FIFO uses the same implementation at8KiB (14-bit pointers).
# Current write/read clocks share125MHz; retain bounded pointer crossings.
set_max_delay 8.000 -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/wgray*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/rq1_wgray*/D*}]
set_max_delay 8.000 -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/rgray*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/wq1_rgray*/D*}]
set_max_delay 8.000 -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/wbin_13*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/rq1_wgray_13*/D*}]
set_max_delay 8.000 -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/rbin_13*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/wq1_rgray_13*/D*}]
set_false_path -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/wr_reset_sync*/CLEAR}]
set_false_path -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/rd_reset_sync*/CLEAR}]
report_timing -setup -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/wgray*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/rq1_wgray*/D*}] -max_paths 100
report_timing -setup -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/rgray*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/wq1_rgray*/D*}] -max_paths 100
report_timing -setup -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/wbin_13*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/rq1_wgray_13*/D*}] -max_paths 100
report_timing -setup -from [get_pins {core/UDP_Send/eth_dcfifo/fifo/rbin_13*/Q}] -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/wq1_rgray_13*/D*}] -max_paths 100
