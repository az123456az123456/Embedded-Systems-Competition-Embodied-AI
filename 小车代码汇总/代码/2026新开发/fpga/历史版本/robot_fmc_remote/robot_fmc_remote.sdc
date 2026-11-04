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

# Camera IP CDC boundaries: only two generated FIFO pointer synchronizer inputs.
# Bound each encoded pointer bus to < both 10ns camera and 8ns reader periods.
# No false path is applied to their downstream synchronized pointer logic.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.wptr*/Q}] -to [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.rq1_wptr*/D*}]
set_max_delay 8.000 -from [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.rptr*/Q}] -to [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.wq1_rptr*/D*}]
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

# Vendor FIFO's dedicated read-reset synchronizer, two stages per camera.
# Only these PRESET pins receive asynchronous PCLK reset; subsequent read logic
# uses the synchronized reset and remains fully timed.
set_false_path -from [get_pins {core/Camera_ETH_Formator*/fifo_aclr*/Q}] -to [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/reset_r_*/PRESET}]
report_timing -setup -from [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.wptr*/Q}] -to [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.rq1_wptr*/D*}] -max_paths 100
report_timing -setup -from [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.rptr*/Q}] -to [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.wq1_rptr*/D*}] -max_paths 100

# Synthesized vendor FIFO read-pointer source (rptr alias can map to rbin_num).
# Still bound to the FIRST write-domain synchronizer; downstream logic is timed.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/rbin_num*/Q}] -to [get_pins {core/controller_top/cache*/dc_fifo/fifo_inst/Equal.wq1_rptr*/D*}]
