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
