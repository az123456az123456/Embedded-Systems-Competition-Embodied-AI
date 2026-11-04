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
# RGMII RX is asynchronous to clk50m. Only first-stage synchronizers and
# explicitly bounded, held-until-ACK data bundles below are exempted.
# No whole-domain clock group: destination-register control paths stay timed.
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
# Camera FIFO reset synchronizers: fifo_aclr is generated in the camera PCLK
# domain, while rd_reset_sync is released in the shared PLL read-clock domain.
# The synchronizer already provides asynchronous assertion and synchronous
# release; exclude only recovery/removal checks on its asynchronous CLEAR pin.
set_false_path -to [get_pins {core/controller_top/cache1/fifo/rd_reset_sync*/CLEAR}]
set_false_path -to [get_pins {core/controller_top/cache2/fifo/rd_reset_sync*/CLEAR}]
set_false_path -to [get_pins {core/UDP_Send/eth_dcfifo/fifo/rd_reset_sync*/CLEAR}]
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

# v41 removes the old NAV2 request/reply CDC instances. NAV3 has independent
# stable-bank handshakes; exact new endpoints are added after RTL mapping.
report_timing -setup -from [get_clocks {rgmii_rx_clk}] -to [get_clocks {rgmii_rx_clk}] -max_paths 1000
report_timing -setup -from [get_pins {motor_status/sender/*/Q}] -to [get_pins {motor_status/sender/*/D}] -max_paths 1000

# FPGA image transport is RAW8 row UDP; no RGB FIFO/session CDC remains.

# v41 NAV3 bounded bundled-data crossings
# Only first request/ack synchronizer stages are asynchronous. The original
# payload stays held until returning ACK. No clock-domain-wide exceptions.
# Except for the captured CANCEL mailbox below, consumer sets include CE
# and D because the remaining dst_data buses are held-wire references.
set_false_path -to [get_pins {navigation_io/ingress/cancel_cross/req_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/ingress/cancel_cross/ack_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/ingress/lane_cross*.command/req_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/ingress/lane_cross*.command/ack_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/ingress/lane_cross*.response/req_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/ingress/lane_cross*.response/ack_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/ingress/lane_cross*.published/req_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/ingress/lane_cross*.published/ack_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/transmit/slot_cross*.queue/req_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/transmit/slot_cross*.queue/ack_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/transmit/read_cross/req_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/transmit/read_cross/ack_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/transmit/read_response/req_sync_0_s0/D}]
set_false_path -to [get_pins {navigation_io/transmit/read_response/ack_sync_0_s0/D}]
set_max_delay 12.000 -from [get_pins {navigation_io/ingress/lane_cross*.command/held*/Q}] -to [get_pins {navigation_io/ingress/rx_pinned*/D navigation_io/ingress/rx_pin_bank*/D navigation_io/ingress/rx_pin_token*/D navigation_io/ingress/rx_latest*/D navigation_io/ingress/resp_data*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/lane_cross*.command/held*/Q}] -to [get_pins {navigation_io/ingress/rx_pinned*/D navigation_io/ingress/rx_pin_bank*/D navigation_io/ingress/rx_pin_token*/D navigation_io/ingress/rx_latest*/D navigation_io/ingress/resp_data*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/lane_cross*.command/held*/Q}] -to [get_pins {navigation_io/ingress/rx_pinned*/D navigation_io/ingress/rx_pin_bank*/D navigation_io/ingress/rx_pin_token*/D navigation_io/ingress/rx_latest*/D navigation_io/ingress/resp_data*/D}]
set_max_delay 12.000 -from [get_pins {navigation_io/ingress/lane_cross*.command/held*/Q}] -to [get_pins {navigation_io/ingress/rx_pinned*/CE navigation_io/ingress/rx_pin_bank*/CE navigation_io/ingress/rx_pin_token*/CE navigation_io/ingress/rx_latest*/CE navigation_io/ingress/resp_data*/CE}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/lane_cross*.command/held*/Q}] -to [get_pins {navigation_io/ingress/rx_pinned*/CE navigation_io/ingress/rx_pin_bank*/CE navigation_io/ingress/rx_pin_token*/CE navigation_io/ingress/rx_latest*/CE navigation_io/ingress/resp_data*/CE}]
set_false_path -hold -from [get_pins {navigation_io/ingress/lane_cross*.command/held*/Q}] -to [get_pins {navigation_io/ingress/rx_pinned*/CE navigation_io/ingress/rx_pin_bank*/CE navigation_io/ingress/rx_pin_token*/CE navigation_io/ingress/rx_latest*/CE navigation_io/ingress/resp_data*/CE}]
set_max_delay 12.000 -from [get_pins {navigation_io/ingress/lane_cross*.response/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_pinned*/D navigation_io/ingress/acquire_busy*/D navigation_io/ingress/replied_empty*/D navigation_io/ingress/cpu_bank*/D navigation_io/ingress/cpu_bytes*/D navigation_io/ingress/cpu_token*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/lane_cross*.response/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_pinned*/D navigation_io/ingress/acquire_busy*/D navigation_io/ingress/replied_empty*/D navigation_io/ingress/cpu_bank*/D navigation_io/ingress/cpu_bytes*/D navigation_io/ingress/cpu_token*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/lane_cross*.response/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_pinned*/D navigation_io/ingress/acquire_busy*/D navigation_io/ingress/replied_empty*/D navigation_io/ingress/cpu_bank*/D navigation_io/ingress/cpu_bytes*/D navigation_io/ingress/cpu_token*/D}]
set_max_delay 12.000 -from [get_pins {navigation_io/ingress/lane_cross*.response/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_pinned*/CE navigation_io/ingress/acquire_busy*/CE navigation_io/ingress/replied_empty*/CE navigation_io/ingress/cpu_bank*/CE navigation_io/ingress/cpu_bytes*/CE navigation_io/ingress/cpu_token*/CE}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/lane_cross*.response/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_pinned*/CE navigation_io/ingress/acquire_busy*/CE navigation_io/ingress/replied_empty*/CE navigation_io/ingress/cpu_bank*/CE navigation_io/ingress/cpu_bytes*/CE navigation_io/ingress/cpu_token*/CE}]
set_false_path -hold -from [get_pins {navigation_io/ingress/lane_cross*.response/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_pinned*/CE navigation_io/ingress/acquire_busy*/CE navigation_io/ingress/replied_empty*/CE navigation_io/ingress/cpu_bank*/CE navigation_io/ingress/cpu_bytes*/CE navigation_io/ingress/cpu_token*/CE}]
set_max_delay 12.000 -from [get_pins {navigation_io/ingress/lane_cross*.published/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_stamp*/D navigation_io/ingress/cpu_pub_token*/D navigation_io/ingress/ordinary_summary*/D navigation_io/ingress/cpu_pinned*/D navigation_io/ingress/cpu_bank*/D navigation_io/ingress/cpu_bytes*/D navigation_io/ingress/cpu_token*/D navigation_io/ingress/accepted_count*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/lane_cross*.published/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_stamp*/D navigation_io/ingress/cpu_pub_token*/D navigation_io/ingress/ordinary_summary*/D navigation_io/ingress/cpu_pinned*/D navigation_io/ingress/cpu_bank*/D navigation_io/ingress/cpu_bytes*/D navigation_io/ingress/cpu_token*/D navigation_io/ingress/accepted_count*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/lane_cross*.published/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_stamp*/D navigation_io/ingress/cpu_pub_token*/D navigation_io/ingress/ordinary_summary*/D navigation_io/ingress/cpu_pinned*/D navigation_io/ingress/cpu_bank*/D navigation_io/ingress/cpu_bytes*/D navigation_io/ingress/cpu_token*/D navigation_io/ingress/accepted_count*/D}]
set_max_delay 12.000 -from [get_pins {navigation_io/ingress/lane_cross*.published/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_stamp*/CE navigation_io/ingress/cpu_pub_token*/CE navigation_io/ingress/ordinary_summary*/CE navigation_io/ingress/cpu_pinned*/CE navigation_io/ingress/cpu_bank*/CE navigation_io/ingress/cpu_bytes*/CE navigation_io/ingress/cpu_token*/CE navigation_io/ingress/accepted_count*/CE}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/lane_cross*.published/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_stamp*/CE navigation_io/ingress/cpu_pub_token*/CE navigation_io/ingress/ordinary_summary*/CE navigation_io/ingress/cpu_pinned*/CE navigation_io/ingress/cpu_bank*/CE navigation_io/ingress/cpu_bytes*/CE navigation_io/ingress/cpu_token*/CE navigation_io/ingress/accepted_count*/CE}]
set_false_path -hold -from [get_pins {navigation_io/ingress/lane_cross*.published/held*/Q}] -to [get_pins {navigation_io/ingress/cpu_stamp*/CE navigation_io/ingress/cpu_pub_token*/CE navigation_io/ingress/ordinary_summary*/CE navigation_io/ingress/cpu_pinned*/CE navigation_io/ingress/cpu_bank*/CE navigation_io/ingress/cpu_bytes*/CE navigation_io/ingress/cpu_token*/CE navigation_io/ingress/accepted_count*/CE}]
# CANCEL now captures its bundle in clk50m on the same edge as dst_valid.
# held is immutable for the two-stage request crossing plus destination capture,
# and remains so through consumption and the two-stage returning ACK.
# Bound ONLY held -> capture to 12ns. Capture -> owner -> motors is synchronous
# clk50m timing; no broad consumer or motor-output exception remains.
set_max_delay 12.000 -from [get_pins {navigation_io/ingress/cancel_cross/held*/Q}] -to [get_pins {navigation_io/ingress/cancel_cross/*payload*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/cancel_cross/held*/Q}] -to [get_pins {navigation_io/ingress/cancel_cross/*payload*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/cancel_cross/held*/Q}] -to [get_pins {navigation_io/ingress/cancel_cross/*payload*/D}]
# Keep the existing 12ns combinational motor-output budget after the new
# destination register. This is local propagation timing, not a CDC exception.
# Candidate22: capture -> final pin_bundle D uses normal clk50m setup/hold.
# Keep the 12ns local pin-path budget from the NEW output registers to pins.
# This is a stage-boundary update, not an external-driver timing signoff.
# It does not preserve a 12ns end-to-end capture-to-pin requirement.
set_max_delay 12.000 -from [get_pins {motors/pin_bundle*/Q}] -to [get_ports {motor_pwm* motor_in1* motor_in2*}]
report_timing -setup -to [get_pins {motors/pin_bundle*/D}] -max_paths 1000
report_timing -setup -from [get_pins {motors/pin_bundle*/Q}] -to [get_ports {motor_pwm* motor_in1* motor_in2*}] -max_paths 1000
set_max_delay 12.000 -from [get_pins {navigation_io/transmit/slot_cross*.queue/held*/Q}] -to [get_pins {navigation_io/transmit/active_slot*/D navigation_io/transmit/serializer/saved_slot*/D navigation_io/transmit/serializer/saved_nva*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/transmit/slot_cross*.queue/held*/Q}] -to [get_pins {navigation_io/transmit/active_slot*/D navigation_io/transmit/serializer/saved_slot*/D navigation_io/transmit/serializer/saved_nva*/D}]
set_false_path -hold -from [get_pins {navigation_io/transmit/slot_cross*.queue/held*/Q}] -to [get_pins {navigation_io/transmit/active_slot*/D navigation_io/transmit/serializer/saved_slot*/D navigation_io/transmit/serializer/saved_nva*/D}]
set_max_delay 12.000 -from [get_pins {navigation_io/transmit/slot_cross*.queue/held*/Q}] -to [get_pins {navigation_io/transmit/active_slot*/CE navigation_io/transmit/serializer/saved_slot*/CE navigation_io/transmit/serializer/saved_nva*/CE}]
set_min_delay 0.000 -from [get_pins {navigation_io/transmit/slot_cross*.queue/held*/Q}] -to [get_pins {navigation_io/transmit/active_slot*/CE navigation_io/transmit/serializer/saved_slot*/CE navigation_io/transmit/serializer/saved_nva*/CE}]
set_false_path -hold -from [get_pins {navigation_io/transmit/slot_cross*.queue/held*/Q}] -to [get_pins {navigation_io/transmit/active_slot*/CE navigation_io/transmit/serializer/saved_slot*/CE navigation_io/transmit/serializer/saved_nva*/CE}]
set_max_delay 12.000 -from [get_pins {navigation_io/transmit/read_cross/held*/Q}] -to [get_pins {navigation_io/transmit/packet_memory/*/ADB*}]
set_min_delay 0.000 -from [get_pins {navigation_io/transmit/read_cross/held*/Q}] -to [get_pins {navigation_io/transmit/packet_memory/*/ADB*}]
set_false_path -hold -from [get_pins {navigation_io/transmit/read_cross/held*/Q}] -to [get_pins {navigation_io/transmit/packet_memory/*/ADB*}]
set_max_delay 12.000 -from [get_pins {navigation_io/transmit/read_response/held*/Q}] -to [get_pins {navigation_io/transmit/bus_rsp_data*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/transmit/read_response/held*/Q}] -to [get_pins {navigation_io/transmit/bus_rsp_data*/D}]
set_false_path -hold -from [get_pins {navigation_io/transmit/read_response/held*/Q}] -to [get_pins {navigation_io/transmit/bus_rsp_data*/D}]
set_max_delay 12.000 -from [get_pins {navigation_io/transmit/read_response/held*/Q}] -to [get_pins {navigation_io/transmit/bus_rsp_data*/CE}]
set_min_delay 0.000 -from [get_pins {navigation_io/transmit/read_response/held*/Q}] -to [get_pins {navigation_io/transmit/bus_rsp_data*/CE}]
set_false_path -hold -from [get_pins {navigation_io/transmit/read_response/held*/Q}] -to [get_pins {navigation_io/transmit/bus_rsp_data*/CE}]
set_max_delay 8.000 -from [get_pins {navigation_io/ingress/bad_gray*/Q}] -to [get_pins {navigation_io/ingress/bad_s0*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/bad_gray*/Q}] -to [get_pins {navigation_io/ingress/bad_s0*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/bad_gray*/Q}] -to [get_pins {navigation_io/ingress/bad_s0*/D}]
set_max_delay 8.000 -from [get_pins {navigation_io/ingress/busy_gray*/Q}] -to [get_pins {navigation_io/ingress/busy_s0*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/busy_gray*/Q}] -to [get_pins {navigation_io/ingress/busy_s0*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/busy_gray*/Q}] -to [get_pins {navigation_io/ingress/busy_s0*/D}]
set_max_delay 8.000 -from [get_pins {navigation_io/ingress/pose_gray*/Q}] -to [get_pins {navigation_io/ingress/pose_s0*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/pose_gray*/Q}] -to [get_pins {navigation_io/ingress/pose_s0*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/pose_gray*/Q}] -to [get_pins {navigation_io/ingress/pose_s0*/D}]
set_max_delay 8.000 -from [get_pins {navigation_io/ingress/permit_gray*/Q}] -to [get_pins {navigation_io/ingress/permit_s0*/D}]
set_min_delay 0.000 -from [get_pins {navigation_io/ingress/permit_gray*/Q}] -to [get_pins {navigation_io/ingress/permit_s0*/D}]
set_false_path -hold -from [get_pins {navigation_io/ingress/permit_gray*/Q}] -to [get_pins {navigation_io/ingress/permit_s0*/D}]
set_false_path -to [get_pins {navigation_io/ingress/latest_s0*/D}]
# Final signoff must audit matched pins and all new unconstrained endpoints.
report_timing -setup -from [get_clocks {clk50m}] -to [get_clocks {clk50m}] -max_paths 1000
report_timing -setup -to [get_pins {navigation_io/owner_gate/*/CE}] -max_paths 1000
report_timing -setup -from [get_clocks {rgmii_rx_clk}] -to [get_clocks {rgmii_rx_clk}] -max_paths 1000

# Reviewed additions only; do not apply to the UDP same-clock FIFO.
# Gray pointers are registered and advance by at most one per source clock.
# Each relevant first-stage input is bounded by the faster 125MHz period (8ns).
# Keep setup/max-delay analysis. Exempt only meaningless asynchronous hold.
# The post-route gate must verify all 60 Q->D paths and their physical delays.
# No set_clock_groups or setup false_path may override these max-delay bounds.
# Synthesis merges write Gray bit14 into wbin_14_s0; explicitly include it.

# core/controller_top/cache1/fifo: write pointer, 15 source Q -> 15 first-stage D.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache1/fifo/wgray_*_s0/Q core/controller_top/cache1/fifo/wbin_14_s0/Q}] -to [get_pins {core/controller_top/cache1/fifo/rq1_wgray_*_s0/D}]
set_min_delay 0.000 -from [get_pins {core/controller_top/cache1/fifo/wgray_*_s0/Q core/controller_top/cache1/fifo/wbin_14_s0/Q}] -to [get_pins {core/controller_top/cache1/fifo/rq1_wgray_*_s0/D}]
set_false_path -hold -from [get_pins {core/controller_top/cache1/fifo/wgray_*_s0/Q core/controller_top/cache1/fifo/wbin_14_s0/Q}] -to [get_pins {core/controller_top/cache1/fifo/rq1_wgray_*_s0/D}]

# core/controller_top/cache1/fifo: read pointer, 15 source Q -> 15 first-stage D.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache1/fifo/rgray_*_s0/Q}] -to [get_pins {core/controller_top/cache1/fifo/wq1_rgray_*_s0/D}]
set_min_delay 0.000 -from [get_pins {core/controller_top/cache1/fifo/rgray_*_s0/Q}] -to [get_pins {core/controller_top/cache1/fifo/wq1_rgray_*_s0/D}]
set_false_path -hold -from [get_pins {core/controller_top/cache1/fifo/rgray_*_s0/Q}] -to [get_pins {core/controller_top/cache1/fifo/wq1_rgray_*_s0/D}]

# core/controller_top/cache2/fifo: write pointer, 15 source Q -> 15 first-stage D.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache2/fifo/wgray_*_s0/Q core/controller_top/cache2/fifo/wbin_14_s0/Q}] -to [get_pins {core/controller_top/cache2/fifo/rq1_wgray_*_s0/D}]
set_min_delay 0.000 -from [get_pins {core/controller_top/cache2/fifo/wgray_*_s0/Q core/controller_top/cache2/fifo/wbin_14_s0/Q}] -to [get_pins {core/controller_top/cache2/fifo/rq1_wgray_*_s0/D}]
set_false_path -hold -from [get_pins {core/controller_top/cache2/fifo/wgray_*_s0/Q core/controller_top/cache2/fifo/wbin_14_s0/Q}] -to [get_pins {core/controller_top/cache2/fifo/rq1_wgray_*_s0/D}]

# core/controller_top/cache2/fifo: read pointer, 15 source Q -> 15 first-stage D.
set_max_delay 8.000 -from [get_pins {core/controller_top/cache2/fifo/rgray_*_s0/Q}] -to [get_pins {core/controller_top/cache2/fifo/wq1_rgray_*_s0/D}]
set_min_delay 0.000 -from [get_pins {core/controller_top/cache2/fifo/rgray_*_s0/Q}] -to [get_pins {core/controller_top/cache2/fifo/wq1_rgray_*_s0/D}]
set_false_path -hold -from [get_pins {core/controller_top/cache2/fifo/rgray_*_s0/Q}] -to [get_pins {core/controller_top/cache2/fifo/wq1_rgray_*_s0/D}]

# Init_Done is a persistent configuration-complete LEVEL, not a pulse.
# camera_init.v:191-197 holds it until reset/Go; two destination FFs exist.
# Source 50MHz and TX125MHz share a PLL. This is a functional level-handshake
# first-stage exception, not an assertion that the PLL outputs are asynchronous.
# Consumers use only init_done_tx[1]; never exempt stage1->stage2 or UDP reset.
set_false_path -to [get_pins {core/init_done_tx_0_s0/D}]
