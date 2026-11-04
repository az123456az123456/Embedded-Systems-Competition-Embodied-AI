create_clock -name clk50m -period 20.000 [get_ports {clk50m}]
create_clock -name rgmii_rx_clk -period 8.000 [get_ports {rgmii_rx_clk}]
set_max_delay 12.000 -from [get_pins {command_mailbox/held*/Q}] -to [get_pins {command_mailbox/dst_data*/D}]
set_min_delay 0.000 -from [get_pins {command_mailbox/held*/Q}] -to [get_pins {command_mailbox/dst_data*/D}]
set_false_path -to [get_pins {command_mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {command_mailbox/ack_sync_0_s0/D}]
set_max_delay 12.000 -from [get_pins {status/mailbox/held*/Q}] -to [get_pins {status/sender/saved*/D}]
set_min_delay 0.000 -from [get_pins {status/mailbox/held*/Q}] -to [get_pins {status/sender/saved*/D}]
set_false_path -to [get_pins {status/mailbox/req_sync_0_s0/D}]
set_false_path -to [get_pins {status/mailbox/ack_sync_0_s0/D}]

# Asynchronous handshake data is held until the returning two-stage ACK.
# Capture occurs only after the synchronized request; same-edge hold checks
# between unrelated clocks do not model this path. Keep the 12ns max bound.
set_false_path -hold -from [get_pins {command_mailbox/held*/Q}] -to [get_pins {command_mailbox/dst_data*/D}]
set_false_path -hold -from [get_pins {status/mailbox/held*/Q}] -to [get_pins {status/sender/saved*/D}]
# Only the dedicated asynchronous-assert / synchronous-release reset chain.
set_false_path -to [get_pins {receiver/reset_sync*/CLEAR}]
