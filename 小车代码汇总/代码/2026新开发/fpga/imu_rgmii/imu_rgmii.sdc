create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
create_clock -name rgmii_rx_clk -period 8.000 [get_ports {rgmii_rx_clk}]
# Bundled-data mailbox: held bus remains stable until two-flop ack returns.
set_clock_groups -asynchronous -group [get_clocks {clk_50m}] -group [get_clocks {rgmii_rx_clk}]
# Board source-synchronous I/O delays must be closed in Gowin STA on hardware.
# Uses the same ODDR topology as the board's tested loopback project.
# This baseline is NOT a complete sign-off of RGMII output timing.
