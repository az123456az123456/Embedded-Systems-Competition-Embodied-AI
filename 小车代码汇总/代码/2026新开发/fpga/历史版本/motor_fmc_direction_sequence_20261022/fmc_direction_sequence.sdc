create_clock -name clk50m -period 20.000 [get_ports {clk50m}]
# Motor control outputs are asynchronous to the motor driver; no external sampling clock.
