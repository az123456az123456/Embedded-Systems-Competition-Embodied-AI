create_clock -name clk50m -period 20.000 [get_ports {clk50m}]
# Motor outputs are slow control signals, no synchronous external sampling assumed.
