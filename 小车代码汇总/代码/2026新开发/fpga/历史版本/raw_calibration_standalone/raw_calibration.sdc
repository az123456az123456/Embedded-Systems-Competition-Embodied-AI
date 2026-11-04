create_clock -name clk50m -period 20.000 [get_ports {clk50m}]
create_clock -name cam1_pclk -period 10.000 [get_ports {cam1_pclk}]
create_clock -name cam2_pclk -period 10.000 [get_ports {cam2_pclk}]
# Camera/RGMII external sampling windows still require board validation.
# Only async assertion of explicit synchronous-release reset chains.
set_false_path -to [get_pins {*reset/release_sync*/CLEAR}]
set_false_path -to [get_pins {ready1_0_s0/D ready2_0_s0/D profile_sync_0_s0/D}]
# Snapshot buses are immutable before terminal is synchronized twice. They
# terminate only at local sampling registers; retain a real route-time bound.
set_max_delay 12.000 -from [get_pins {setup*/snapshot*/Q setup*/stage*/Q setup*/error*/Q setup*/failed_addr*/Q setup*/locked*/Q}] -to [get_pins {profile_saved*/D}]
set_false_path -hold -from [get_pins {setup*/snapshot*/Q setup*/stage*/Q setup*/error*/Q setup*/failed_addr*/Q setup*/locked*/Q}] -to [get_pins {profile_saved*/D}]
# Registered Gray pointers, no full-domain false-path cuts.
set_max_delay 8.000 -from [get_pins {rows/cache*/fifo/wgray*/Q rows/cache*/fifo/wbin_14*/Q}] -to [get_pins {rows/cache*/fifo/rq1_wgray*/D}]
set_max_delay 8.000 -from [get_pins {rows/cache*/fifo/rgray*/Q}] -to [get_pins {rows/cache*/fifo/wq1_rgray*/D}]
set_false_path -to [get_pins {rows/cache*/fifo/wr_reset_sync*/CLEAR rows/cache*/fifo/rd_reset_sync*/CLEAR}]
report_timing -setup -max_paths 1000
report_timing -hold -max_paths 1000
report_timing -recovery -max_paths 1000
report_timing -removal -max_paths 1000
