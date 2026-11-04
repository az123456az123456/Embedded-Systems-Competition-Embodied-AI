# After opening robot_control_v39_rgb.gprj in Gowin IDE,
# source this file in its Tcl console before building. No RTL/SDC is changed.
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 0
set_option -use_jtag_as_gpio 0
set_option -place_option 2
set_option -route_option 0
set_option -clock_route_order 1
set_option -timing_driven 1
set_option -synthesis_tool gowinsynthesis
set_option -top_module robot_fmc_remote_top
set_option -output_base_name robot_control_v39_rgb
