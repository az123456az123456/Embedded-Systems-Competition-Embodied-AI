cd [file dirname [info script]]
set_device GW5AT-LV60PG484AC1/I0
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -place_option 2
set_option -route_option 0
set_option -clock_route_order 1
set_option -timing_driven 1
set_option -synthesis_tool gowinsynthesis
set_option -top_module raw_calibration_top
set_option -output_base_name raw_calibration
add_file camera/CRC32_D8.v
add_file camera/Camera_ETH_Formator.v
add_file camera/UDP_Send.v
add_file camera/cache.v
add_file camera/camera_async_fifo.v
add_file camera/controller.v
add_file camera/controller_top.v
add_file camera/eth_fifo_timing.v
add_file camera/gmii_to_rgmii.v
add_file camera/gowin_pll.v
add_file camera/gowin_pll_mod.v
add_file camera/i2c_bit_shift.v
add_file camera/i2c_control.v
add_file camera/ov5640_init_table_raw.v
add_file camera/pll_init.v
add_file rtl/raw_calibration_top.v
add_file rtl/raw_camera_setup.v
add_file raw_calibration.cst
add_file raw_calibration.sdc
run all
