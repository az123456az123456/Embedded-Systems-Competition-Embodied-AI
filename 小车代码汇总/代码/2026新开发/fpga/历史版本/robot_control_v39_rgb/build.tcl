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
set_option -top_module robot_fmc_remote_top
set_option -output_base_name robot_control_v39_rgb
add_file camera/Camera_ETH_Formator.v
add_file camera/cache.v
add_file camera/camera_init.v
add_file camera/i2c_bit_shift.v
add_file camera/i2c_control.v
add_file camera/ov5640_init_table_jpeg.v
add_file camera/ov5640_init_table_raw.v
add_file camera/ov7725_init_table_rgb.v
add_file camera/controller.v
add_file camera/controller_top.v
add_file camera/camera_async_fifo.v
add_file camera/CRC32_D8.v
add_file camera/UDP_Send.v
add_file camera/eth_fifo_timing.v
add_file camera/gmii_to_rgmii.v
add_file camera/gowin_pll.v
add_file camera/gowin_pll_mod.v
add_file camera/mdio_bit_shift.v
add_file camera/phy_reg_config.v
add_file camera/ov5640x2_udp_rgmii.v
add_file camera/pll_init.v
add_file camera/camera_diag.v
add_file camera/ov5640_af_loader.v
add_file imu/i2c_reg_master.v
add_file imu/mpu9250_reader.v
add_file imu/imu_ethernet_tx.v
add_file imu/imu_sample_filter.v
add_file imu/imu_gmii_source.v
add_file softcore/vendor/picorv32.v
add_file softcore/robot_nav_soc.v
add_file rtl/robot_fmc_remote_top.v
add_file rtl/motor_packet_rx.v
add_file rtl/nav_packet_rx.v
add_file rtl/robot_rgmii_rx.v
add_file rtl/nav_message_tx.v
add_file rtl/motor_remote_control.v
add_file rtl/motor_encoder_sync.v
add_file rtl/motor_status_tx.v
add_file robot_fmc_remote.cst
add_file robot_fmc_remote.sdc
add_file sensors/encoder_4ch.v
add_file sensors/robot_sensor_capture.v
add_file sensors/sensor_status_tx.v
add_file sensors/a02_uart_rx.v
add_file sensors/ultrasonic_sensor.v
add_file sensors/ultrasonic_driver.v
add_file sensors/ultrasonic_3ch.v
add_file preprocess/bayer_demosaic.v
add_file preprocess/rgb_bilinear_resize.v
add_file preprocess/rgb_capture_bridge.v
add_file preprocess/rgb_letterbox.v
add_file preprocess/rgb_packet_tx.v
add_file preprocess/rgb_session_rx.v
add_file preprocess/rgb_stream_fifo.v
run all
