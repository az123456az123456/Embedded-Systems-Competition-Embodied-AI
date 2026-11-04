cd [file dirname [info script]]
set_device GW5AT-LV60PG484AC1/I0
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -place_option 2
# User-requested clean retry: Place Option 2, original Route Option 2.
set_option -route_option 2
set_option -clock_route_order 1
set_option -timing_driven 1
set_option -synthesis_tool gowinsynthesis
set_option -top_module robot_fmc_remote_top
set_option -output_base_name robot_navigation_v42_singlemap
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
add_file camera/ov5640_af_loader.v
add_file imu/i2c_reg_master.v
add_file imu/mpu9250_reader.v
add_file imu/imu_ethernet_tx.v
add_file imu/imu_sample_filter.v
add_file imu/imu_gmii_source.v
add_file softcore/vendor/picorv32.v
add_file softcore/nav_ram80.v
add_file softcore/robot_nav_soc.v
add_file rtl/nav_exec_io.v
add_file rtl/nav3_ingress.v
add_file rtl/nav_eq_lut.v
add_file rtl/nav_owner_gate.v
add_file rtl/nav3_tx_queue.v
add_file rtl/robot_fmc_remote_top.v
add_file rtl/motor_packet_rx.v
add_file rtl/robot_rgmii_rx.v
add_file rtl/motor_remote_control.v
add_file rtl/motor_encoder_sync.v
add_file rtl/motor_status_tx.v
add_file rtl/motor_status_align.v
add_file robot_fmc_remote.cst
add_file robot_fmc_remote.sdc
add_file sensors/encoder_4ch.v
add_file sensors/robot_sensor_capture.v
add_file sensors/sensor_status_tx.v
add_file sensors/a02_uart_rx.v
add_file sensors/ultrasonic_sensor.v
add_file sensors/ultrasonic_driver.v
add_file sensors/ultrasonic_3ch.v
add_file rtl/nav3_frame_rx.v
add_file rtl/nav3_async_mailbox.v
add_file rtl/nav3_packet_ram.v
add_file rtl/nav3_serializer.v
add_file preprocess/robot_gmii_share.v
set_option -inc_pnr 0
set_option -inc_place 0
run all
