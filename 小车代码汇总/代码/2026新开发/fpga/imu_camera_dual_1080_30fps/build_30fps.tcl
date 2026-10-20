cd [file dirname [info script]]
set_device GW5AT-LV60PG484AC1/I0
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
set_option -synthesis_tool gowinsynthesis
set_option -top_module imu_camera_dual_top
set_option -output_base_name imu_camera_dual_1080_30fps
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
add_file camera/dc_fifo.v
add_file camera/CRC32_D8.v
add_file camera/UDP_Send.v
add_file camera/eth_dcfifo.v
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
add_file imu/imu_sample_filter.v
add_file imu/imu_ethernet_tx.v
add_file imu/imu_gmii_source.v
add_file rtl/gmii_frame_share.v
add_file rtl/imu_camera_dual_top.v
add_file imu_camera_dual.cst
add_file imu_camera_dual.sdc
run all
