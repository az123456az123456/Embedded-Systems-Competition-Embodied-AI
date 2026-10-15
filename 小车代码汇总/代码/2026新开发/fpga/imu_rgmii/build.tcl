set_device GW5AT-LV60PG484AC1/I0
set_option -synthesis_tool gowinsynthesis
set_option -top_module imu_rgmii_top
set_option -output_base_name imu_rgmii
add_file rtl/i2c_reg_master.v
add_file rtl/mpu9250_reader.v
add_file rtl/imu_ethernet_tx.v
add_file rtl/imu_rgmii_top.v
add_file imu_rgmii.cst
add_file imu_rgmii.sdc
run all
