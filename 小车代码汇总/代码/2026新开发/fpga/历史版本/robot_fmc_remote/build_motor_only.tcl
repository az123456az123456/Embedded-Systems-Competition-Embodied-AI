cd [file dirname [info script]]
set_device GW5AT-LV60PG484AC1/I0
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
set_option -synthesis_tool gowinsynthesis
set_option -top_module fmc_remote_motor_only_top
set_option -output_base_name fmc_remote_motor_only
add_file camera/gmii_to_rgmii.v
add_file imu/imu_ethernet_tx.v
add_file rtl/motor_packet_rx.v
add_file rtl/motor_remote_control.v
add_file rtl/motor_status_tx.v
add_file rtl/fmc_remote_motor_only_top.v
add_file fmc_remote_motor_only.cst
add_file fmc_remote_motor_only.sdc
run all
