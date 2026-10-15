cd [file dirname [info script]]
set_device GW5AT-LV60PG484AC1/I0
set_option -enable_dsrm 1
set_option -use_cpu_as_gpio 1
set_option -synthesis_tool gowinsynthesis
set_option -top_module fmc_four_motor_top
set_option -output_base_name fmc_four_motor
add_file rtl/four_motor_ramp_test.v
add_file rtl/fmc_four_motor_top.v
add_file fmc_four_motor.cst
add_file fmc_four_motor.sdc
run all
