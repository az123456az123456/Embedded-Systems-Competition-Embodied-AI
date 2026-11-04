cd [file dirname [info script]]
source project_settings.tcl
add_file rtl/motor_direction_sequence.v
add_file rtl/diag_uart_logger.v
add_file rtl/fmc_direction_sequence_top.v
add_file fmc_direction_sequence.cst
add_file fmc_direction_sequence.sdc
run all
