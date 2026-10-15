.PHONY: test
test:
	mkdir -p build
	iverilog -g2012 -s tb_four_motor_ramp_test -o build/tb rtl/four_motor_ramp_test.v tests/tb_four_motor_ramp_test.v
	vvp build/tb
	iverilog -g2012 -s tb_real_clock -o build/tb_real rtl/four_motor_ramp_test.v tests/tb_real_clock.v
	vvp build/tb_real
	iverilog -g2012 -s tb_staggered_pwm -o build/tb_staggered rtl/four_motor_ramp_test.v tests/tb_staggered_pwm.v
	vvp build/tb_staggered
	iverilog -g2012 -s tb_final_all_continuity -o build/tb_final_all rtl/four_motor_ramp_test.v tests/tb_final_all_continuity.v
	vvp build/tb_final_all
