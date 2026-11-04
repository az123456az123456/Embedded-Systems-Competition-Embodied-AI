IVERILOG ?= iverilog
VVP ?= vvp
.PHONY: test build
test:
	mkdir -p build reports
	python3 tools/generate_messages.py
	$(IVERILOG) -g2012 -s tb_direction_sequence -o build/test rtl/motor_direction_sequence.v rtl/diag_uart_logger.v tests/tb_direction_sequence.sv
	$(VVP) build/test > reports/simulation.log
	python3 tools/check_uart.py
	cat reports/simulation.log
build:
	sh tools/build.sh
