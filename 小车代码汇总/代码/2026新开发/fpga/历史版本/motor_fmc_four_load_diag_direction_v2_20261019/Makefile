.PHONY: test
test:
	mkdir -p build
	iverilog -g2012 -s tb_load_diag -o build/tb rtl/*.v tests/tb_load_diag.v
	vvp build/tb
