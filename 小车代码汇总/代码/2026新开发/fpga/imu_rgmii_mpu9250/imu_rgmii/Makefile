CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CROSS_COMPILE ?= aarch64-none-linux-gnu-
IMU_SHARED = ../../../rk3588/robot_phone/backend/perception
IMU_SOURCE = $(IMU_SHARED)/imu_data.c
RTL = rtl/i2c_reg_master.v rtl/mpu9250_reader.v rtl/imu_ethernet_tx.v rtl/imu_rgmii_top.v
.PHONY: all test synth rk3588
all: build/imu-monitor
build:
	mkdir -p build
build/imu-monitor: software/imu_monitor.c $(IMU_SOURCE) $(IMU_SHARED)/imu_data.h | build
	$(CC) $(CFLAGS) -I$(IMU_SHARED) $< $(IMU_SOURCE) -lm -o $@
rk3588: | build
	$(CROSS_COMPILE)gcc $(CFLAGS) -I$(IMU_SHARED) -static software/imu_monitor.c $(IMU_SOURCE) -lm -o build/imu-monitor.aarch64
test: all
	$(CC) $(CFLAGS) -I$(IMU_SHARED) $(IMU_SOURCE) ../../../rk3588/robot_phone/tests/test_imu.c -lm -o build/test-attitude
	./build/test-attitude
	iverilog -g2012 -s tb_sensor -o build/tb_sensor rtl/i2c_reg_master.v rtl/mpu9250_reader.v tests/tb_sensor.v
	vvp build/tb_sensor
	iverilog -g2012 -s tb_network -o build/tb_network rtl/imu_ethernet_tx.v tests/tb_network.v
	vvp build/tb_network
	iverilog -g2012 -s tb_top -o build/tb_top $(RTL) tests/tb_top.v
	vvp build/tb_top
	python3 tests/check_frames.py
synth: | build
	yosys -Q -l build/synthesis.log -p 'read_verilog $(RTL); synth_gowin -family gw5a -strict-gw5a-dffs -top imu_rgmii_top; check -assert; stat' > build/synthesis.stdout
