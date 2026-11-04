PYTHON ?= python3
.PHONY: test
test:
	mkdir -p build
	iverilog -g2012 -s tb_shared_tx -o build/tb_shared_tx camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v rtl/gmii_frame_share.v tests/tb_shared_tx.v
	vvp build/tb_shared_tx
	$(PYTHON) tests/check_shared_frames.py
	iverilog -g2012 -s camera_raw1080_pair_tb -o build/tb_pair camera/controller.v tests/camera_raw1080_pair_tb.sv
	vvp build/tb_pair
	iverilog -g2012 -s tb_sensor -o build/tb_sensor imu/i2c_reg_master.v imu/mpu9250_reader.v tests/tb_sensor.v
	vvp build/tb_sensor
	iverilog -g2012 -s tb_top -o build/tb_imu_source imu/*.v camera/gmii_to_rgmii.v tests/tb_imu_source.v
	vvp build/tb_imu_source

.PHONY: test-camera-reset test-gap
test-camera-reset:
	mkdir -p build
	iverilog -g2012 -s tb_camera_reset -o build/camera_reset camera/ov5640x2_udp_rgmii.v camera/controller_top.v camera/controller.v tests/tb_camera_reset.v
	vvp build/camera_reset
test-gap:
	mkdir -p build
	iverilog -g2012 -s tb_udp_gap -o build/udp-gap camera/UDP_Send.v camera/CRC32_D8.v tests/tb_shared_tx.v tests/tb_udp_gap.v
	vvp build/udp-gap

.PHONY: test-controller-equivalence
test-controller-equivalence:
	mkdir -p build
	iverilog -g2012 -s tb_controller_equivalence -o build/controller-equivalence camera/controller.v tests/controller_reference.v tests/tb_controller_equivalence.v
	vvp build/controller-equivalence

.PHONY: test-imu-filter
test-imu-filter:
	mkdir -p build reports/imu_filter
	iverilog -g2012 -s tb_imu_filter -o build/imu_filter imu/imu_sample_filter.v tests/tb_imu_filter.v
	vvp build/imu_filter
	iverilog -g2012 -s tb_top -o build/imu_filter_source imu/*.v camera/gmii_to_rgmii.v tests/tb_imu_filter_source.v
	vvp build/imu_filter_source
	$(PYTHON) tests/check_imu_filter_frames.py

.PHONY: test-30fps
test-30fps:
	mkdir -p build reports/30fps
	iverilog -g2012 -s tb_camera_30fps -o build/camera-30fps camera/camera_init.v camera/ov5640_init_table_raw.v camera/ov5640_init_table_jpeg.v camera/ov7725_init_table_rgb.v tests/tb_camera_30fps.sv
	vvp build/camera-30fps
	$(PYTHON) tests/test_stream_budget.py
