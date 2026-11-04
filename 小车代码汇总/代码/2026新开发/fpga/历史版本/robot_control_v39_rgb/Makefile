IVERILOG ?= iverilog
VVP ?= vvp
GOWIN_IDE ?= $(abspath ../../Gowin_V1.9.12.03/IDE)
.DEFAULT_GOAL := test-v39
.PHONY: test-v39
# v39 RGB transport tests; legacy RAW/UDP tests below remain reference-only.
test-v39:
	python3 tests/test_bayer_demosaic.py
	python3 tests/test_bayer_pipeline.py
	python3 tests/preprocess/run_resize_tests.py
	python3 tests/preprocess/run_letterbox_bram_safety.py
	python3 tests/preprocess/run_capture_bridge_tests.py
	python3 tests/preprocess/run_session_pipeline_tests.py
	python3 tests/preprocess/run_rgb_fifo_tests.py
	python3 tests/test_rgb_head_cache.py
	python3 tests/test_rgb_packet_tx.py --full --fps 40 --reports reports/rgb-packet-v35-40fps
	python3 tests/preprocess/run_dual_40fps_tests.py --report ../../integration/robot_rgb_v39/integration/reports/dual-capture-packet-40fps.json
	python3 ../../integration/robot_rgb_v39/integration/test_fpga.py
	python3 ../../integration/robot_rgb_v39/integration/test_fpga_control.py
.PHONY: test-camera-timing
test-camera-timing:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_mailbox_idle_capture -o build/mailbox-idle imu/imu_ethernet_tx.v tests/tb_mailbox_idle_capture.sv
	$(VVP) build/mailbox-idle
	$(IVERILOG) -g2012 -s tb_camera_async_fifo -o build/camera-async-fifo camera/camera_async_fifo.v tests/tb_camera_async_fifo.sv
	$(VVP) build/camera-async-fifo
	$(IVERILOG) -g2012 -s tb_camera_async_fifo -Ptb_camera_async_fifo.AW=13 -o build/camera-async-fifo-8k camera/camera_async_fifo.v tests/tb_camera_async_fifo.sv
	$(VVP) build/camera-async-fifo-8k
	$(IVERILOG) -g2012 -s tb_camera_async_fifo -Ptb_camera_async_fifo.AW=14 -o build/camera-async-fifo-16k camera/camera_async_fifo.v tests/tb_camera_async_fifo.sv
	$(VVP) build/camera-async-fifo-16k
	$(IVERILOG) -g2012 -s tb_camera_fifo_stream -o build/camera-fifo-stream camera/camera_async_fifo.v camera/controller.v tests/tb_camera_fifo_stream.sv
	$(VVP) build/camera-fifo-stream
.PHONY: test test-camera test-gap test-gowin test-real-clock test-all clean
test:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_packet_rx -o build/packet rtl/motor_packet_rx.v tests/tb_packet_rx.v
	$(VVP) build/packet
	$(IVERILOG) -g2012 -s tb_motor_remote -o build/motor rtl/motor_remote_control.v tests/tb_motor_remote.v
	$(VVP) build/motor
	$(IVERILOG) -g2012 -s tb_status_tx -o build/status rtl/motor_status_tx.v tests/tb_status_tx.v
	$(VVP) build/status
	$(IVERILOG) -g2012 -s tb_cdc_clock_loss -o build/cdc rtl/motor_packet_rx.v rtl/motor_remote_control.v tests/tb_cdc_clock_loss.v
	$(VVP) build/cdc
	$(IVERILOG) -g2012 -s tb_gmii_share -o build/share rtl/motor_status_tx.v tests/tb_gmii_share.v
	$(VVP) build/share
test-camera:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_camera_reset -o build/camera_reset camera/ov5640x2_udp_rgmii.v camera/controller_top.v camera/controller.v tests/tb_camera_reset.v
	$(VVP) build/camera_reset
	$(IVERILOG) -g2012 -s tb_shared_tx -o build/shared camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v rtl/motor_status_tx.v tests/tb_shared_tx.v
	$(VVP) build/shared
	python3 tests/check_shared_frames.py
	$(IVERILOG) -g2012 -s camera_raw1080_pair_tb -o build/pair camera/controller.v tests/camera_raw1080_pair_tb.sv
	$(VVP) build/pair
test-gap:
	mkdir -p build
	$(IVERILOG) -g2012 -s tb_udp_gap -o build/udp-gap camera/UDP_Send.v camera/CRC32_D8.v tests/tb_shared_tx.v tests/tb_udp_gap.v
	$(VVP) build/udp-gap
	$(IVERILOG) -g2012 -s tb_udp_gap -Ptb_udp_gap.GAP=256 -o build/udp-gap-old camera/UDP_Send.v camera/CRC32_D8.v tests/tb_shared_tx.v tests/tb_udp_gap.v
	$(VVP) build/udp-gap-old
	$(IVERILOG) -g2012 -s tb_controller_equivalence -o build/controller-equivalence camera/controller.v tests/controller_reference.v tests/tb_controller_equivalence.v
	$(VVP) build/controller-equivalence
test-all: test test-camera test-gowin test-real-clock
test-gowin:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_rgmii_rx -o build/rgmii rtl/motor_packet_rx.v tests/tb_rgmii_rx.v $(GOWIN_IDE)/simlib/gw5a/prim_sim.v
	$(VVP) build/rgmii
test-real-clock:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_real_clock -o build/real_clock rtl/motor_remote_control.v tests/tb_real_clock.v
	$(VVP) build/real_clock
clean:
	rm -rf build

.PHONY: test-imu-filter test-40fps test-rotation test-release
test-imu-filter:
	mkdir -p build reports/imu_filter
	$(IVERILOG) -g2012 -s tb_imu_filter -o build/imu_filter imu/imu_sample_filter.v tests/tb_imu_filter.v
	$(VVP) build/imu_filter
	$(IVERILOG) -g2012 -s tb_top -o build/imu_filter_source imu/*.v camera/gmii_to_rgmii.v tests/tb_imu_filter_source.v
	$(VVP) build/imu_filter_source
	python3 tests/check_imu_filter_frames.py
test-40fps:
	mkdir -p build reports/40fps
	$(IVERILOG) -g2012 -s tb_camera_40fps -o build/camera-40fps camera/camera_init.v camera/ov5640_init_table_raw.v camera/ov5640_init_table_jpeg.v camera/ov7725_init_table_rgb.v tests/tb_camera_40fps.sv
	$(VVP) build/camera-40fps
	python3 tests/test_stream_budget.py
test-rotation:
	mkdir -p build
	$(IVERILOG) -g2012 -s tb_rotation_layout -o build/rotation-layout rtl/motor_remote_control.v tests/tb_rotation_layout.v
	$(VVP) build/rotation-layout
test-release: test-all test-gap test-imu-filter test-40fps test-rotation

.PHONY: test-motor-crosswire test-scheduler-stress
test-motor-crosswire:
	python3 tests/test_motor_crosswire.py
# About100ms of real-rate30FPS camera scheduling, full CRC and real CDC.
# Separate from quick regression because this is computationally expensive.
test-scheduler-stress:
	mkdir -p build reports/scheduler_stress
	$(IVERILOG) -g2012 -s tb_scheduler_30fps_stress -o build/scheduler-stress camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v rtl/motor_status_tx.v tests/tb_scheduler_30fps_stress.v
	$(VVP) build/scheduler-stress > reports/scheduler_stress/rtl.log
	cat reports/scheduler_stress/rtl.log

.PHONY: firmware test-softcore test-nav-network test-nav-scheduler test-nav-crosswire synth
firmware:
	$(MAKE) -C softcore/firmware
test-softcore: firmware
	$(MAKE) -C softcore/firmware host-test
	CCACHE_DISABLE=1 CCACHE_TEMPDIR=/tmp verilator --binary --timing -Wno-fatal --top-module tb_nav_soc --Mdir build/nav_verilator -CFLAGS '-std=c++20' softcore/vendor/picorv32.v softcore/robot_nav_soc.v softcore/tests/tb_nav_soc.v
	build/nav_verilator/Vtb_nav_soc
	$(MAKE) -C softcore/tools test
test-nav-network:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_nav_packets -o build/nav-packets rtl/motor_status_tx.v softcore/tests/tb_nav_packets.v
	$(VVP) build/nav-packets
test-nav-scheduler:
	mkdir -p build reports
	CCACHE_DISABLE=1 CCACHE_TEMPDIR=/tmp verilator --binary --timing -Wno-fatal --top-module tb_scheduler_nav_stress --Mdir build/nav_stress_verilator -CFLAGS '-std=c++20' camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v rtl/motor_status_tx.v softcore/tests/tb_scheduler_nav_stress.v
	build/nav_stress_verilator/Vtb_scheduler_nav_stress
test-nav-crosswire:
	python3 tests/test_motor_crosswire_nav.py
synth:
	python3 ../../integration/robot_control_v39_rgb/build_fpga.py --synthesis-only

.PHONY: test-sensors test-sensor-network test-mask5
test-sensors:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_sensor_capture -o build/sensor-capture sensors/*.v tests/tb_sensor_capture.v
	$(VVP) build/sensor-capture

test-sensor-network:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_sensor_network -o build/sensor-network imu/imu_ethernet_tx.v sensors/sensor_status_tx.v tests/tb_sensor_network.v
	$(VVP) build/sensor-network

test-mask5:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_rotation_layout_mask5 -o build/rotation-layout-mask5 rtl/motor_remote_control.v tests/tb_rotation_layout_mask5.v
	$(VVP) build/rotation-layout-mask5

.PHONY: test-sensor-mmio
test-sensor-mmio:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_sensor_mmio -o build/sensor-mmio softcore/vendor/picorv32.v softcore/robot_nav_soc.v softcore/tests/tb_sensor_mmio.v
	$(VVP) build/sensor-mmio

.PHONY: test-sensor-scheduler
test-sensor-scheduler:
	mkdir -p build reports
	CCACHE_DISABLE=1 CCACHE_TEMPDIR=/tmp verilator --binary --timing -Wno-fatal --top-module tb_scheduler_sensor_stress --Mdir build/sensor_stress_verilator -CFLAGS '-std=c++20' camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v rtl/motor_status_tx.v sensors/sensor_status_tx.v tests/tb_scheduler_sensor_stress.v
	build/sensor_stress_verilator/Vtb_scheduler_sensor_stress

# New acquisition and transport checks are part of release validation too.
test-release: test-sensors test-sensor-network test-sensor-mmio test-mask5

.PHONY: test-encoder-feedback
test-encoder-feedback:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_encoder_sync -o build/encoder-sync rtl/motor_encoder_sync.v tests/tb_encoder_sync.sv
	$(VVP) build/encoder-sync > reports/encoder-sync-unit.log
	$(IVERILOG) -g2012 -s tb_encoder_motor -o build/encoder-motor rtl/motor_encoder_sync.v rtl/motor_remote_control.v sensors/*.v tests/tb_encoder_motor.sv
	$(VVP) build/encoder-motor > reports/encoder-motor-integration.log
	cat reports/encoder-sync-unit.log reports/encoder-motor-integration.log

test-release: test-encoder-feedback

.PHONY: test-navigation
test-navigation: firmware
	mkdir -p build reports
	$(CC) -std=c99 -Wall -Wextra -Werror -fsanitize=address,undefined -g -o build/test-nav-protocol softcore/tests/test_nav_protocol.c softcore/firmware/astar.c softcore/firmware/nav_protocol.c
	ASAN_OPTIONS=detect_leaks=0 build/test-nav-protocol
	python3 softcore/tests/test_nav_rx.py
	python3 softcore/tests/nav_vectors.py
	CCACHE_DISABLE=1 CCACHE_TEMPDIR=/tmp verilator --binary --timing -Wno-fatal --top-module tb_nav_transport --Mdir build/nav_transport_verilator -CFLAGS '-std=c++20' softcore/vendor/picorv32.v softcore/robot_nav_soc.v imu/imu_ethernet_tx.v rtl/nav_packet_rx.v rtl/nav_message_tx.v softcore/tests/tb_nav_transport.v > build/nav_compile.log 2>&1
	build/nav_transport_verilator/Vtb_nav_transport
	python3 softcore/tests/check_nav_transport.py

.PHONY: test-planner-scheduler
test-planner-scheduler:
	mkdir -p build reports
	CCACHE_DISABLE=1 CCACHE_TEMPDIR=/tmp verilator --binary --timing -Wno-fatal --top-module tb_scheduler_planner_stress --Mdir build/planner_stress_verilator -CFLAGS '-std=c++20' camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v rtl/motor_status_tx.v sensors/sensor_status_tx.v rtl/nav_message_tx.v tests/tb_scheduler_planner_stress.v > build/planner_stress_compile.log 2>&1
	build/planner_stress_verilator/Vtb_scheduler_planner_stress > reports/planner_scheduler.log 2>&1
	cat reports/planner_scheduler.log

# v26 replaces the old rolling-min expectations with the MA contract tests.
# Historical v18/v24/v25 files remain for comparison with their frozen projects.
.PHONY: test-sonar-v26
test-sonar-v26:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_sonar_moving_average_v26 -o build/sonar-ma-v26 sensors/*.v tests/tb_sonar_moving_average_v26.sv
	$(VVP) build/sonar-ma-v26 > reports/sonar-ma-v26.log
	cat reports/sonar-ma-v26.log
	$(IVERILOG) -g2012 -s tb_sonar_capture_v26 -o build/sonar-capture-v26 sensors/*.v tests/tb_sonar_capture_v26.sv
	$(VVP) build/sonar-capture-v26 > reports/sonar-capture-v26.log
	cat reports/sonar-capture-v26.log
	$(IVERILOG) -g2012 -s tb_sonar_uart_v18 -o build/sonar-uart sensors/a02_uart_rx.v tests/tb_sonar_uart_v18.sv
	$(VVP) build/sonar-uart > reports/sonar-uart.log
	cat reports/sonar-uart.log

test-release: test-sonar-v26

# New FIFO and mailbox behavior are required release regressions.
test-release: test-camera-timing

.PHONY: test-timing-system
test-timing-system:
	GOWIN_IDE="$(GOWIN_IDE)" python3 tests/test_robot_rgmii_pipeline.py
	CCACHE_DISABLE=1 CCACHE_TEMPDIR=/tmp verilator --binary --timing -Wno-fatal -GSENSOR_PERIOD_CYCLES=50000 --top-module tb_scheduler_real_fifo --Mdir build/real_fifo_scheduler -CFLAGS '-std=c++20' camera/camera_async_fifo.v camera/eth_fifo_timing.v camera/UDP_Send.v camera/CRC32_D8.v imu/imu_ethernet_tx.v rtl/motor_status_tx.v sensors/sensor_status_tx.v rtl/nav_message_tx.v tests/tb_scheduler_real_fifo.v > build/real_fifo_compile.log 2>&1
	build/real_fifo_scheduler/Vtb_scheduler_real_fifo > reports/real_fifo_scheduler.log 2>&1
	cat reports/real_fifo_scheduler.log
test-release: test-timing-system

.PHONY: test-motor-arc-v21
test-motor-arc-v21:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_motor_arc_v21 -o build/motor-arc-v21 rtl/motor_remote_control.v tests/tb_motor_arc_v21.sv
	$(VVP) build/motor-arc-v21 > reports/motor-arc-v21.log
	cat reports/motor-arc-v21.log
	$(IVERILOG) -g2012 -s tb_packet_arc_rx -o build/packet-arc-v21 rtl/motor_packet_rx.v tests/tb_packet_arc_rx.sv
	$(VVP) build/packet-arc-v21 > reports/packet-arc-v21.log
	cat reports/packet-arc-v21.log
	$(IVERILOG) -g2012 -s tb_motor_legacy_equivalence_v21 -o build/motor-legacy-v21 rtl/motor_encoder_sync.v rtl/motor_remote_control.v tests/motor_remote_control_v20_reference.v tests/tb_motor_legacy_equivalence_v21.sv
	$(VVP) build/motor-legacy-v21 > reports/motor-legacy-equivalence-v21.log
	cat reports/motor-legacy-equivalence-v21.log
test-release: test-motor-arc-v21


.PHONY: test-layered-v28
# Actual v28 control boundaries. Real RV32 firmware scheduler/plant tests are
# in softcore/tests/v28_tb_layered_firmware.sv and firmware verification.
test-layered-v28:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_layered_motor_v28 -o build/layered-motor-v28 rtl/motor_remote_control.v tests/tb_layered_motor_v28.sv
	$(VVP) build/layered-motor-v28 > reports/layered-motor-v28.log
	$(IVERILOG) -g2012 -s tb_layered_network_v28 -o build/layered-network-v28 rtl/motor_packet_rx.v rtl/motor_remote_control.v tests/tb_layered_network_v28.sv
	$(VVP) build/layered-network-v28 > reports/layered-network-v28.log
	$(IVERILOG) -g2012 -s tb_status_ack4_v28 -o build/status-ack4-v28 rtl/motor_status_tx.v tests/tb_status_ack4_v28.sv
	$(VVP) build/status-ack4-v28 > reports/status-ack4-v28.log
	$(IVERILOG) -g2012 -s tb_layered_capture_v28 -o build/layered-capture-v28 sensors/*.v tests/tb_layered_capture_v28.sv
	$(VVP) build/layered-capture-v28 > reports/layered-capture-v28.log
	$(IVERILOG) -g2012 -s tb_layered_mmio_v28 -o build/layered-mmio-v28 softcore/vendor/picorv32.v softcore/robot_nav_soc.v softcore/tests/tb_layered_mmio_v28.sv
	$(VVP) build/layered-mmio-v28 > reports/layered-mmio-v28.log
	cat reports/status-ack4-v28.log reports/layered-motor-v28.log reports/layered-network-v28.log reports/layered-capture-v28.log reports/layered-mmio-v28.log

.PHONY: test-encoder-pipeline-v28
test-encoder-pipeline-v28:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_encoder_pipeline_v28 -o build/encoder-pipeline-v28 rtl/motor_encoder_sync.v tests/motor_encoder_sync_v26_reference.v tests/tb_encoder_pipeline_v28.sv
	$(VVP) build/encoder-pipeline-v28 > reports/encoder-pipeline-v28.log
	cat reports/encoder-pipeline-v28.log
test-layered-v28: test-encoder-pipeline-v28
