IVERILOG ?= iverilog
VVP ?= vvp
GOWIN_IDE ?= ../../Gowin_V1.9.12.03/IDE
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
	@test -f "$(GOWIN_IDE)/simlib/gw5a/prim_sim.v" || { echo "找不到高云功能模型，请设置 GOWIN_IDE=/实际高云安装目录/IDE" >&2; exit 1; }
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_rgmii_rx -o build/rgmii rtl/motor_packet_rx.v tests/tb_rgmii_rx.v "$(GOWIN_IDE)/simlib/gw5a/prim_sim.v"
	$(VVP) build/rgmii
test-real-clock:
	mkdir -p build reports
	$(IVERILOG) -g2012 -s tb_real_clock -o build/real_clock rtl/motor_remote_control.v tests/tb_real_clock.v
	$(VVP) build/real_clock
clean:
	rm -rf build
