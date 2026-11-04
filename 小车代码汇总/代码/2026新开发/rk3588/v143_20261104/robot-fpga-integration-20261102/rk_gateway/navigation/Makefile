CXX ?= g++
CC ?= gcc
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -pthread
FPGA_FIRMWARE ?= ../../../fpga/robot_control_v39_rgb/softcore/firmware
CXXFLAGS += -I$(FPGA_FIRMWARE)
ARM_PREFIX ?= ../../../bsp_r4/ACG138-Offline-BSP-20260923-r4/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-
WRONG_LOG_OBJ = build/wrong-log.o
SOURCES = main.cc navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc
.PHONY: all arm64 test clean
all: build/robot-navigation
build:
	mkdir -p build reports
build/robot-navigation: $(SOURCES) navcore.h navcore_reactive.inc mjpeg_proxy.h imu_alignment.h sensor_bridge.h vehicle_geometry.h protocol.h gateway.h map_snapshot.h sonar_diagnostics.h ../common/wrong_log.h ../common/sonar_range.h $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) $(SOURCES) $(WRONG_LOG_OBJ) -o $@
arm64: | build
	$(ARM_PREFIX)g++ $(CXXFLAGS) $(SOURCES) ../common/wrong_log.cc -Wl,--build-id -o build/robot-navigation.aarch64
test: build/robot-navigation $(WRONG_LOG_OBJ) | build
	mkdir -p reports
	$(CXX) $(CXXFLAGS) protocol.cc tests/test_protocol.cc $(WRONG_LOG_OBJ) -o build/test-protocol
	./build/test-protocol
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_navcore.cc $(WRONG_LOG_OBJ) -o build/test-navcore
	./build/test-navcore
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_sonar_censored.cc $(WRONG_LOG_OBJ) -o build/test-sonar-censored
	./build/test-sonar-censored
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_vehicle_profile.cc $(WRONG_LOG_OBJ) -o build/test-vehicle-profile
	./build/test-vehicle-profile
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_runtime_profile_state.cc $(WRONG_LOG_OBJ) -o build/test-runtime-profile-state
	./build/test-runtime-profile-state
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_input_pipeline_replay.cc $(WRONG_LOG_OBJ) -o build/test-input-pipeline-replay
	./build/test-input-pipeline-replay
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc tests/test_imu_alignment.cc $(WRONG_LOG_OBJ) -o build/test-imu-alignment
	./build/test-imu-alignment
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_localization_diagnostics.cc $(WRONG_LOG_OBJ) -o build/test-localization-diagnostics
	./build/test-localization-diagnostics
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc tests/test_sensor_bridge.cc $(WRONG_LOG_OBJ) -o build/test-sensor-bridge
	./build/test-sensor-bridge
	$(CXX) $(CXXFLAGS) tests/test_sonar_diagnostics.cc $(WRONG_LOG_OBJ) -o build/test-sonar-diagnostics
	./build/test-sonar-diagnostics
	$(CXX) $(CXXFLAGS) navcore.cc map_snapshot.cc tests/test_map_snapshot.cc $(WRONG_LOG_OBJ) -o build/test-map-snapshot
	./build/test-map-snapshot
	$(CXX) $(CXXFLAGS) navcore.cc map_snapshot.cc tests/test_map_goal.cc $(WRONG_LOG_OBJ) -o build/test-map-goal
	./build/test-map-goal
	$(CXX) $(CXXFLAGS) gateway.cc vendor/http_server.cc vendor/json_flat.cc tests/test_gateway.cc $(WRONG_LOG_OBJ) -o build/test-gateway
	./build/test-gateway
	$(CXX) $(CXXFLAGS) gateway.cc vendor/http_server.cc vendor/json_flat.cc tests/test_gateway_straight.cc $(WRONG_LOG_OBJ) -o build/test-gateway-straight
	./build/test-gateway-straight
	$(CXX) $(CXXFLAGS) gateway.cc vendor/http_server.cc vendor/json_flat.cc tests/test_gateway_source_clock.cc $(WRONG_LOG_OBJ) -o build/test-gateway-source-clock
	./build/test-gateway-source-clock
	$(CXX) $(CXXFLAGS) gateway.cc vendor/http_server.cc vendor/json_flat.cc tests/test_gateway_concurrency.cc $(WRONG_LOG_OBJ) -o build/test-gateway-concurrency
	./build/test-gateway-concurrency
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_gateway_core_handoff.cc $(WRONG_LOG_OBJ) -o build/test-gateway-core-handoff
	./build/test-gateway-core-handoff
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_gateway_preflight.cc $(WRONG_LOG_OBJ) -o build/test-gateway-preflight
	./build/test-gateway-preflight
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_wronglog_navigation.cc $(WRONG_LOG_OBJ) -o build/test-wronglog-navigation
	./build/test-wronglog-navigation
	$(CXX) $(CXXFLAGS) gateway.cc vendor/http_server.cc vendor/json_flat.cc tests/test_gateway_observation_barrier.cc $(WRONG_LOG_OBJ) -o build/test-gateway-observation-barrier
	./build/test-gateway-observation-barrier
	$(CC) -O2 -c $(FPGA_FIRMWARE)/astar.c -o build/astar-host.o
	$(CC) -O2 -c $(FPGA_FIRMWARE)/nav_protocol.c -o build/firmware-protocol-host.o
	$(CXX) $(CXXFLAGS) -DNAV_TEST_TRANSPORT protocol.cc tests/test_firmware_client.cc build/astar-host.o build/firmware-protocol-host.o $(WRONG_LOG_OBJ) -o build/test-firmware-client
	./build/test-firmware-client
	$(CXX) $(CXXFLAGS) navcore.cc integration/test_actual_astar_drive.cc build/astar-host.o $(WRONG_LOG_OBJ) -o build/test-actual-astar-drive
	./build/test-actual-astar-drive
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_bootstrap_navigation.cc build/astar-host.o $(WRONG_LOG_OBJ) -o build/test-bootstrap-navigation
	./build/test-bootstrap-navigation
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_straight_assist.cc build/astar-host.o $(WRONG_LOG_OBJ) -o build/test-straight-assist
	./build/test-straight-assist
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_straight_runtime.cc $(WRONG_LOG_OBJ) -o build/test-straight-runtime
	./build/test-straight-runtime
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_navigation_timeline.cc build/astar-host.o $(WRONG_LOG_OBJ) -o build/test-navigation-timeline
	./build/test-navigation-timeline > reports/navigation-timeline.csv
	tail -n 1 reports/navigation-timeline.csv
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_encoder_window.cc $(WRONG_LOG_OBJ) -o build/test-encoder-window
	./build/test-encoder-window
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc ../dashboard/phone/controller.cc ../dashboard/phone/follow_geometry.cc ../dashboard/phone/motor_link.cc integration/test_controller_navigation_lifecycle.cc $(WRONG_LOG_OBJ) -o build/test-controller-navigation-lifecycle
	./build/test-controller-navigation-lifecycle > reports/controller-navigation-lifecycle.log
	tail -n 1 reports/controller-navigation-lifecycle.log
	python3 tests/test_runtime.py
	python3 tests/test_unified_launcher.py
	python3 tests/test_unified_launcher.py --busybox
	python3 tests/test_vehicle_profile.py
clean:
	rm -rf build

.PHONY: test-phone-timing test-phone-timing-old
test-phone-timing:
	./integration/run-phone-timing.sh new
test-phone-timing-old:
	./integration/run-phone-timing.sh old

build/wrong-log.o: ../common/wrong_log.cc ../common/wrong_log.h | build
	$(CXX) $(CXXFLAGS) -c ../common/wrong_log.cc -o $@

.PHONY: test-wronglog-isolation
build/test-wronglog-format-isolation: tests/test_wronglog_format_isolation.cc main.cc gateway.cc $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) -Wno-mismatched-new-delete navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_wronglog_format_isolation.cc $(WRONG_LOG_OBJ) -o $@
test-wronglog-isolation: build/test-wronglog-format-isolation
	./build/test-wronglog-format-isolation

.PHONY: test-straight
build/astar-host.o: $(FPGA_FIRMWARE)/astar.c $(FPGA_FIRMWARE)/astar.h | build
	$(CC) -O2 -c $< -o $@
test-straight: build/robot-navigation $(WRONG_LOG_OBJ) build/astar-host.o
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_straight_assist.cc build/astar-host.o $(WRONG_LOG_OBJ) -o build/test-straight-assist
	./build/test-straight-assist
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_straight_runtime.cc $(WRONG_LOG_OBJ) -o build/test-straight-runtime
	./build/test-straight-runtime

# Resume the runtime/export regressions without recompiling the already-tested
# core/HTTP suite; also works in a clean source copy with no reports directory.
.PHONY: test-runtime-tail
test-runtime-tail: build/robot-navigation $(WRONG_LOG_OBJ) build/astar-host.o
	mkdir -p reports
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_navigation_timeline.cc build/astar-host.o $(WRONG_LOG_OBJ) -o build/test-navigation-timeline
	./build/test-navigation-timeline > reports/navigation-timeline.csv
	tail -n 1 reports/navigation-timeline.csv
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_encoder_window.cc $(WRONG_LOG_OBJ) -o build/test-encoder-window
	./build/test-encoder-window
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc ../dashboard/phone/controller.cc ../dashboard/phone/follow_geometry.cc ../dashboard/phone/motor_link.cc integration/test_controller_navigation_lifecycle.cc $(WRONG_LOG_OBJ) -o build/test-controller-navigation-lifecycle
	./build/test-controller-navigation-lifecycle > reports/controller-navigation-lifecycle.log
	tail -n 1 reports/controller-navigation-lifecycle.log
	python3 tests/test_runtime.py
	python3 tests/test_unified_launcher.py
	python3 tests/test_unified_launcher.py --busybox
	python3 tests/test_vehicle_profile.py

.PHONY: test-layered
build/test-controller-navigation-lifecycle: $(SOURCES) integration/test_controller_navigation_lifecycle.cc ../dashboard/phone/motor_link.cc ../dashboard/phone/motor_link.h ../dashboard/phone/controller.cc $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc ../dashboard/phone/controller.cc ../dashboard/phone/follow_geometry.cc ../dashboard/phone/motor_link.cc integration/test_controller_navigation_lifecycle.cc $(WRONG_LOG_OBJ) -o $@
.PHONY: test-controller-lifecycle
test-controller-lifecycle: build/test-controller-navigation-lifecycle
	./build/test-controller-navigation-lifecycle > reports/controller-navigation-lifecycle.log
	tail -n 1 reports/controller-navigation-lifecycle.log
build/test-reactive-avoidance: navcore.cc navcore_reactive.inc tests/test_reactive_avoidance.cc $(WRONG_LOG_OBJ) build/astar-host.o | build
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_reactive_avoidance.cc build/astar-host.o $(WRONG_LOG_OBJ) -o $@
build/test-reactive-velocity: navcore.cc navcore_reactive.inc tests/test_reactive_avoidance.cc $(WRONG_LOG_OBJ) build/astar-host.o | build
	$(CXX) $(CXXFLAGS) -DV28_TEST_VELOCITY navcore.cc tests/test_reactive_avoidance.cc build/astar-host.o $(WRONG_LOG_OBJ) -o $@
test-layered: build/test-reactive-avoidance build/test-reactive-velocity
	./build/test-reactive-avoidance > reports/reactive-legacy.log
	./build/test-reactive-velocity > reports/reactive-velocity.log
	tail -n 1 reports/reactive-legacy.log
	tail -n 1 reports/reactive-velocity.log

.PHONY: test-local-rotation
build/test-local-rotation: navcore.cc navcore.h navcore_reactive.inc tests/test_local_rotation_map.cc tests/fixtures/local-rotation-map-20261027.json $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_local_rotation_map.cc $(WRONG_LOG_OBJ) -o $@
test-local-rotation: build/test-local-rotation
	./build/test-local-rotation > reports/local-rotation-map.log
	tail -n 1 reports/local-rotation-map.log

build/test-local-guard-status: $(SOURCES) tests/test_local_guard_status.cc tests/test_local_rotation_map.cc $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) navcore.cc imu_alignment.cc sensor_bridge.cc protocol.cc gateway.cc map_snapshot.cc vendor/http_server.cc vendor/json_flat.cc tests/test_local_guard_status.cc $(WRONG_LOG_OBJ) -o $@
.PHONY: test-local-guard-status
test-local-guard-status: build/test-local-guard-status
	./build/test-local-guard-status > reports/local-guard-status.log
	tail -n 1 reports/local-guard-status.log

.PHONY: test-sonar-policy
build/test-sonar-policy: navcore.cc navcore.h navcore_reactive.inc tests/test_sonar_avoidance_policy.cc tests/fixtures/sonar-policy-blocker-20210101.json $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_sonar_avoidance_policy.cc $(WRONG_LOG_OBJ) -o $@
test-sonar-policy: build/test-sonar-policy
	./build/test-sonar-policy > reports/sonar-policy.log
	tail -n 1 reports/sonar-policy.log

.PHONY: test-three-sonar-policy
build/test-three-sonar-policy: navcore.cc navcore.h navcore_reactive.inc tests/test_three_sonar_policy.cc tests/fixtures/three-sonar-real-20261028.json $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_three_sonar_policy.cc $(WRONG_LOG_OBJ) -o $@
test-three-sonar-policy: build/test-three-sonar-policy
	./build/test-three-sonar-policy > reports/three-sonar-policy.log
	tail -n 1 reports/three-sonar-policy.log

.PHONY: test-reactive-safety-v32
build/test-reactive-safety-v32: navcore.cc navcore.h navcore_reactive.inc tests/test_reactive_safety_v32.cc $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_reactive_safety_v32.cc $(WRONG_LOG_OBJ) -o $@
test-reactive-safety-v32: build/test-reactive-safety-v32
	./build/test-reactive-safety-v32 > reports/reactive-safety-v32.log
	tail -n 1 reports/reactive-safety-v32.log

.PHONY: test-reactive-safety-v33
build/test-reactive-safety-v33: navcore.cc navcore.h navcore_reactive.inc tests/test_reactive_safety_v33.cc $(WRONG_LOG_OBJ) | build
	$(CXX) $(CXXFLAGS) navcore.cc tests/test_reactive_safety_v33.cc $(WRONG_LOG_OBJ) -o $@
test-reactive-safety-v33: build/test-reactive-safety-v33
	./build/test-reactive-safety-v33 > reports/reactive-safety-v33.log
	tail -n 1 reports/reactive-safety-v33.log
