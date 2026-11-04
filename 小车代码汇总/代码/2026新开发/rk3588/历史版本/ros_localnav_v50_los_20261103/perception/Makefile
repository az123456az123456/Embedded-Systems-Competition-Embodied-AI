CC ?= cc
.DEFAULT_GOAL := all
CXX ?= c++
PYTHON ?= python3
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Werror
CPPFLAGS += -Isrc -Ivendor -Iassets -Iphone
RKNN ?= 0
FPGA_NAV ?= 0
RGA ?= 0
RGA_INCLUDE ?= /usr/include/rga
WAYLAND_INCLUDE ?= /usr/include
WAYLAND_LIB ?= /usr/lib
RKNN_INCLUDE ?= /usr/include
RKNN_LIB ?= /usr/lib
TARGET ?= build/perception-dashboard
OBJDIR ?= build/obj-$(if $(filter 1,$(RKNN)),rknn,host)

C_SOURCE = src/main.c src/display_output.c src/dashboard.c src/imu_data.c src/sensor_data.c src/display_transform.c vendor/xdg-shell-protocol.c vendor/xdg-output-protocol.c
CXX_SOURCE = src/navigation_map.cc src/vision.cc src/camera.cc src/rgb_camera.cc src/detector.cc src/detector_math.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc
OBJECTS = $(addprefix $(OBJDIR)/,$(C_SOURCE:.c=.o) $(CXX_SOURCE:.cc=.o))
CPPFLAGS += -I$(WAYLAND_INCLUDE)
APP_LDLIBS = -lwayland-client -lm -pthread
APP_LDFLAGS = -L$(WAYLAND_LIB)
# SIMD JPEG is linked into the executable, so the board needs no new library.
# JPEG=stb remains available for a controlled codec comparison.
JPEG ?= turbo
JPEG_ARCH ?= $(if $(filter aarch64%,$(shell $(CC) -dumpmachine 2>/dev/null)),aarch64,x64)
ifeq ($(JPEG),turbo)
CPPFLAGS += -DHAVE_TURBOJPEG -Ivendor/jpeg_turbo/include
JPEG_LIB = vendor/jpeg_turbo/Linux/$(JPEG_ARCH)/libturbojpeg.a
APP_LDLIBS += $(JPEG_LIB)
endif
JPEG_OBJECT = $(OBJDIR)/phone/jpeg_codec-$(JPEG)-$(JPEG_ARCH).o
WRONG_LOG_OBJECT = $(OBJDIR)/common/wrong_log.o
OBJECTS += $(JPEG_OBJECT) $(WRONG_LOG_OBJECT)
ifeq ($(FPGA_NAV),1)
CPPFLAGS += -DHAVE_FPGA_NAV -I../acg138_fpga_map_bridge/include -I../common
FPGA_NAV_OBJECTS = $(addprefix $(OBJDIR)/navigation/,nav_authority.o fpga_delegation.o nva_socket.o phone_nav_adapter.o nav3_wire.o ros_bridge_codec.o nav_exec_wire.o)
OBJECTS += $(FPGA_NAV_OBJECTS)
$(OBJDIR)/navigation/%.o: ../gateway/%.cc
	@mkdir -p "$(@D)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c "$<" -o "$@"
$(OBJDIR)/navigation/nav3_wire.o: ../acg138_fpga_map_bridge/src/nav3_wire.cpp
	@mkdir -p "$(@D)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c "$<" -o "$@"
$(OBJDIR)/navigation/ros_bridge_codec.o: ../common/ros_bridge_codec.cc
	@mkdir -p "$(@D)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c "$<" -o "$@"
$(OBJDIR)/navigation/nav_exec_wire.o: ../gateway/wire/nav_exec_wire.c
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c "$<" -o "$@"
endif
JPEG_CONFIG = $(OBJDIR)/jpeg-config
RGA_CONFIG = $(OBJDIR)/rga-config
$(OBJDIR)/src/detector.o: $(RGA_CONFIG)
.PHONY: FORCE_RGA_CONFIG
$(RGA_CONFIG): FORCE_RGA_CONFIG
	@mkdir -p "$(@D)"
	@printf '%s\n' '$(RGA)-$(RGA_INCLUDE)' | cmp -s - "$@" || printf '%s\n' '$(RGA)-$(RGA_INCLUDE)' > "$@"
$(WRONG_LOG_OBJECT): ../common/wrong_log.cc ../common/wrong_log.h
	@mkdir -p "$(@D)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c "$<" -o "$@"

$(JPEG_OBJECT): phone/jpeg_codec.c phone/jpeg_codec.h
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c "$<" -o "$@"
# Make does not otherwise track command-line CPPFLAGS. Re-link when switching
# codecs, including switching back to an already-existing codec object.
.PHONY: FORCE_JPEG_CONFIG
$(JPEG_CONFIG): FORCE_JPEG_CONFIG
	@mkdir -p "$(@D)"
	@printf '%s\n' '$(JPEG)-$(JPEG_ARCH)' | cmp -s - "$@" || printf '%s\n' '$(JPEG)-$(JPEG_ARCH)' > "$@"
ifeq ($(RKNN),1)
CPPFLAGS += -DHAVE_RKNN -I$(RKNN_INCLUDE)
APP_LDFLAGS += -L$(RKNN_LIB) -Wl,-rpath,'$$ORIGIN/lib'
APP_LDLIBS += -lrknnrt -ldl
endif
ifeq ($(RGA),1)
CPPFLAGS += -DHAVE_RGA -I$(RGA_INCLUDE)
# Resolve the board's existing librga at runtime; an absent/incompatible RGA
# retains cached CPU preprocessing instead of preventing application startup.
APP_LDLIBS += -ldl
endif

.PHONY: all arm64 test test-window test-onnx test-launcher test-stage-timing test-ui-schedule clean
all: $(TARGET)

$(TARGET): $(OBJECTS) $(JPEG_LIB) $(JPEG_CONFIG)
	@mkdir -p "$(@D)"
	$(CXX) $(LDFLAGS) $(APP_LDFLAGS) $(OBJECTS) $(APP_LDLIBS) $(LDLIBS) -o "$@"

$(OBJDIR)/%.o: %.c
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c "$<" -o "$@"

$(OBJDIR)/%.o: %.cc
	@mkdir -p "$(@D)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c "$<" -o "$@"

-include $(OBJECTS:.o=.d)

arm64:
	./scripts/build-arm64.sh

build:
	mkdir -p build

build/test-imu: src/imu_data.c tests/test_imu.c src/imu_data.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/imu_data.c tests/test_imu.c -lm -o "$@"

build/test-imu-bias: src/imu_data.c tests/test_imu_bias.c src/imu_data.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/imu_data.c tests/test_imu_bias.c -lm -o "$@"

build/test-imu-offset: src/imu_data.c tests/test_imu_offset.c src/imu_data.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/imu_data.c tests/test_imu_offset.c -lm -o "$@"

build/test-display-transform: src/display_transform.c tests/test_display_transform.c src/display_transform.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/display_transform.c tests/test_display_transform.c -lm -o "$@"

build/test-camera: src/camera.cc src/rgb_camera.cc src/rgb_camera.h src/camera.h src/stage_clock.h tests/test_camera.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/camera.cc src/rgb_camera.cc tests/test_camera.cc -pthread -o "$@"

build/test-detector: src/detector.cc src/detector_math.cc src/detector_math.h src/detector.h src/stage_clock.h src/model_labels.h tests/test_detector.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector.cc src/detector_math.cc tests/test_detector.cc $(APP_LDFLAGS) $(APP_LDLIBS) $(LDLIBS) -o "$@"

build/test-pipeline: src/vision.cc tests/test_pipeline.cc src/vision.h src/detector.h src/camera.h src/stage_clock.h src/box_motion.h src/model_labels.h src/eye_scheduler.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/vision.cc tests/test_pipeline.cc -pthread -o "$@"

build/test-eye-scheduler: tests/test_eye_scheduler.cc src/eye_scheduler.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_eye_scheduler.cc -o "$@"
.PHONY: test-scheduler-v34
test-scheduler-v34: build/test-eye-scheduler build/test-pipeline
	./build/test-eye-scheduler
	./build/test-pipeline --scheduler-v34

build/test-preprocess: src/detector_math.cc tests/test_preprocess.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/test_preprocess.cc -o "$@"

test: all build/test-imu build/test-imu-bias build/test-imu-offset build/test-display-transform build/test-camera build/test-detector build/test-pipeline build/test-preprocess build/test-ui-schedule build/test-box-motion build/test-sensors
	./build/test-imu
	./build/test-imu-bias
	./build/test-imu-offset
	./build/test-display-transform
	./build/test-camera
	./build/test-detector
	./build/test-pipeline
	./build/test-preprocess
	./build/test-ui-schedule
	./build/test-box-motion
	./build/test-sensors
	./$(TARGET) --preview build/preview.ppm
	./$(TARGET) --preview build/waiting.ppm --waiting

build/test-wayland: tests/test_wayland.c vendor/xdg-shell-protocol.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_wayland.c vendor/xdg-shell-protocol.c -lwayland-server -o "$@"

test-window: all build/test-wayland
	PERCEPTION_TEST_APP="$(abspath $(TARGET))" $(PYTHON) tests/check_wayland.py

# Multi-output Weston startup without any input seat. The fixture advertises
# wl_output v3 and identifies HDMI solely through matching xdg-output names.
build/test-hdmi-wayland: tests/test_hdmi_wayland.c vendor/xdg-shell-protocol.c vendor/xdg-output-protocol.c vendor/xdg-output-server-protocol.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_hdmi_wayland.c vendor/xdg-shell-protocol.c vendor/xdg-output-protocol.c -lwayland-server -o "$@"

.PHONY: test-hdmi-window
test-hdmi-window: all build/test-hdmi-wayland
	PERCEPTION_TEST_APP="$(abspath $(TARGET))" $(PYTHON) tests/check_hdmi_wayland.py

build/onnx-decode-probe: tests/onnx_decode_probe.cc src/detector_math.cc src/detector_math.h src/detector.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/onnx_decode_probe.cc -o "$@"

test-onnx: build/onnx-decode-probe
	$(PYTHON) tests/test_onnx_pipeline.py --probe build/onnx-decode-probe $(ONNX_TEST_ARGS)

test-launcher:
	$(PYTHON) tests/test_launcher.py

# Real detector code against a fake SDK: timings are tested, not hardware speed.
RKNN_TEST_INCLUDE ?= $(abspath ../../../YOLO/rknn_model_zoo/3rdparty/rknpu2/include)
test-stage-timing: | build
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RKNN -I$(RKNN_TEST_INCLUDE) src/detector.cc src/detector_math.cc tests/test_stage_timing.cc -rdynamic -ldl -pthread -o build/test-stage-timing
	./build/test-stage-timing

clean:
	rm -rf build

build/test-preprocess-plan: src/detector_math.cc src/preprocess_plan.h src/preprocess_gather_v10.h tests/test_preprocess_plan.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/test_preprocess_plan.cc -o $@
build/test-camera-fast: src/camera.cc src/rgb_camera.cc src/rgb_camera.h src/camera_color_math.h src/camera_timestamp.h tests/test_camera_fast.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/camera.cc src/rgb_camera.cc tests/test_camera_fast.cc -pthread -o $@
build/bench-camera: src/camera.cc src/rgb_camera.cc src/rgb_camera.h src/camera_color_math.h src/camera_timestamp.h tests/bench_camera.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/camera.cc src/rgb_camera.cc tests/bench_camera.cc -pthread -o $@

build/test-fpga-rgb: src/rgb_camera.cc src/rgb_camera.h src/fpga_rgb_input.h tests/test_fpga_rgb.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/rgb_camera.cc tests/test_fpga_rgb.cc -pthread -o $@
.PHONY: test-fpga-rgb
test-fpga-rgb: build/test-fpga-rgb
	./build/test-fpga-rgb
build/test-fpga-follow-geometry: phone/controller.cc phone/follow_geometry.cc tests/test_fpga_follow_geometry.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) phone/controller.cc phone/follow_geometry.cc tests/test_fpga_follow_geometry.cc -o $@
.PHONY: test-fpga-follow-geometry
test-fpga-follow-geometry: build/test-fpga-follow-geometry
	./build/test-fpga-follow-geometry
build/rgb-codec-probe: src/rgb_camera.cc src/rgb_camera.h src/rgb_capture_files.h tools/rgb_codec_probe.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/rgb_camera.cc tools/rgb_codec_probe.cc -pthread -o $@
build/capture-fpga-rgb: src/rgb_camera.cc src/rgb_camera.h src/rgb_capture_files.h tools/capture_fpga_rgb.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/rgb_camera.cc tools/capture_fpga_rgb.cc -pthread -o $@
build/camera-link-probe: src/rgb_camera.cc src/camera_link_protocol.h tools/camera_link_probe.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/rgb_camera.cc tools/camera_link_probe.cc -pthread -o $@
build/test-camera-link: tests/test_camera_link_protocol.cc src/camera_link_protocol.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_camera_link_protocol.cc -o $@
build/test-stream-deadline: tests/test_stream_deadline.cc phone/http_server.cc phone/http_server.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_stream_deadline.cc phone/http_server.cc -pthread -o $@
test-fast: build/test-preprocess-plan build/test-camera-fast
	./build/test-preprocess-plan --benchmark
	./build/test-camera-fast
test-input-memory: | build
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RKNN -I$(RKNN_TEST_INCLUDE) src/detector.cc src/detector_math.cc tests/test_input_memory.cc -rdynamic -ldl -pthread -o build/test-input-memory
	./build/test-input-memory
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RKNN -DNO_IO_API -I$(RKNN_TEST_INCLUDE) src/detector.cc src/detector_math.cc tests/test_input_memory.cc -rdynamic -ldl -pthread -o build/test-input-memory-no-api
	./build/test-input-memory-no-api

build/test-preprocess-rga: tests/test_preprocess_rga.cc src/preprocess_rga.h src/preprocess_plan.h src/detector_math.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/test_preprocess_rga.cc $(APP_LDFLAGS) $(APP_LDLIBS) -o "$@"
.PHONY: test-preprocess-rga
test-preprocess-rga: build/test-preprocess-rga build/test-preprocess-plan
	./build/test-preprocess-rga
	./build/test-preprocess-plan

build/bench-preprocess-v34: tests/bench_preprocess_v34.cc src/preprocess_plan.h src/detector_math.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/bench_preprocess_v34.cc -pthread -o "$@"

.PHONY: test-rga-native test-rga-detector
test-rga-native: | build
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RGA -I$(RGA_INCLUDE) src/detector_math.cc tests/test_rga_native.cc -Wl,--wrap=dlopen,--wrap=dlclose,--wrap=dlsym,--wrap=open,--wrap=close,--wrap=ioctl,--wrap=mmap,--wrap=munmap -ldl -o build/test-rga-native
	./build/test-rga-native
test-rga-detector: | build
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RKNN -DHAVE_RGA -I$(RKNN_TEST_INCLUDE) -I$(RGA_INCLUDE) src/detector.cc src/detector_math.cc tests/test_input_memory.cc -rdynamic -ldl -pthread -o build/test-input-memory-rga
	./build/test-input-memory-rga

test-ui-render:
	sh tests/run_ui_render_test.sh --benchmark

build/test-ui-schedule: tests/test_ui_schedule.c src/ui_schedule.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_ui_schedule.c -o "$@"
test-ui-schedule: build/test-ui-schedule
	./build/test-ui-schedule

build/test-phone-controller: phone/controller.cc phone/follow_geometry.cc tests/test_phone_controller.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) phone/controller.cc phone/follow_geometry.cc tests/test_phone_controller.cc -o "$@"
build/test-phone-wire: phone/json_flat.cc phone/http_server.cc phone/motor_link.cc tests/test_phone_wire.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) phone/json_flat.cc phone/http_server.cc phone/motor_link.cc tests/test_phone_wire.cc -pthread -o "$@"
test-phone: all build/test-phone-controller build/test-phone-wire
	./build/test-phone-controller
	./build/test-phone-wire
	$(PYTHON) tests/test_phone_api.py

build/phone-publish-imu.o: src/imu_data.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -c src/imu_data.c -o "$@"
build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o: phone/jpeg_codec.c phone/jpeg_codec.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -c phone/jpeg_codec.c -o "$@"
build/phone-publish-probe: tests/phone_publish_probe.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_CONFIG) $(WRONG_LOG_OBJECT) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/phone_publish_probe.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_LIB) $(WRONG_LOG_OBJECT) -pthread -lm -o "$@"
test-phone-publish: build/phone-publish-probe
	$(PYTHON) tests/test_phone_publish.py

build/phone-partial-probe: tests/phone_partial_probe.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_CONFIG) $(WRONG_LOG_OBJECT) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/phone_partial_probe.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_LIB) $(WRONG_LOG_OBJECT) -pthread -lm -o "$@"
.PHONY: test-phone-partial
test-phone-partial: build/phone-partial-probe
	$(PYTHON) tests/test_phone_partial.py

build/phone-rate-probe: tests/phone_rate_probe.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_CONFIG) $(WRONG_LOG_OBJECT) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/phone_rate_probe.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_LIB) $(WRONG_LOG_OBJECT) -pthread -lm -Wl,--wrap=jpeg_from_argb -o "$@"
.PHONY: test-phone-rate
test-phone-rate: build/phone-rate-probe
	$(PYTHON) tests/test_phone_rate.py

build/phone-attach-probe: tests/phone_attach_probe.cc tests/test_pipeline.cc src/vision.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_CONFIG) $(WRONG_LOG_OBJECT) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/phone_attach_probe.cc src/vision.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_LIB) $(WRONG_LOG_OBJECT) -pthread -lm -o "$@"
.PHONY: test-phone-attach
test-phone-attach: build/phone-attach-probe
	$(PYTHON) tests/test_phone_attach.py

build/test-box-motion: tests/test_box_motion.cc src/box_motion.h src/vision.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_box_motion.cc -o "$@"
.PHONY: test-box-motion
test-box-motion: build/test-box-motion
	./build/test-box-motion

test-jpeg:
	$(PYTHON) tests/test_jpeg_codec.py

.PHONY: test-sensors
build/test-sensors: src/sensor_data.c src/sensor_data.h ../common/sonar_range.h tests/test_sensors.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/sensor_data.c tests/test_sensors.c -o "$@"
build/sensor-decode-probe: src/sensor_data.c src/sensor_data.h ../common/sonar_range.h tests/sensor_decode_probe.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/sensor_data.c tests/sensor_decode_probe.c -o "$@"
test-sensors: build/test-sensors build/sensor-decode-probe
	./build/test-sensors

build/test-sonar-display: src/sensor_data.c src/sensor_data.h ../common/sonar_range.h tests/test_sonar_display.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/sensor_data.c tests/test_sonar_display.c -o "$@"
.PHONY: test-sonar-display
test-sonar-display: build/test-sonar-display
	./build/test-sonar-display

build/test-sonar-observation-age: src/sensor_data.c src/sensor_data.h tests/test_sonar_observation_age.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) src/sensor_data.c tests/test_sonar_observation_age.c -o "$@"
.PHONY: test-sonar-observation-age
test-sonar-observation-age: build/test-sonar-observation-age
	./build/test-sonar-observation-age

build/test-sensor-ui: tests/test_sensor_ui.c src/dashboard.c src/dashboard.h src/sensor_data.c src/sensor_data.h src/imu_data.c src/display_transform.c assets/font_atlas.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_sensor_ui.c src/dashboard.c src/sensor_data.c src/imu_data.c src/display_transform.c -lm -o "$@"
.PHONY: test-sensor-ui
test-sensor-ui: build/test-sensor-ui
	./build/test-sensor-ui

build/test-navigation-map: tests/test_navigation_map.cc src/navigation_map.cc src/navigation_map.h vendor/picojson.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_navigation_map.cc src/navigation_map.cc -pthread -o "$@"
build/test-navigation-ui: tests/test_navigation_ui.c src/dashboard.c src/dashboard.h src/navigation_render.h src/navigation_map.h src/sensor_data.c src/imu_data.c src/display_transform.c assets/font_atlas.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_navigation_ui.c src/dashboard.c src/sensor_data.c src/imu_data.c src/display_transform.c -lm -o "$@"
.PHONY: test-navigation
test-navigation: build/test-navigation-map build/test-navigation-ui
	./build/test-navigation-map
	./build/test-navigation-ui

.PHONY: test-unified-navigation
test-unified-navigation: all build/test-wayland
	mkdir -p build/android-protocol
	javac -encoding UTF-8 --release 8 -d build/android-protocol tests/android-protocol/MultipartReader.java tests/android-protocol/ProtocolProbe.java
	$(PYTHON) tests/test_unified_window.py

build/phone-publish-sensors.o: src/sensor_data.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -c src/sensor_data.c -o "$@"

build/test-angle-follow: tests/test_angle_follow.cc phone/controller.cc phone/follow_geometry.cc phone/motor_link.cc phone/follow_sensors.h build/phone-publish-sensors.o | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_angle_follow.cc phone/controller.cc phone/follow_geometry.cc phone/motor_link.cc build/phone-publish-sensors.o -o "$@"
test-angle-follow: build/test-angle-follow
	./build/test-angle-follow

build/test-custom-pipeline: src/vision.cc tests/test_custom_pipeline.cc src/model_labels.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/vision.cc tests/test_custom_pipeline.cc -pthread -o "$@"
.PHONY: test-custom-model
test-custom-model: build/test-detector build/test-custom-pipeline
	./build/test-detector
	./build/test-custom-pipeline

build/test-manual-arc: tests/test_manual_arc.cc phone/controller.cc phone/follow_geometry.cc phone/motor_link.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_manual_arc.cc phone/controller.cc phone/follow_geometry.cc phone/motor_link.cc -o "$@"
.PHONY: test-manual-arc
test-manual-arc: build/test-manual-arc
	./build/test-manual-arc

.PHONY: test-wronglog
build/test-control-audit: tests/test_control_audit.cc phone/control_audit.h src/wronglog_state.h phone/controller.cc phone/follow_geometry.cc phone/motor_link.cc phone/json_flat.cc build/phone-publish-imu.o build/phone-publish-sensors.o $(WRONG_LOG_OBJECT) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_control_audit.cc phone/controller.cc phone/follow_geometry.cc phone/motor_link.cc phone/json_flat.cc build/phone-publish-imu.o build/phone-publish-sensors.o $(WRONG_LOG_OBJECT) -pthread -lm -o "$@"
test-wronglog: all build/test-control-audit
	./build/test-control-audit
	$(PYTHON) tests/test_wronglog_runtime.py

.PHONY: test-control-clock
build/test-control-clock-race: tests/test_control_clock_race.cc phone/phone_service.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(WRONG_LOG_OBJECT) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_control_clock_race.cc phone/controller.cc phone/follow_geometry.cc phone/http_server.cc phone/json_flat.cc phone/motor_link.cc build/phone-publish-imu.o build/phone-publish-sensors.o build/phone-publish-jpeg-$(JPEG)-$(JPEG_ARCH).o $(JPEG_LIB) $(WRONG_LOG_OBJECT) -Wl,--wrap=pthread_mutex_lock,--wrap=pthread_mutex_unlock,--wrap=imu_clock_ms -pthread -lm -o "$@"
test-control-clock: build/test-control-clock-race
	FOLLOW_GEOMETRY_FILE=tests/synthetic-bearing-fixture.json ./build/test-control-clock-race

.PHONY: test-velocity
build/test-velocity-control: phone/controller.cc phone/follow_geometry.cc tests/test_velocity_control.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) phone/controller.cc phone/follow_geometry.cc tests/test_velocity_control.cc -o "$@"
test-velocity: build/test-velocity-control build/test-phone-wire
	./build/test-velocity-control
	./build/test-phone-wire

build/test-softcore-nav1: phone/motor_link.cc phone/motor_link.h phone/controller.cc phone/follow_geometry.cc tests/test_softcore_nav1.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) phone/motor_link.cc phone/controller.cc phone/follow_geometry.cc tests/test_softcore_nav1.cc -Wl,--wrap=socket,--wrap=bind,--wrap=ioctl,--wrap=sendto,--wrap=recvfrom,--wrap=close,--wrap=if_nametoindex -o $@
.PHONY: test-softcore
test-softcore: build/test-softcore-nav1
	./build/test-softcore-nav1

build/test-fpga-rgb-open: src/rgb_camera.cc src/rgb_camera.h tests/test_fpga_rgb_open.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/rgb_camera.cc tests/test_fpga_rgb_open.cc -pthread -Wl,--wrap=socket,--wrap=ioctl,--wrap=bind,--wrap=setsockopt,--wrap=getsockopt,--wrap=sendto,--wrap=close -o $@
.PHONY: test-fpga-rgb-open
test-fpga-rgb-open: build/test-fpga-rgb-open
	./build/test-fpga-rgb-open

build/test-stream-rate: tests/test_stream_rate.cc phone/stream_rate.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_stream_rate.cc -o $@
.PHONY: test-stream-rate
test-stream-rate: build/test-stream-rate
	./build/test-stream-rate
