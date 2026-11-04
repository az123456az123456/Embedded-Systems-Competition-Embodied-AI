CC ?= cc
CXX ?= c++
PYTHON ?= python3
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Werror
CPPFLAGS += -Isrc -Ivendor -Iassets
RKNN ?= 0
WAYLAND_INCLUDE ?= /usr/include
WAYLAND_LIB ?= /usr/lib
RKNN_INCLUDE ?= /usr/include
RKNN_LIB ?= /usr/lib
TARGET ?= build/perception-dashboard
OBJDIR ?= build/obj-$(if $(filter 1,$(RKNN)),rknn,host)

C_SOURCE = src/main.c src/dashboard.c src/imu_data.c src/display_transform.c vendor/xdg-shell-protocol.c
CXX_SOURCE = src/vision.cc src/camera.cc src/detector.cc src/detector_math.cc
OBJECTS = $(addprefix $(OBJDIR)/,$(C_SOURCE:.c=.o) $(CXX_SOURCE:.cc=.o))
CPPFLAGS += -I$(WAYLAND_INCLUDE)
APP_LDLIBS = -lwayland-client -lm -pthread
APP_LDFLAGS = -L$(WAYLAND_LIB)
ifeq ($(RKNN),1)
CPPFLAGS += -DHAVE_RKNN -I$(RKNN_INCLUDE)
APP_LDFLAGS += -L$(RKNN_LIB) -Wl,-rpath,'$$ORIGIN/lib'
APP_LDLIBS += -lrknnrt -ldl
endif

.PHONY: all arm64 test test-window test-onnx test-launcher test-stage-timing test-ui-schedule clean
all: $(TARGET)

$(TARGET): $(OBJECTS)
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

build/test-camera: src/camera.cc src/camera.h src/stage_clock.h tests/test_camera.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/camera.cc tests/test_camera.cc -pthread -o "$@"

build/test-detector: src/detector.cc src/detector_math.cc src/detector_math.h src/detector.h src/stage_clock.h tests/test_detector.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector.cc src/detector_math.cc tests/test_detector.cc $(APP_LDFLAGS) $(APP_LDLIBS) $(LDLIBS) -o "$@"

build/test-pipeline: src/vision.cc tests/test_pipeline.cc src/vision.h src/detector.h src/camera.h src/stage_clock.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/vision.cc tests/test_pipeline.cc -pthread -o "$@"

build/test-preprocess: src/detector_math.cc tests/test_preprocess.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/test_preprocess.cc -o "$@"

test: all build/test-imu build/test-imu-bias build/test-imu-offset build/test-display-transform build/test-camera build/test-detector build/test-pipeline build/test-preprocess build/test-ui-schedule
	./build/test-imu
	./build/test-imu-bias
	./build/test-imu-offset
	./build/test-display-transform
	./build/test-camera
	./build/test-detector
	./build/test-pipeline
	./build/test-preprocess
	./build/test-ui-schedule
	./$(TARGET) --preview build/preview.ppm
	./$(TARGET) --preview build/waiting.ppm --waiting

build/test-wayland: tests/test_wayland.c vendor/xdg-shell-protocol.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_wayland.c vendor/xdg-shell-protocol.c -lwayland-server -o "$@"

test-window: all build/test-wayland
	PERCEPTION_TEST_APP="$(abspath $(TARGET))" $(PYTHON) tests/check_wayland.py

build/onnx-decode-probe: tests/onnx_decode_probe.cc src/detector_math.cc src/detector_math.h src/detector.h | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/onnx_decode_probe.cc -o "$@"

test-onnx: build/onnx-decode-probe
	$(PYTHON) tests/test_onnx_pipeline.py --probe build/onnx-decode-probe $(ONNX_TEST_ARGS)

test-launcher:
	$(PYTHON) tests/test_launcher.py

# Real detector code against a fake SDK: timings are tested, not hardware speed.
RKNN_TEST_INCLUDE ?= ../../YOLO/rknn_model_zoo/3rdparty/rknpu2/include
test-stage-timing: | build
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RKNN -I$(RKNN_TEST_INCLUDE) src/detector.cc src/detector_math.cc tests/test_stage_timing.cc -rdynamic -ldl -pthread -o build/test-stage-timing
	./build/test-stage-timing

clean:
	rm -rf build

build/test-preprocess-plan: src/detector_math.cc src/preprocess_plan.h src/preprocess_gather_v10.h tests/test_preprocess_plan.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector_math.cc tests/test_preprocess_plan.cc -o $@
build/test-camera-fast: src/camera.cc src/camera_color_math.h src/camera_timestamp.h tests/test_camera_fast.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/camera.cc tests/test_camera_fast.cc -pthread -o $@
build/bench-camera: src/camera.cc src/camera_color_math.h src/camera_timestamp.h tests/bench_camera.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/camera.cc tests/bench_camera.cc -pthread -o $@
test-fast: build/test-preprocess-plan build/test-camera-fast
	./build/test-preprocess-plan --benchmark
	./build/test-camera-fast
test-input-memory: | build
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RKNN -I$(RKNN_TEST_INCLUDE) src/detector.cc src/detector_math.cc tests/test_input_memory.cc -rdynamic -ldl -pthread -o build/test-input-memory
	./build/test-input-memory
	$(CXX) $(CXXFLAGS) -Isrc -DHAVE_RKNN -DNO_IO_API -I$(RKNN_TEST_INCLUDE) src/detector.cc src/detector_math.cc tests/test_input_memory.cc -rdynamic -ldl -pthread -o build/test-input-memory-no-api
	./build/test-input-memory-no-api

test-ui-render:
	sh tests/run_ui_render_test.sh --benchmark

build/test-ui-schedule: tests/test_ui_schedule.c src/ui_schedule.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_ui_schedule.c -o "$@"
test-ui-schedule: build/test-ui-schedule
	./build/test-ui-schedule
