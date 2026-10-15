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

C_SOURCE = src/main.c src/dashboard.c src/imu_data.c vendor/xdg-shell-protocol.c
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

.PHONY: all arm64 test test-window test-onnx test-launcher clean
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

build/test-camera: src/camera.cc src/camera.h tests/test_camera.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/camera.cc tests/test_camera.cc -pthread -o "$@"

build/test-detector: src/detector.cc src/detector_math.cc src/detector_math.h src/detector.h tests/test_detector.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/detector.cc src/detector_math.cc tests/test_detector.cc $(APP_LDFLAGS) $(APP_LDLIBS) $(LDLIBS) -o "$@"

test: all build/test-imu build/test-camera build/test-detector
	./build/test-imu
	./build/test-camera
	./build/test-detector
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

clean:
	rm -rf build
