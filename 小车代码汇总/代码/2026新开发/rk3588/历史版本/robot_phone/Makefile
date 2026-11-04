CC ?= cc
CXX ?= c++
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Werror
CPPFLAGS += -Ibackend -Ibackend/perception
RKNN ?= 0
RKNN_INCLUDE ?= /usr/include
RKNN_LIB ?= /usr/lib
TARGET ?= build/robot-phone-server
OBJDIR ?= build/obj-$(if $(filter 1,$(RKNN)),rknn,host)
C_SOURCES = backend/perception/imu_data.c backend/jpeg_codec.c
CXX_SOURCES = backend/robot_server.cc backend/controller.cc backend/http_server.cc backend/json_flat.cc backend/motor_link.cc backend/perception/vision.cc backend/perception/camera.cc backend/perception/detector.cc backend/perception/detector_math.cc
OBJECTS = $(addprefix $(OBJDIR)/,$(C_SOURCES:.c=.o) $(CXX_SOURCES:.cc=.o))
APP_LIBS = -lm -pthread
ifeq ($(RKNN),1)
CPPFLAGS += -DHAVE_RKNN -I$(RKNN_INCLUDE)
APP_LIBS += -L$(RKNN_LIB) -Wl,-rpath,'$$ORIGIN/lib' -lrknnrt -ldl
endif
.PHONY: all test arm64 test-api clean
all: $(TARGET)
$(TARGET): $(OBJECTS)
	@mkdir -p "$(@D)"
	$(CXX) $(LDFLAGS) $(OBJECTS) $(APP_LIBS) -o "$@"
$(OBJDIR)/%.o: %.cc
	@mkdir -p "$(@D)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c "$<" -o "$@"
$(OBJDIR)/%.o: %.c
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c "$<" -o "$@"
-include $(OBJECTS:.o=.d)
build:
	mkdir -p build
build/test-controller: backend/controller.cc backend/controller.h tests/test_controller.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) backend/controller.cc tests/test_controller.cc -o "$@"
build/test-wire: backend/json_flat.cc backend/http_server.cc backend/motor_link.cc tests/test_wire.cc | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) backend/json_flat.cc backend/http_server.cc backend/motor_link.cc tests/test_wire.cc -pthread -o "$@"
build/test-imu: backend/perception/imu_data.c backend/perception/imu_data.h tests/test_imu.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) backend/perception/imu_data.c tests/test_imu.c -lm -o "$@"
test: all build/test-controller build/test-wire build/test-imu
	./build/test-controller
	./build/test-wire
	./build/test-imu
	python3 tests/test_protocol_vectors.py
test-api: all
	python3 tests/test_api.py
arm64:
	./scripts/build-arm64.sh
clean:
	rm -rf build
