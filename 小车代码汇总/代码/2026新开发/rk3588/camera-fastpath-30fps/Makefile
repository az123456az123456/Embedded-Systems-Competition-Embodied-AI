CC ?= cc
CFLAGS ?= -O2 -std=gnu11 -Wall -Wextra -Werror
.PHONY: test test-fps arm64 arm64-30fps
build:
	mkdir -p build
build/test-receiver: tests/test_receiver.c src/raw_receiver.h src/raw_rows.h | build
	$(CC) $(CFLAGS) -Isrc tests/test_receiver.c -pthread -o $@
test: build/test-receiver
	./build/test-receiver
build/test-output-fps: tests/test_output_fps.c src/v4l2_output.h src/camera_rate.h src/pixels.h | build
	$(CC) $(CFLAGS) -Wno-unused-function -Isrc $< -o $@
build/test-output-fps-20: tests/test_output_fps.c src/v4l2_output.h src/camera_rate.h src/pixels.h | build
	$(CC) $(CFLAGS) -Wno-unused-function -DCAMERA_DEFAULT_FPS=20 -Isrc $< -o $@
build/test-output-fps-30: tests/test_output_fps.c src/v4l2_output.h src/camera_rate.h src/pixels.h | build
	$(CC) $(CFLAGS) -Wno-unused-function -DCAMERA_DEFAULT_FPS=30 -Isrc $< -o $@
test-fps: build/test-output-fps build/test-output-fps-20 build/test-output-fps-30
	./build/test-output-fps
	./build/test-output-fps-20
	./build/test-output-fps-30
	/usr/bin/python3 tests/test_launcher_fps.py
arm64:
	./scripts/build-arm64.sh
arm64-30fps:
	CAMERA_DEFAULT_FPS=30 ./scripts/build-arm64.sh
