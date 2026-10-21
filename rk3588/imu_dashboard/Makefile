CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CPPFLAGS += -Isrc -Ivendor -Iassets
SOURCE = src/main.c src/dashboard.c src/imu_data.c vendor/xdg-shell-protocol.c
CROSS_COMPILE ?= aarch64-none-linux-gnu-
WAYLAND_INCLUDE ?= /usr/include
WAYLAND_LIB ?= /usr/lib
.PHONY: all test arm64
all: build/imu-dashboard
build:
	mkdir -p build
build/imu-dashboard: $(SOURCE) src/imu_data.h src/dashboard.h assets/font_atlas.h | build
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SOURCE) -lwayland-client -lm -o $@
arm64: | build
	$(CROSS_COMPILE)gcc $(CFLAGS) $(CPPFLAGS) -I$(WAYLAND_INCLUDE) $(SOURCE) -L$(WAYLAND_LIB) -Wl,--allow-shlib-undefined -lwayland-client -lm -o build/imu-dashboard.aarch64
	$(CROSS_COMPILE)strip build/imu-dashboard.aarch64
test: all
	$(CC) $(CFLAGS) -Isrc src/imu_data.c tests/test_imu.c -lm -o build/test-imu
	./build/test-imu
	./build/imu-dashboard --preview build/preview.ppm
	./build/imu-dashboard --preview build/waiting.ppm --waiting

.PHONY: test-window
test-window: all
	$(CC) $(CFLAGS) -Ivendor tests/test_wayland.c vendor/xdg-shell-protocol.c -lwayland-server -o build/test-wayland
	python3 tests/check_wayland.py
