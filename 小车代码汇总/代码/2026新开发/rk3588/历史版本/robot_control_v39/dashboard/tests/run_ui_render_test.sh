#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build
"${CC:-cc}" -O2 -std=c11 -Wall -Wextra -Werror -Isrc -Iassets \
    src/dashboard.c src/display_transform.c src/imu_data.c src/sensor_data.c \
    tests/test_ui_render.c \
    -lm -o build/test-ui-render
./build/test-ui-render "$@"
