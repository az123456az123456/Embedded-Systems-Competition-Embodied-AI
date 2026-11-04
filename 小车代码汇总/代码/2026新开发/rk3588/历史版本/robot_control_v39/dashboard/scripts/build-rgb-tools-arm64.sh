#!/bin/sh
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
TASK_WORKSPACE=$APP_DIR
while [ "$TASK_WORKSPACE" != / ]; do
    if [ -d "$TASK_WORKSPACE/YOLO/rknn_model_zoo" ]; then break; fi
    TASK_WORKSPACE=$(dirname -- "$TASK_WORKSPACE")
done
if [ -z "${CROSS_COMPILE:-}" ]; then
    for compiler in "$TASK_WORKSPACE"/bsp_r4/*/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++; do
        if [ -x "$compiler" ]; then CROSS_COMPILE=${compiler%g++}; break; fi
    done
fi
[ -n "${CROSS_COMPILE:-}" ] && command -v "${CROSS_COMPILE}g++" >/dev/null 2>&1 || { echo 'BSP ARM64 compiler not found; set CROSS_COMPILE.' >&2; exit 1; }
mkdir -p "$APP_DIR/build"
for tool in capture_fpga_rgb rgb_codec_probe; do
    case "$tool" in capture_fpga_rgb) output=capture-fpga-rgb ;; *) output=rgb-codec-probe ;; esac
    "${CROSS_COMPILE}g++" -O2 -std=c++17 -Wall -Wextra -Werror -I"$APP_DIR/src" \
        "$APP_DIR/src/rgb_camera.cc" "$APP_DIR/tools/$tool.cc" -pthread \
        -static-libstdc++ -static-libgcc -o "$APP_DIR/build/$output.aarch64"
    "${CROSS_COMPILE}strip" "$APP_DIR/build/$output.aarch64"
done
echo 'Built standalone ARM64 RGB capture/probe tools. No RKNN or Wayland dependency; not executed on hardware.'
