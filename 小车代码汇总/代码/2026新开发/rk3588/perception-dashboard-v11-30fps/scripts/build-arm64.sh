#!/bin/sh
# Cross-build only. Runtime libraries must come from the board's matching BSP.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
WORKSPACE=$APP_DIR
while [ "$WORKSPACE" != / ]; do
    if [ -d "$WORKSPACE/YOLO/rknn_model_zoo" ]; then break; fi
    WORKSPACE=$(dirname -- "$WORKSPACE")
done

if [ -z "${CROSS_COMPILE:-}" ]; then
    for prefix in aarch64-buildroot-linux-gnu- aarch64-none-linux-gnu- aarch64-linux-gnu-; do
        if command -v "${prefix}gcc" >/dev/null 2>&1; then CROSS_COMPILE=$prefix; break; fi
    done
fi
if [ -z "${CROSS_COMPILE:-}" ] && [ "$WORKSPACE" != / ]; then
    for compiler in "$WORKSPACE"/bsp_r4/*/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc; do
        if [ -x "$compiler" ]; then CROSS_COMPILE=${compiler%gcc}; break; fi
    done
fi
if [ -z "${CROSS_COMPILE:-}" ] || ! command -v "${CROSS_COMPILE}gcc" >/dev/null 2>&1 || ! command -v "${CROSS_COMPILE}g++" >/dev/null 2>&1; then
    printf '%s\n' 'ARM64 compiler not found. Set CROSS_COMPILE=/path/to/SDK/host/bin/aarch64-buildroot-linux-gnu-' >&2
    exit 1
fi

if [ -z "${WAYLAND_INCLUDE:-}" ]; then
    WAYLAND_INCLUDE=/usr/include
    if [ "$WORKSPACE" != / ] && [ -f "$WORKSPACE/gst-dev/usr/include/wayland-client.h" ]; then
        WAYLAND_INCLUDE=$WORKSPACE/gst-dev/usr/include
    fi
fi
if [ -z "${WAYLAND_LIB:-}" ]; then
    WAYLAND_LIB=/usr/lib
    if [ "$WORKSPACE" != / ] && [ -f "$WORKSPACE/gst-dev/lib/libwayland-client.so" ]; then
        WAYLAND_LIB=$WORKSPACE/gst-dev/lib
    fi
fi
if [ -z "${RKNN_INCLUDE:-}" ]; then
    RKNN_INCLUDE=/usr/include
    if [ "$WORKSPACE" != / ] && [ -f "$WORKSPACE/YOLO/rknn_model_zoo/3rdparty/rknpu2/include/rknn_api.h" ]; then
        RKNN_INCLUDE=$WORKSPACE/YOLO/rknn_model_zoo/3rdparty/rknpu2/include
    fi
fi
if [ -z "${RKNN_LIB:-}" ]; then
    RKNN_LIB=/usr/lib
    if [ "$WORKSPACE" != / ] && [ -f "$WORKSPACE/YOLO/rknn_model_zoo/3rdparty/rknpu2/Linux/aarch64/librknnrt.so" ]; then
        RKNN_LIB=$WORKSPACE/YOLO/rknn_model_zoo/3rdparty/rknpu2/Linux/aarch64
    fi
fi
if [ ! -f "$WAYLAND_INCLUDE/wayland-client.h" ] || [ ! -f "$RKNN_INCLUDE/rknn_api.h" ] || [ ! -f "$RKNN_LIB/librknnrt.so" ]; then
    printf '%s\n' 'Missing target development files. Set WAYLAND_INCLUDE, WAYLAND_LIB, RKNN_INCLUDE and RKNN_LIB to matching SDK paths.' >&2
    exit 1
fi

make -C "$APP_DIR" all \
    CC="${CROSS_COMPILE}gcc" CXX="${CROSS_COMPILE}g++" RKNN=1 \
    TARGET=build/perception-dashboard.aarch64 OBJDIR=build/obj-arm64 \
    WAYLAND_INCLUDE="$WAYLAND_INCLUDE" WAYLAND_LIB="$WAYLAND_LIB" \
    RKNN_INCLUDE="$RKNN_INCLUDE" RKNN_LIB="$RKNN_LIB" \
    LDFLAGS="${LDFLAGS:-} -static-libstdc++ -static-libgcc -Wl,--allow-shlib-undefined" "$@"
"${CROSS_COMPILE}strip" "$APP_DIR/build/perception-dashboard.aarch64"
printf '%s\n' 'Built build/perception-dashboard.aarch64. Use the board runtime libraries; no host libraries are bundled.'
