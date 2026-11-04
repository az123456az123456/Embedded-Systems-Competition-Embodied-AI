#!/bin/sh
# Build the dedicated FakeCam library; never install into the host or SDK.
set -eu
APP=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
ROOT=$APP
while [ "$ROOT" != / ] && [ ! -d "$ROOT/bsp_r4" ]; do ROOT=$(dirname -- "$ROOT"); done
SDK=${BSP_SDK:-$ROOT/bsp_r4/ACG138-Offline-BSP-20260923-r4/sdk/yy3588_extras}
SOURCE=${RKAIQ_SOURCE:-$SDK/external/camera_engine_rkaiq}
TC=${ARM64_TOOLCHAIN:-$SDK/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu}
CMAKE=${CMAKE:-cmake}
WORK=${FAKECAM_WORK:-$APP/build/fakecam}
DRM_HEADERS=${DRM_HEADERS:-$SOURCE/rkisp_demo/demo/include}
DRM_LIBRARY=${DRM_LIBRARY:-$SOURCE/rkisp_demo/demo/libs/arm64/libdrm.so}
mkdir -p "$WORK/source" "$WORK/build" "$APP/build/lib"
# Vendor CMake auto-detects ccache; avoid a cache outside this isolated build.
export CCACHE_DISABLE=1
if [ ! -f "$WORK/source/.patched-fps-v4" ]; then
    [ ! -f "$WORK/source/rkaiq/CMakeLists.txt" ] || { echo 'Use a fresh FAKECAM_WORK directory.' >&2; exit 2; }
    tar -C "$SOURCE" --exclude=CMakeFiles --exclude=CMakeCache.txt --exclude='*.o' --exclude=build -cf - . |
        tar -C "$WORK/source" -xf -
    python3 "$APP/scripts/patch-fakecam.py" "$WORK/source"
    touch "$WORK/source/.patched-fps-v4"
fi
cat > "$WORK/toolchain.cmake" <<EOF
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER "$TC/bin/aarch64-none-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "$TC/bin/aarch64-none-linux-gnu-g++")
set(CMAKE_SYSROOT "$TC/aarch64-none-linux-gnu/libc")
set(CMAKE_FIND_ROOT_PATH "$TC/aarch64-none-linux-gnu/libc")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
EOF
# The rkaiq subproject builds the library without unrelated demo/server targets.
"$CMAKE" -S "$WORK/source/rkaiq" -B "$WORK/build" \
    -DCMAKE_TOOLCHAIN_FILE="$(realpath "$WORK/toolchain.cmake")" \
    -DLIBDRM_LIBRARY="$DRM_LIBRARY" -DLIBDRM_INCLUDE_DIR="$DRM_HEADERS/libdrm" \
    -DLIBDRM_XF86DRM_INCLUDE_DIR="$DRM_HEADERS" \
    -DCMAKE_CXX_FLAGS="-I$DRM_HEADERS" \
    -DCMAKE_SKIP_RPATH=ON \
    -DCMAKE_INSTALL_PREFIX=/usr -DISP_HW_VERSION=-DISP_HW_V30 -DARCH=aarch64 \
    -DRKAIQ_TARGET_SOC=rk3588 -DCMAKE_BUILD_TYPE=RelWithDebInfo
"$CMAKE" --build "$WORK/build" --target rkaiq -j"${JOBS:-4}"
LIB=$(find "$WORK/build" -name librkaiq.so -type f | head -n 1)
[ -n "$LIB" ]
cp "$LIB" "$APP/build/lib/librkaiq.so"
"$TC/bin/aarch64-none-linux-gnu-strip" "$APP/build/lib/librkaiq.so"
printf '%s\n' 'Built build/lib/librkaiq.so; install only in the isolated FPGA-camera runtime.'
