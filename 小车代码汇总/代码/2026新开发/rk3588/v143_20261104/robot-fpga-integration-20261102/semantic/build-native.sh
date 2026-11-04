#!/bin/sh
# Run on the board; requires staged headers, libraries and source.
set -eu
ROOTFS=${ROOTFS_DIR:-/acg138-ros2/rootfs}
BUILD=/opt/smol-semantic-build
chroot "$ROOTFS" /usr/bin/g++ -O2 -std=c++17 -Wall -Wextra -Werror \
  -isystem "$BUILD/include" "$BUILD/src/smol_worker.cc" \
  -L"$BUILD/lib" -lrkllmrt -lrknnrt -ldl -lgomp -pthread -o "$BUILD/smol-worker"
