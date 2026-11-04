#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ROOTFS=${ROOTFS_DIR:-/acg138-ros2/rootfs}
exec "$ROOTFS/lib/ld-linux-aarch64.so.1" \
    --library-path "$BASE/lib:$ROOTFS/usr/lib/aarch64-linux-gnu:$ROOTFS/lib/aarch64-linux-gnu" \
    "$BASE/bin/smol-worker" \
    "$BASE/models/smolvlm2_500m_vision_fp16_rk3588.rknn" \
    "$BASE/models/smolvlm2_500m_llm_w8a8_rk3588.rkllm" "${SMOL_MAX_TOKENS:-48}" "$@"
