#!/bin/sh
set -eu
# Ignore permanent video numbers: discover only the combined FPGA publisher.
for path in /sys/class/video4linux/video*; do
 [ -r "$path/name" ] || continue
 name=$(cat "$path/name")
 case "$name" in FPGA_Combined*)
  node=/dev/${path##*/}
  [ -c "$node" ] || continue
  format=$(v4l2-ctl -d "$node" --get-fmt-video 2>/dev/null || true)
  case "$format" in *NV12*) printf '%s\n' "$node"; exit 0;; esac
 ;; esac
done
exit 1
