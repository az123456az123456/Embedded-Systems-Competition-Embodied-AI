#!/bin/sh
# Read-only status by default. capture mode owns a video node temporarily.
set -eu
MODE=${1:-status}
case "$MODE" in status|capture) ;; *) echo "Usage: $0 [status|capture /dev/videoN]" >&2; exit 2 ;; esac
printf '\n%s\n' '=== Camera / inference processes ==='
ps -ef | grep -E '[i]sp-probe|[p]erception-dashboard|[r]knn_yolov8_demo|[g]st-launch' || true
printf '\n%s\n' '=== Internal FPGA Ethernet ==='
for path in /sys/class/net/*; do
 device=$(readlink -f "$path/device" 2>/dev/null || true)
 case "$device" in */fe1c0000.ethernet)
  iface=${path##*/};printf 'interface=%s device=%s\n' "$iface" "$device"
  for item in mtu speed duplex carrier statistics/rx_packets statistics/rx_bytes statistics/rx_errors statistics/rx_dropped statistics/rx_missed_errors; do
   if [ -r "$path/$item" ]; then printf '%s=' "$item"; cat "$path/$item" 2>/dev/null || true; fi
  done
  if command -v ethtool >/dev/null 2>&1; then ethtool -S "$iface" 2>/dev/null || true; fi
 ;; esac
done
printf '\n%s\n' '=== ISP measurements (new receiver prints every 30 output frames) ==='
if [ -r /tmp/fpga-isp.log ]; then
 grep -E 'FASTPATH|ISP_RATE|ISP_RESULT|timeout|invalid|receive buffer' /tmp/fpga-isp.log | tail -n 15 || true
fi
printf '\n%s\n' '=== UDP receive errors / socket memory ==='
if [ -r /proc/net/snmp ]; then grep '^Udp:' /proc/net/snmp; fi
if [ -r /proc/sys/net/core/rmem_max ]; then cat /proc/sys/net/core/rmem_max; fi
printf '\n%s\n' '=== Frequency / temperature (read only) ==='
for path in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor /sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq /sys/class/devfreq/*npu*/cur_freq /sys/kernel/debug/rknpu/load /sys/class/thermal/thermal_zone*/temp; do
 if [ -r "$path" ]; then printf '%s: ' "$path"; cat "$path" 2>/dev/null || true; fi
done
if [ "$MODE" = capture ]; then
 VIDEO=${2:-}
 [ -n "$VIDEO" ] && [ -c "$VIDEO" ] || { echo 'Pass the actual NV12 /dev/videoN node.' >&2; exit 2; }
 if ps -ef | grep -E '[p]erception-dashboard|[r]knn_yolov8_demo|[g]st-launch' >/dev/null; then
  echo 'Close video windows first; leave isp-probe running.' >&2;exit 2
 fi
 command -v timeout >/dev/null 2>&1 || { echo 'timeout is required for a bounded measurement.' >&2;exit 2; }
 printf '\n%s\n' '=== V4L2 delivery test: no YOLO, RGB conversion or window rendering ==='
 v4l2-ctl -d "$VIDEO" --get-fmt-video
 timeout 25 v4l2-ctl -d "$VIDEO" --stream-mmap=4 --stream-count=90 --stream-to=/dev/null
fi
