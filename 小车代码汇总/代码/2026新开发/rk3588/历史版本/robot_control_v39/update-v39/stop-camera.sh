#!/bin/sh
# Stop only the known FPGA ISP session. Park and exit dashboard/navigation first.
set -eu
for f in /proc/[0-9]*/cmdline; do
 [ -r "$f" ] || continue
 if tr '\000' '\n' < "$f" | awk 'NR==1 {n=split($0,a,"/"); shell=(a[n]~/^(sh|ash|bash|dash)$/)} shell && $0=="/opt/fpga-camera-isp-1080/run_isp_camera.sh" {ok=1} END {exit !ok}'; then
  pid=${f#/proc/}; pid=${pid%/cmdline}
  kill -TERM "$pid" 2>/dev/null || :
 fi
done
for e in /proc/[0-9]*/exe; do
 [ -L "$e" ] || continue
 target=$(readlink "$e" 2>/dev/null || :)
 case "$target" in /opt/fpga-camera-isp-1080/isp-probe|/opt/fpga-camera-isp-1080/isp-probe\ \(deleted\))
  pid=${e#/proc/}; pid=${pid%/exe}
  kill -TERM "$pid" 2>/dev/null || : ;;
 esac
done
i=0
while [ "$i" -lt 40 ]; do
 busy=0
 for e in /proc/[0-9]*/exe; do
  [ -L "$e" ] || continue
  target=$(readlink "$e" 2>/dev/null || :)
  case "$target" in /opt/fpga-camera-isp-1080/isp-probe|/opt/fpga-camera-isp-1080/isp-probe\ \(deleted\)) busy=1 ;; esac
 done
 [ "$busy" = 1 ] || break
 sleep 0.1; i=$((i+1))
done
[ "$busy" = 0 ] || { echo 'ISP did not exit. Keep the update stopped and inspect its process.' >&2; exit 1; }
if [ -d /tmp/fpga-isp.lock ]; then
 owner=$(cat /tmp/fpga-isp.lock/pid 2>/dev/null || :)
 case "$owner" in ''|*[!0-9]*) echo 'Unrecognized ISP lock; inspect it before updating.' >&2; exit 1 ;; esac
 if ! kill -0 "$owner" 2>/dev/null; then rm -f /tmp/fpga-isp.lock/pid; rmdir /tmp/fpga-isp.lock; fi
fi
echo 'Known FPGA ISP session stopped. No dashboard/navigation process was killed.'
