#!/bin/sh
# Read-only: no ioctl writes, permission changes, baud changes or motor commands.
set -u
SERIAL=${SERIAL_DEVICE:-/dev/ttyUSB0}
DATA=${DATA_DIR:-/userdata/acg138-ros2}
printf '%s\n' 'ACG138 ROS 2 preflight v37 (read-only; missing tools are reported)'
uname -a
printf '\nArchitecture / C library / Python\n'
getconf GNU_LIBC_VERSION 2>/dev/null || true
python3 --version 2>&1 || true
printf '\nAvailable memory and storage (mapping + current YOLO share resources)\n'
sed -n '/^MemTotal:/p;/^MemAvailable:/p;/^SwapTotal:/p' /proc/meminfo
CHECK_DIR=$DATA
while [ ! -d "$CHECK_DIR" ] && [ "$CHECK_DIR" != / ]; do CHECK_DIR=$(dirname "$CHECK_DIR"); done
df -Pk "$CHECK_DIR"
printf '\nUSB inventory / selected serial device\n'
if command -v lsusb >/dev/null 2>&1; then lsusb; else echo 'lsusb unavailable'; fi
ls -l "$SERIAL" 2>&1 || true
if command -v udevadm >/dev/null 2>&1; then
    udevadm info -q property -n "$SERIAL" 2>/dev/null | sed -n '/^ID_VENDOR_ID=/p;/^ID_MODEL_ID=/p;/^ID_SERIAL_SHORT=/p;/^ID_PATH=/p;/^DEVNAME=/p'
fi
printf '\nExpected: RPLIDAR C1 460800 baud. Read termios without changing it.\n'
if command -v python3 >/dev/null 2>&1; then
    SERIAL_DEVICE="$SERIAL" python3 - <<'PY'
import os, termios
p = os.environ['SERIAL_DEVICE']
try:
    fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    try:
        a = termios.tcgetattr(fd)
        print('termios_input_speed_code=%s output_speed_code=%s expected_B460800=%s' %
              (a[4], a[5], getattr(termios, 'B460800', 'unavailable')))
    finally:
        os.close(fd)
except (OSError, termios.error) as e:
    print('termios_read_unavailable=%s' % e)
PY
else echo 'Python missing: SDK configures 460800 directly; BusyBox stty is not needed.'; fi
printf '\nKernel features (running config; not the development PC config)\n'
FEATURES='CONFIG_NAMESPACES CONFIG_UTS_NS CONFIG_IPC_NS CONFIG_PID_NS CONFIG_NET_NS CONFIG_USER_NS CONFIG_CGROUPS CONFIG_CGROUP_DEVICE CONFIG_CGROUP_PIDS CONFIG_MEMCG CONFIG_SECCOMP CONFIG_SECCOMP_FILTER CONFIG_OVERLAY_FS CONFIG_UNIX98_PTYS CONFIG_SYSVIPC CONFIG_TMPFS CONFIG_USB_SERIAL_CP210X CONFIG_VETH CONFIG_BRIDGE'
if [ -r /proc/config.gz ] && command -v zcat >/dev/null 2>&1; then
    zcat /proc/config.gz | awk -v features="$FEATURES" 'BEGIN { n=split(features,a," "); for(i=1;i<=n;i++) wanted[a[i]]=1 } { key=$1; sub(/=.*/,"",key); if(key in wanted) print; if($1=="#" && $2 in wanted) print }'
    zcat /proc/config.gz | sha256sum
else echo 'running kernel config unavailable; consult BSP configs/kernel.config and verify board matches'; fi
printf '\nRuntime availability (does not start a daemon)\n'
sed -n '/^CapInh:/p;/^CapPrm:/p;/^CapEff:/p;/^CapBnd:/p' /proc/self/status
for c in docker containerd runc chroot mount umount setsid udevadm setpriv; do
    command -v "$c" 2>/dev/null || printf '%s unavailable\n' "$c"
done
printf '\nCgroups / filesystems / network names\n'
cat /proc/cgroups
sed -n '/cgroup/p;/overlay/p;/devpts/p' /proc/filesystems
if command -v ip >/dev/null 2>&1; then ip -br link 2>/dev/null || ip link; fi
printf '\nStorage guidance: use a persistent ext4 directory, >= 4 GiB free for exported userspace; PC image builds need substantially more.\n'
printf '%s\n' 'This report intentionally excludes process arguments, Wi-Fi passwords and pairing files.'
