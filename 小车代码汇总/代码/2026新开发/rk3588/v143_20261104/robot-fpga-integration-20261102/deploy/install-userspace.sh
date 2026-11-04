#!/bin/sh
set -eu
[ "$(id -u)" -eq 0 ] || { echo 'Run as root on RK3588.' >&2; exit 1; }
[ "$(uname -m)" = aarch64 ] || { echo 'ARM64 RK3588 required.' >&2; exit 1; }
ARCHIVE=${1:?Usage: install-userspace.sh ARM64-rootfs.tar.gz [rootfs-directory]}
ROOTFS=${2:-/userdata/acg138-ros2/rootfs}
[ ! -e "$ROOTFS" ] || { echo "Refusing existing path $ROOTFS" >&2; exit 1; }
[ -f "$ARCHIVE.sha256" ] || { echo 'Matching .sha256 file required.' >&2; exit 1; }
# Check the archive by its basename in its own directory.
(cd "$(dirname "$ARCHIVE")" && sha256sum -c "$(basename "$ARCHIVE").sha256")
PARENT=$(dirname "$ROOTFS")
mkdir -p "$PARENT"
FREE_KB=$(df -Pk "$PARENT" | awk 'END { print $4 }')
[ "$FREE_KB" -ge 4194304 ] || { echo 'At least 4 GiB free required for this extraction. Prefer persistent ext4 storage.' >&2; exit 1; }
mkdir "$ROOTFS"
if ! tar -xzf "$ARCHIVE" -C "$ROOTFS"; then
    echo "Extraction failed; incomplete directory retained for inspection: $ROOTFS" >&2
    exit 1
fi
[ -f "$ROOTFS/etc/acg138-ros2-rootfs" ] && [ -x "$ROOTFS/usr/local/bin/acg138-ros-entrypoint" ] || {
    echo 'Wrong userspace archive; marker or entrypoint missing.' >&2; exit 1;
}
printf 'Installed isolated userspace: %s\n' "$ROOTFS"
printf '%s\n' 'Existing Buildroot /lib, /usr, kernel, YOLO and network drivers were not replaced.'
