#!/bin/sh
set -eu
# PC only. Produces a complete ARM64 filesystem, not a Buildroot OS image.
IMAGE=${ROS_IMAGE:-acg138-ros2:jazzy-v37}
DEST=${1:-acg138-jazzy-v37-rootfs.tar.gz}
command -v docker >/dev/null 2>&1 || { echo 'Docker required on the development PC.' >&2; exit 1; }
[ ! -e "$DEST" ] || { echo "Refusing overwrite: $DEST" >&2; exit 1; }
CID=$(docker create --platform linux/arm64 "$IMAGE")
TMP=${DEST}.partial
trap 'docker rm "$CID" >/dev/null 2>&1 || true; rm -f "$TMP"' EXIT HUP INT TERM
# Avoid pipeline success hiding a failed docker export.
TAR_TMP=${DEST}.uncompressed.partial
trap 'docker rm "$CID" >/dev/null 2>&1 || true; rm -f "$TMP" "$TAR_TMP"' EXIT HUP INT TERM
docker export --output "$TAR_TMP" "$CID"
gzip -c "$TAR_TMP" > "$TMP"
mv "$TMP" "$DEST"
(cd "$(dirname "$DEST")" && sha256sum "$(basename "$DEST")") > "$DEST.sha256"
printf 'Copy %s and %s.sha256 to RK3588. This userspace shares the existing Linux 6.1.75 kernel.\n' "$DEST" "$DEST"
