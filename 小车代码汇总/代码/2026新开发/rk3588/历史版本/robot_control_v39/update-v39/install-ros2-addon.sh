#!/bin/sh
# Install v37 sources/helpers only. The separately built ROS userspace stays intact.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
[ "$(id -u)" = 0 ] || { echo 'Run as root.' >&2; exit 1; }
(cd "$HERE" && sha256sum -c SHA256SUMS) >/dev/null
DEST=/root/rplidar-ros2-v37
if [ -e "$DEST" ] || [ -L "$DEST" ]; then
 [ -d "$DEST" ] && [ ! -L "$DEST" ] || { echo 'Existing addon path is not a real directory.' >&2; exit 1; }
 cmp -s "$HERE/ros2-source.sha256" "$DEST/CHECKSUMS.sha256" || { echo 'Different ROS source version exists; preserve it and choose another path.' >&2; exit 1; }
 (cd "$DEST" && sha256sum -c CHECKSUMS.sha256) >/dev/null
 echo 'Exact v37 source addon already installed; ROS rootfs retained.'
 exit 0
fi
STAGE=$(mktemp -d /root/.rplidar-v37-source.XXXXXX)
trap 'rm -rf -- "$STAGE"' EXIT HUP INT TERM
tar -xzf "$HERE/rplidar-v37-source.tar.gz" -C "$STAGE"
(cd "$STAGE/rplidar-ros2-v37" && sha256sum -c CHECKSUMS.sha256) >/dev/null
mv "$STAGE/rplidar-ros2-v37" "$DEST"
echo 'v37 ROS sources/helpers installed. Full ROS userspace must be built and installed separately.'
echo 'Guide: /root/rplidar-ros2-v37/README_部署与联调.txt'
