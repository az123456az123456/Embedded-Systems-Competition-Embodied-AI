#!/bin/sh
set -eu
[ "$(uname -m)" = aarch64 ] || { echo 'Run this installer on the RK3588 (aarch64).' >&2; exit 1; }
SRC=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DEST=/root/image-cal-v40
cd "$SRC"
sha256sum -c payload.sha256
chmod 755 bin/fpc1.aarch64
./bin/fpc1.aarch64 --self-test
mkdir -p "$DEST"
if [ -f "$DEST/fpc1" ]; then
    cp -p "$DEST/fpc1" "$DEST/fpc1.before-native-$(date +%s)"
fi
cp bin/fpc1.aarch64 "$DEST/fpc1"
chmod 755 "$DEST/fpc1"
echo 'Installed /root/image-cal-v40/fpc1 (native ARM64; no Python required).'
echo 'First read-only check: /root/image-cal-v40/fpc1 --interface eth0 info --eye 0'
