#!/bin/sh
set -eu
[ "$(uname -m)" = aarch64 ] || { echo 'Run on RK3588/aarch64.' >&2; exit 1; }
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$BASE"
sha256sum -c payload.sha256
chmod 755 bin/rawcap.aarch64
./bin/rawcap.aarch64 --help >/dev/null
mkdir -p /root/raw-calibration
if [ -f /root/raw-calibration/rawcap ]; then cp -p /root/raw-calibration/rawcap "/root/raw-calibration/rawcap.before-$(date +%s)"; fi
cp bin/rawcap.aarch64 /root/raw-calibration/rawcap
cp prepare-network.sh restore-network.sh /root/raw-calibration/
chmod 755 /root/raw-calibration/rawcap /root/raw-calibration/prepare-network.sh /root/raw-calibration/restore-network.sh
echo 'Installed native RAW receiver to /root/raw-calibration. Load the standalone SRAM bitstream with both lenses facing a uniform flat field.'
