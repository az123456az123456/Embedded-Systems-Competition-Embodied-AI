#!/usr/bin/env bash
# Build only the external USB Wi-Fi module; the BSP sources stay untouched.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
[[ $# == 2 ]] || { echo "Usage: $0 ACG138_BSP_ROOT MATCHING_MODULE_SYMVERS" >&2; exit 2; }
bsp=$(realpath "$1")
symvers=$(realpath "$2")
kernel="$bsp/sdk/yy3588_linux"
cross="$bsp/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-"
commit=5b39398e2de146edeb76716420f3288f508bea61
archive="$here/downloads/rtl8821cu-$commit.tar.gz"
out="$here/build/kernel"
src="$here/build/source"
hash() { sha256sum "$1" | awk '{print $1}'; }
[[ $(hash "$archive") == 3ccec54462a8668142a9c2b2ddfffe4e0434feebe6caf6a7de6812dc8713c37f ]]
[[ $(hash "$bsp/configs/kernel.config") == 7114825305fb4b5f032a523d009ae4eaa321f1c816ff60f44eeaca3b9a0ec20f ]]
[[ $(hash "$symvers") == ac81968d24f0c3885e824263897c46ad0f1c990c80ce5186bae3a19df85d41fa ]]
[[ -x "${cross}gcc" && -f "$kernel/Makefile" ]]
mkdir -p "$out" "$src" "$here/reports"
if [[ ! -f "$src/Makefile" ]]; then
    tar -xzf "$archive" --strip-components=1 -C "$src"
fi
cp "$bsp/configs/kernel.config" "$out/.config"
# This changes only SCM suffix generation, as in the BSP's prepared.config.
"$kernel/scripts/config" --file "$out/.config" --disable LOCALVERSION_AUTO
make -C "$kernel" O="$out" ARCH=arm64 CROSS_COMPILE="$cross" LOCALVERSION= olddefconfig modules_prepare
[[ $(cat "$out/include/config/kernel.release") == 6.1.75 ]]
# No CONFIG_MODVERSIONS: the matching BSP export list suffices for modpost.
cp "$symvers" "$out/Module.symvers"
make -j"${JOBS:-4}" -C "$kernel" O="$out" ARCH=arm64 CROSS_COMPILE="$cross" LOCALVERSION= \
    M="$src" CONFIG_PLATFORM_AUTODETECT=n CONFIG_RTL8821CU=m KVER=6.1.75 \
    USER_EXTRA_CFLAGS='-DCONFIG_LITTLE_ENDIAN -DCONFIG_IOCTL_CFG80211 -DRTW_USE_CFG80211_STA_EVENT -Wno-error' modules
"${cross}strip" --strip-debug "$src/8821cu.ko"
modinfo "$src/8821cu.ko" > "$here/reports/modinfo.txt"
"${cross}readelf" -h "$src/8821cu.ko" > "$here/reports/elf-header.txt"
echo "Built: $src/8821cu.ko"
