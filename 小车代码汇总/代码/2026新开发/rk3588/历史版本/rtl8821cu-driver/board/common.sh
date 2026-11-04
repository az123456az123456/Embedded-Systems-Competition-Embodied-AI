#!/bin/sh
# Shared BusyBox/POSIX sh helpers. Source from the package directory.
set -eu
RELEASE=6.1.75
CONFIG_HASH=7114825305fb4b5f032a523d009ae4eaa321f1c816ff60f44eeaca3b9a0ec20f
SRCVERSION=94E22DDFC21FC4A76E27889
MODDIR=/lib/modules/6.1.75/extra/acg138-rtl8821cu
MODFILE=$MODDIR/8821cu.ko
INITFILE=/etc/init.d/S35rtl8821cu
CONFFILE=/etc/modprobe.d/acg138-rtl8821cu.conf
fail() { echo "ERROR: $*" >&2; exit 1; }
hash_file() { sha256sum "$1" | awk '{print $1}'; }
verify_package() {
    command -v sha256sum >/dev/null 2>&1 || fail 'sha256sum is required.'
    (cd "$PKG" && sha256sum -c SHA256SUMS >/dev/null) || fail 'Package checksum failed; copy/extract it again.'
}
check_kernel() {
    [ "$(uname -m)" = aarch64 ] || fail 'This package requires aarch64.'
    [ "$(uname -r)" = "$RELEASE" ] || fail 'This package requires Linux 6.1.75.'
    [ -r /proc/config.gz ] || fail '/proc/config.gz unavailable; cannot verify the running kernel.'
    actual=$(zcat /proc/config.gz | sha256sum | awk '{print $1}')
    [ "$actual" = "$CONFIG_HASH" ] || fail "Kernel config mismatch: $actual. Do not force-load this module."
}
check_loaded() {
    if [ -d /sys/module/8821cu ]; then
        [ "$(cat /sys/module/8821cu/srcversion 2>/dev/null)" = "$SRCVERSION" ] || fail 'A different 8821cu module is already loaded.'
    fi
}
check_destination() {
    src=$1 dst=$2
    [ ! -L "$dst" ] || fail "Existing symlink: $dst; nothing overwritten."
    if [ -e "$dst" ]; then
        [ -f "$dst" ] && [ "$(hash_file "$src")" = "$(hash_file "$dst")" ] || fail "Existing different file: $dst; nothing overwritten."
    fi
}
check_conflicts() {
    for dev in /sys/bus/usb/devices/*; do
        [ -f "$dev/idVendor" ] && [ -f "$dev/idProduct" ] || continue
        [ "$(cat "$dev/idVendor"):$(cat "$dev/idProduct")" = 0bda:c820 ] || continue
        for intf in "$dev":*; do
            [ -L "$intf/driver" ] || continue
            # Only inspect the vendor-specific Wi-Fi interface; Bluetooth may coexist.
            [ "$(cat "$intf/bInterfaceClass" 2>/dev/null)" = ff ] || continue
            owner=$(readlink "$intf/driver/module" 2>/dev/null || true)
            [ "${owner##*/}" = 8821cu ] || fail "USB Wi-Fi interface already bound to another driver: $intf"
        done
    done
    if [ -d /lib/modules/6.1.75 ]; then
        others=$(find /lib/modules/6.1.75 -type f \( -name '8821cu.ko*' -o -name 'rtl8821cu.ko*' \) ! -path "$MODFILE")
        [ -z "$others" ] || fail "Other RTL8821CU modules found; inspect them first: $others"
    fi
}
refresh_index() {
    if command -v depmod >/dev/null 2>&1; then
        depmod -a "$RELEASE" || echo 'NOTE: depmod failed; the boot script uses insmod directly.' >&2
    fi
}
