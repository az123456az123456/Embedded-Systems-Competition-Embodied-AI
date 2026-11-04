#!/bin/sh
set -eu
PKG=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
. "$PKG/common.sh"
verify_package
[ "$(id -u)" = 0 ] || fail 'Run as root.'
check_destination "$PKG/payload/8821cu.ko" "$MODFILE"
check_destination "$PKG/payload/S35rtl8821cu" "$INITFILE"
check_destination "$PKG/payload/acg138-rtl8821cu.conf" "$CONFFILE"
check_loaded
if [ -d /sys/module/8821cu ]; then
    echo 'Unloading RTL8821CU; its Wi-Fi connection will disconnect.'
    rmmod 8821cu || fail 'Module is busy; no installed files removed.'
fi
rm -f "$INITFILE" "$CONFFILE" "$MODFILE"
rmdir "$MODDIR" 2>/dev/null || true
refresh_index
echo 'REMOVED: RTL8821CU driver and its boot auto-load settings.'
