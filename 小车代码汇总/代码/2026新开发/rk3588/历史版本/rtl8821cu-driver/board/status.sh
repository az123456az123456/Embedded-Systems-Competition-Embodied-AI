#!/bin/sh
# Read-only: does not bring interfaces up or down, scan, or change routes.
set -u
echo "Kernel: $(uname -r) / $(uname -m)"
if [ -d /sys/module/8821cu ]; then
    echo 'Driver: LOADED (8821cu)'
    for p in rtw_power_mgnt rtw_ips_mode rtw_enusbss; do
        [ ! -r "/sys/module/8821cu/parameters/$p" ] || echo "$p=$(cat "/sys/module/8821cu/parameters/$p")"
    done
else
    echo 'Driver: NOT LOADED'
fi
found=0 bound=0
for dev in /sys/bus/usb/devices/*; do
    [ -f "$dev/idVendor" ] && [ -f "$dev/idProduct" ] || continue
    [ "$(cat "$dev/idVendor"):$(cat "$dev/idProduct")" = 0bda:c820 ] || continue
    found=1
    echo "USB: 0bda:c820 at ${dev##*/}"
    for intf in "$dev":*; do
        [ -d "$intf" ] || continue
        owner=$(readlink "$intf/driver/module" 2>/dev/null || true)
        [ "${owner##*/}" = 8821cu ] || continue
        echo "USB Wi-Fi: BOUND to 8821cu (${intf##*/})"
        for net in "$intf"/net/*; do
            [ -d "$net" ] || continue
            bound=1
            echo "Wireless interface: ${net##*/}"
        done
    done
done
[ "$found" != 0 ] || echo 'USB: 0bda:c820 not present; insert the adapter, then run status.sh again.'
if command -v iw >/dev/null 2>&1; then iw dev; fi
if command -v ip >/dev/null 2>&1; then ip link show; fi
if [ "$bound" = 1 ]; then
    echo 'READY: USB driver bound and a wireless interface created. Wi-Fi connection setup is separate.'
elif [ "$found" = 1 ]; then
    echo 'NOT READY: USB adapter found but no 8821cu network interface; inspect dmesg | tail -n 60.'
fi
