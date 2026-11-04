#!/bin/sh
# Read-only snapshot. Does not read the pairing code, send API requests, or move motors.
set -u
echo 'ACG138 v29-reactive-clearance phone-link snapshot (read-only)'
date
uname -r
for app in /root/robot-navigation/robot-navigation.aarch64; do
    [ ! -x "$app" ] || "$app" --version
done
echo 'USB Wi-Fi module power parameters (0 means disabled for these driver options)'
for module in 8821cu rtl8821cu; do
    for name in rtw_power_mgnt rtw_ips_mode rtw_enusbss; do
        f=/sys/module/$module/parameters/$name
        [ ! -r "$f" ] || printf '%s/%s=%s\n' "$module" "$name" "$(cat "$f")"
    done
done
echo 'Wireless interfaces'
for path in /sys/class/net/*; do
    [ -d "$path/wireless" ] || continue
    iface=${path##*/}
    printf '%s state=%s carrier=%s\n' "$iface" "$(cat "$path/operstate" 2>/dev/null || :)" "$(cat "$path/carrier" 2>/dev/null || :)"
    if command -v iw >/dev/null 2>&1; then
        iw dev "$iface" get power_save 2>/dev/null || :
        iw dev "$iface" link 2>/dev/null | sed -n '/signal:/p; /tx bitrate:/p; /rx bitrate:/p; /Not connected/p'
    fi
done
echo 'USB adapter inventory'
if command -v lsusb >/dev/null 2>&1; then lsusb 2>/dev/null | grep -iE '0bda|wireless|802\.11' || :; fi
echo 'Recent control and stream diagnostics'
for log in /tmp/robot-nav-dashboard.log /tmp/robot-navigation.log; do
    if [ -r "$log" ]; then
        printf 'log=%s\n' "$log"
        grep -E 'GATEWAY_CTRL|PHONE_CTRL|SOFTCORE_CTRL|SOFTCORE_NAV|PHONE_PERF|FOLLOW geometry|FOLLOW target=' "$log" | tail -n 16
    fi
done
echo 'Recent Wi-Fi/USB kernel events'
dmesg 2>/dev/null | grep -iE '8821|wlan|cfg80211|deauth|disassoc|usb.*(reset|disconnect|error)|firmware.*(fail|error)' | tail -n 30 || :
echo 'Follow geometry (camera intrinsics only; pairing file is never read)'
if [ -r /root/robot-navigation/follow_geometry.json ]; then
    head -c 16384 /root/robot-navigation/follow_geometry.json
    echo
else
    echo 'follow_geometry.json missing: angle following is disabled until camera calibration is installed.'
fi

echo 'Navigation status including passive FPGA NAV1 diagnostics (no API/control request)'
if [ -r /tmp/robot-navigation/status.json ]; then
    head -c 16384 /tmp/robot-navigation/status.json
    echo
fi
