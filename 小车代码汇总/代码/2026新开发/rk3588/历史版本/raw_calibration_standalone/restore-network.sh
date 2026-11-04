#!/bin/sh
set -eu
IFACE=${1:-eth0}
case "$IFACE" in ''|*[!a-zA-Z0-9_.:-]*) exit 1;; esac
SAVE=/tmp/raw-calibration-net-$IFACE
[ -f "$SAVE/mtu" ] && [ -f "$SAVE/rmem_max" ] || { echo 'No saved network settings.' >&2; exit 1; }
MTU=$(cat "$SAVE/mtu");RMEM=$(cat "$SAVE/rmem_max")
case "$MTU:$RMEM" in *[!0-9:]*) echo 'Invalid saved settings' >&2; exit 1;; esac
ip link set dev "$IFACE" mtu "$MTU"
echo "$RMEM" > /proc/sys/net/core/rmem_max
rm "$SAVE/mtu" "$SAVE/rmem_max"
rmdir "$SAVE"
echo "Restored MTU and socket limit for $IFACE"
