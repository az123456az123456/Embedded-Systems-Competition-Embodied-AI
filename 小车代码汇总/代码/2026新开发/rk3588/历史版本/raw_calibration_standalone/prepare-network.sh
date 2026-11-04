#!/bin/sh
set -eu
IFACE=${1:-eth0}
case "$IFACE" in ''|*[!a-zA-Z0-9_.:-]*) echo 'Invalid interface' >&2; exit 1;; esac
[ -d "/sys/class/net/$IFACE" ] || { echo "Interface missing: $IFACE" >&2; exit 1; }
SAVE=/tmp/raw-calibration-net-$IFACE
if [ ! -d "$SAVE" ]; then
    mkdir "$SAVE"
    cat "/sys/class/net/$IFACE/mtu" > "$SAVE/mtu"
    cat /proc/sys/net/core/rmem_max > "$SAVE/rmem_max"
fi
ip link set dev "$IFACE" mtu 4000
ip link set dev "$IFACE" up
# Temporary increase, not written into persistent sysctl configuration.
if [ -w /proc/sys/net/core/rmem_max ]; then echo 16777216 > /proc/sys/net/core/rmem_max; fi
echo "Prepared $IFACE for jumbo RAW rows; original MTU/socket limit saved in $SAVE"
