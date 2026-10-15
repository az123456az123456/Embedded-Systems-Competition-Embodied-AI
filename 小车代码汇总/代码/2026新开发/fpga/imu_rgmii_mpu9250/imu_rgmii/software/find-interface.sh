#!/bin/sh
# Resolve the FPGA GMAC by controller identity, never by probe-order name.
set -eu
found=
for path in /sys/class/net/*; do
    device=$(readlink -f "$path/device" 2>/dev/null || true)
    if [ "${device##*/}" = fe1c0000.ethernet ]; then
        [ -z "$found" ] || { echo 'Multiple GMAC1 interfaces' >&2; exit 1; }
        found=${path##*/}
    fi
done
[ -n "$found" ] || { echo 'FPGA GMAC1 interface not registered; check DTS/pinmux/dmesg' >&2; exit 1; }
printf '%s\n' "$found"
