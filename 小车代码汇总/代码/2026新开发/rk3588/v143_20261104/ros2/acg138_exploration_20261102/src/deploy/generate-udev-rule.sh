#!/bin/sh
# Prints a rule; does not install/reload permissions until the user copies it.
set -eu
DEV=${1:-/dev/ttyUSB0}
command -v udevadm >/dev/null 2>&1 || { echo 'udevadm unavailable; use SERIAL_DEVICE=/dev/ttyUSB0 until an exact device rule can be identified.' >&2; exit 1; }
PROPS=$(udevadm info -q property -n "$DEV")
VID=$(printf '%s\n' "$PROPS" | sed -n 's/^ID_VENDOR_ID=//p')
PID=$(printf '%s\n' "$PROPS" | sed -n 's/^ID_MODEL_ID=//p')
SER=$(printf '%s\n' "$PROPS" | sed -n 's/^ID_SERIAL_SHORT=//p')
case "$VID:$PID" in ????\:????) ;; *) echo 'USB VID/PID not found.' >&2; exit 1;; esac
case "$VID$PID" in *[!0-9a-fA-F]*) echo 'Unsafe VID/PID value.' >&2; exit 1;; esac
case "$SER" in ''|*[!a-zA-Z0-9_.-]*) echo 'No safe unique serial ID: inspect udevadm info and use a physical ID_PATH rule; do not match every CP210x.' >&2; exit 1;; esac
printf '%s\n' '# Exact adapter identity. Add a dialout group if absent before installing.'
printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", ATTRS{serial}=="%s", SYMLINK+="rplidar_c1", GROUP="dialout", MODE="0660"\n' "$VID" "$PID" "$SER"
