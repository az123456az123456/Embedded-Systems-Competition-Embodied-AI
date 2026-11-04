#!/bin/sh
# Read-only collection. Never reads pairing/config/session files or sends control.
set -eu
NAVDIR=${WRONG_EXPORT_NAVDIR:-/root/robot-navigation}
OUTDIR=${WRONG_EXPORT_OUTDIR:-/root}
LOG=${ROBOT_WRONG_LOG_PATH:-$NAVDIR/wrong.log}
case "$LOG" in /*) ;; *) echo 'The log path must be absolute.' >&2; exit 2;; esac
[ -d "$OUTDIR" ] || { echo 'Output directory is missing.' >&2; exit 1; }
umask 077
WORK=$(mktemp -d /tmp/acg138-wrong-export.XXXXXX)
cleanup() { rm -rf -- "$WORK"; }
trap cleanup EXIT HUP INT TERM
mkdir "$WORK/wrong-diagnostics"
DEST=$WORK/wrong-diagnostics
found=0
for suffix in '' .1 .2; do
    file=$LOG$suffix
    if [ -f "$file" ] && [ ! -L "$file" ]; then
        cp "$file" "$DEST/wrong.log$suffix"
        found=1
    fi
done
[ "$found" = 1 ] || { echo "No regular wrong.log files found at $LOG" >&2; exit 1; }
if [ -f /tmp/robot-navigation/status.json ] && [ ! -L /tmp/robot-navigation/status.json ]; then
    cp /tmp/robot-navigation/status.json "$DEST/navigation-status.json"
fi
{
    printf '%s\n' 'Read-only fault-log export; no motion or network settings changed.'
    printf 'collected_at='; date -Iseconds 2>/dev/null || date
    printf 'kernel='; uname -a
    printf 'uptime='; cat /proc/uptime 2>/dev/null || :
    printf 'boot_id='; cat /proc/sys/kernel/random/boot_id 2>/dev/null || :
    printf '%s\n' 'For a stable complete final snapshot: park, allow the post-event interval, then exit the application before exporting.'
    printf '%s\n' 'Export while running is allowed, but rotation/appends can overlap a copy; a truncated final JSON line is incomplete.'
} > "$DEST/collection.txt"
if command -v dmesg >/dev/null 2>&1; then
    # A limited driver diagnostic tail; no wireless credentials or network commands.
    dmesg 2>/dev/null | tail -n 300 | awk 'tolower($0) ~ /8821|rtw|wlan|usb.*disconnect|usb.*reset|netdev.*watchdog|rknpu|out of memory|oom-kill/ {print}' > "$DEST/driver-events.txt" || :
fi
NAME=wrong-diagnostics-$(date +%Y%m%d-%H%M%S)-$$.tar.gz
tar -czf "$OUTDIR/$NAME" -C "$WORK" wrong-diagnostics
printf 'Saved: %s/%s\n' "$OUTDIR" "$NAME"
