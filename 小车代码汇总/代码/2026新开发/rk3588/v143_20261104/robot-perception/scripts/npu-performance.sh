#!/bin/sh
# Select an existing kernel governor; never alter OPPs, voltage or thermal policy.
set -eu
umask 077

SYSFS_ROOT=${SYSFS_ROOT:-/sys}
PROCFS_ROOT=${PROCFS_ROOT:-/proc}
STATE=${NPU_STATE_DIR:-/run/perception-npu-performance}
LOCK_HELD=0
TEMP_BACKUP=

die() { printf 'NPU performance: %s\n' "$*" >&2; exit 1; }
cleanup() {
    if [ -n "$TEMP_BACKUP" ]; then
        rm -f "$TEMP_BACKUP/governor" "$TEMP_BACKUP/target"
        rmdir "$TEMP_BACKUP" 2>/dev/null || :
    fi
    if [ "$LOCK_HELD" -eq 1 ]; then rmdir "$STATE/.lock" 2>/dev/null || :; fi
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

valid_word() {
    case "$1" in ''|*[!a-zA-Z0-9_-]*) return 1;; *) return 0;; esac
}
has_governor() {
    [ -r "$NPU/available_governors" ] || return 1
    GOVERNORS=$(cat "$NPU/available_governors")
    FOUND=1
    set -f
    for ITEM in $GOVERNORS; do if [ "$ITEM" = "$1" ]; then FOUND=0; break; fi; done
    set +f
    return "$FOUND"
}
discover() {
    NPU=
    COUNT=0
    for DEVICE in "$SYSFS_ROOT"/class/devfreq/*; do
        [ -d "$DEVICE" ] || continue
        NAME=${DEVICE##*/}
        if [ -r "$DEVICE/name" ]; then NAME="$NAME $(cat "$DEVICE/name")"; fi
        case "$NAME" in *npu*|*NPU*)
            NPU=$DEVICE
            COUNT=$((COUNT + 1))
            ;;
        esac
    done
    [ "$COUNT" -eq 1 ] || die "expected one NPU devfreq device, found $COUNT; nothing changed"
    TARGET=$(readlink -f "$NPU") || die "cannot resolve NPU devfreq target"
    [ -n "$TARGET" ] || die "empty NPU devfreq target"
}
prepare_state() {
    case "$STATE" in /*) :;; *) die "NPU_STATE_DIR must be an absolute private directory";; esac
    case "$STATE" in /|/run|/run/|/tmp|/tmp/) die "refusing a shared NPU_STATE_DIR";; esac
    [ ! -L "$STATE" ] || die "NPU state directory must not be a symbolic link"
    if [ ! -e "$STATE" ]; then mkdir -p "$STATE" || die "cannot create state directory $STATE"; fi
    [ -d "$STATE" ] || die "NPU state path is not a directory"
    STATE=$(readlink -f "$STATE") || die "cannot resolve NPU state directory"
    case "$STATE" in /|/run|/tmp) die "refusing a shared NPU_STATE_DIR";; esac
    OWNER=$(stat -c '%u' "$STATE") || die "cannot inspect state directory owner"
    [ "$OWNER" = "$(id -u)" ] || die "NPU state directory belongs to another user"
    chmod 700 "$STATE" || die "cannot protect state directory"
    mkdir "$STATE/.lock" 2>/dev/null || die "another performance operation is active (state lock exists)"
    LOCK_HELD=1
}
read_backup() {
    [ ! -L "$STATE/backup" ] && [ -d "$STATE/backup" ] || die "invalid NPU governor backup directory"
    for FILE in governor target; do
        [ ! -L "$STATE/backup/$FILE" ] && [ -f "$STATE/backup/$FILE" ] || die "invalid NPU backup $FILE"
    done
    SAVED_TARGET=$(cat "$STATE/backup/target")
    SAVED_GOVERNOR=$(cat "$STATE/backup/governor")
    [ "$SAVED_TARGET" = "$TARGET" ] || die "saved governor belongs to a different NPU; nothing changed"
    valid_word "$SAVED_GOVERNOR" || die "invalid saved governor; nothing changed"
    has_governor "$SAVED_GOVERNOR" || die "saved governor is no longer offered by this NPU; nothing changed"
}
field() {
    printf '%s: ' "$1"
    if [ -r "$NPU/$2" ]; then cat "$NPU/$2"; else printf 'unavailable\n'; fi
}
status() {
    discover
    printf 'NPU devfreq: %s\n' "$NPU"
    field 'Current frequency (Hz)' cur_freq
    field 'Available frequencies (Hz)' available_frequencies
    field 'Minimum frequency (Hz)' min_freq
    field 'Maximum frequency (Hz)' max_freq
    field 'Governor' governor
    field 'Available governors' available_governors
    CURRENT_HZ=$(cat "$NPU/cur_freq" 2>/dev/null || :)
    MAXIMUM_HZ=$(cat "$NPU/max_freq" 2>/dev/null || :)
    case "$CURRENT_HZ:$MAXIMUM_HZ" in
        *[!0-9:]*|:*|*:) :;;
        *) if [ "$MAXIMUM_HZ" -gt 0 ] && [ "$CURRENT_HZ" -ge "$MAXIMUM_HZ" ]; then
               printf 'Clock note: already at the configured maximum (%s Hz); performance can maintain the allowed clock, not select a higher frequency.\n' "$MAXIMUM_HZ"
           fi;;
    esac
    printf 'Driver load: '
    if [ -r "$SYSFS_ROOT/kernel/debug/rknpu/load" ]; then
        cat "$SYSFS_ROOT/kernel/debug/rknpu/load"
    elif [ -r "$PROCFS_ROOT/rknpu/load" ]; then
        cat "$PROCFS_ROOT/rknpu/load"
    else
        printf 'unavailable (no readable rknpu/load entry)\n'
    fi
}
enable() {
    discover
    has_governor performance || die "performance is not an available NPU governor; nothing changed"
    CURRENT=$(cat "$NPU/governor") || die "cannot read current NPU governor"
    valid_word "$CURRENT" && has_governor "$CURRENT" || die "invalid current NPU governor; nothing changed"
    prepare_state
    if [ -e "$STATE/backup" ] || [ -L "$STATE/backup" ]; then
        read_backup
    else
        TEMP_BACKUP=$STATE/.backup.$$
        mkdir "$TEMP_BACKUP" || die "cannot prepare governor backup"
        printf '%s\n' "$TARGET" > "$TEMP_BACKUP/target"
        printf '%s\n' "$CURRENT" > "$TEMP_BACKUP/governor"
        mv "$TEMP_BACKUP" "$STATE/backup" || die "cannot save governor backup"
        TEMP_BACKUP=
    fi
    printf '%s\n' performance > "$NPU/governor" || die "cannot select performance; governor backup retained"
    [ "$(cat "$NPU/governor")" = performance ] || die "kernel did not accept performance; governor backup retained"
    printf 'NPU governor: performance (original governor saved; frequency limits and thermal policy retained)\n'
    status
}
restore() {
    if [ ! -e "$STATE" ] && [ ! -L "$STATE" ]; then
        printf 'NPU performance: no saved governor to restore\n'; return
    fi
    prepare_state
    if [ ! -e "$STATE/backup" ] && [ ! -L "$STATE/backup" ]; then
        printf 'NPU performance: no saved governor to restore\n'; return
    fi
    discover
    read_backup
    printf '%s\n' "$SAVED_GOVERNOR" > "$NPU/governor" || die "cannot restore governor; backup retained"
    [ "$(cat "$NPU/governor")" = "$SAVED_GOVERNOR" ] || die "kernel rejected restored governor; backup retained"
    rm -f "$STATE/backup/governor" "$STATE/backup/target"
    rmdir "$STATE/backup" || die "governor restored, but backup directory contains unexpected files"
    printf 'NPU governor restored: %s\n' "$SAVED_GOVERNOR"
    status
}

case "${1:-status}" in
    status) [ "$#" -le 1 ] || die "usage: $0 status|enable|restore"; status;;
    enable) [ "$#" -eq 1 ] || die "usage: $0 status|enable|restore"; enable;;
    restore) [ "$#" -eq 1 ] || die "usage: $0 status|enable|restore"; restore;;
    *) die "usage: $0 status|enable|restore";;
esac
