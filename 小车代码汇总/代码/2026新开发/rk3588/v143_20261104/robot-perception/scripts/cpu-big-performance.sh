#!/bin/sh
# RK3588 big cores only. Select kernel-provided performance without changing
# frequency limits, OPPs, voltage, thermal policy, or the little-core policy0.
set -eu
umask 077
SYSFS_ROOT=${SYSFS_ROOT:-/sys}
STATE=${CPU_STATE_DIR:-/run/perception-cpu-big-performance}
CPU_ROOT=$SYSFS_ROOT/devices/system/cpu/cpufreq
POLICY4=$CPU_ROOT/policy4
POLICY6=$CPU_ROOT/policy6
LOCK_HELD=0
TEMP_BACKUP=
TRANSACTION_ACTIVE=0
ROLLBACK4=
ROLLBACK6=

die() { printf 'CPU big performance: %s\n' "$*" >&2; exit 1; }
valid_word() { case "$1" in ''|*[!a-zA-Z0-9_-]*) return 1;; *) return 0;; esac; }
has_governor() {
    [ -r "$1/scaling_available_governors" ] || return 1
    GOVERNORS=$(cat "$1/scaling_available_governors")
    FOUND=1
    set -f
    for ITEM in $GOVERNORS; do if [ "$ITEM" = "$2" ]; then FOUND=0; break; fi; done
    set +f
    return "$FOUND"
}
verify_policy() {
    [ -d "$1" ] && [ -r "$1/related_cpus" ] || die "missing RK3588 big-core policy $1"
    EXPECTED=$2
    CPU_SET=$(cat "$1/related_cpus")
    set -f
    set -- $CPU_SET
    set +f
    [ "$#" -eq 2 ] && [ "$1 $2" = "$EXPECTED" ] || die "unexpected related_cpus; only RK3588 policies 4 5 and 6 7 are supported"
}
discover() {
    verify_policy "$POLICY4" '4 5'
    verify_policy "$POLICY6" '6 7'
    TARGET4=$(readlink -f "$POLICY4") || die "cannot resolve policy4"
    TARGET6=$(readlink -f "$POLICY6") || die "cannot resolve policy6"
    [ -n "$TARGET4" ] && [ -n "$TARGET6" ] && [ "$TARGET4" != "$TARGET6" ] || die "invalid big-core policy targets"
}
write_governor() {
    PREVIOUS=$(cat "$1/scaling_governor") || return 1
    if [ "$PREVIOUS" = "$2" ]; then return 0; fi
    printf '%s\n' "$2" > "$1/scaling_governor" || return 1
    [ "$(cat "$1/scaling_governor")" = "$2" ]
}
rollback() {
    FAILED=0
    if ! write_governor "$POLICY4" "$ROLLBACK4"; then FAILED=1; fi
    if ! write_governor "$POLICY6" "$ROLLBACK6"; then FAILED=1; fi
    if [ "$FAILED" -eq 0 ]; then
        printf 'CPU big performance: incomplete operation rolled back both previous governors\n' >&2
    else
        printf 'CPU big performance: rollback could not restore every governor; backup retained, inspect status\n' >&2
    fi
}
cleanup() {
    if [ "$TRANSACTION_ACTIVE" -eq 1 ]; then rollback; fi
    if [ -n "$TEMP_BACKUP" ]; then
        rm -f "$TEMP_BACKUP/policy4.target" "$TEMP_BACKUP/policy4.governor" \
              "$TEMP_BACKUP/policy6.target" "$TEMP_BACKUP/policy6.governor"
        rmdir "$TEMP_BACKUP" 2>/dev/null || :
    fi
    if [ "$LOCK_HELD" -eq 1 ]; then rmdir "$STATE/.lock" 2>/dev/null || :; fi
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

prepare_state() {
    case "$STATE" in /*) :;; *) die "CPU_STATE_DIR must be an absolute private directory";; esac
    [ ! -L "$STATE" ] || die "CPU state directory must not be a symbolic link"
    if [ ! -e "$STATE" ]; then mkdir -p "$STATE" || die "cannot create state directory"; fi
    [ -d "$STATE" ] || die "CPU state path is not a directory"
    STATE=$(readlink -f "$STATE") || die "cannot resolve state directory"
    case "$STATE" in /|/run|/tmp) die "refusing a shared CPU_STATE_DIR";; esac
    [ "$(stat -c '%u' "$STATE")" = "$(id -u)" ] || die "CPU state directory belongs to another user"
    chmod 700 "$STATE" || die "cannot protect state directory"
    mkdir "$STATE/.lock" 2>/dev/null || die "another CPU operation is active (state lock exists)"
    LOCK_HELD=1
}
read_backup() {
    [ ! -L "$STATE/backup" ] && [ -d "$STATE/backup" ] || die "invalid CPU backup directory"
    for FILE in policy4.target policy4.governor policy6.target policy6.governor; do
        [ ! -L "$STATE/backup/$FILE" ] && [ -f "$STATE/backup/$FILE" ] || die "invalid CPU backup file $FILE"
    done
    [ "$(cat "$STATE/backup/policy4.target")" = "$TARGET4" ] &&
    [ "$(cat "$STATE/backup/policy6.target")" = "$TARGET6" ] || die "saved governors belong to different CPU policy targets; nothing changed"
    SAVED4=$(cat "$STATE/backup/policy4.governor")
    SAVED6=$(cat "$STATE/backup/policy6.governor")
    valid_word "$SAVED4" && valid_word "$SAVED6" || die "invalid saved CPU governor; nothing changed"
    has_governor "$POLICY4" "$SAVED4" && has_governor "$POLICY6" "$SAVED6" || die "saved CPU governor is no longer available; nothing changed"
}
current_governors() {
    CURRENT4=$(cat "$POLICY4/scaling_governor") || die "cannot read policy4 governor"
    CURRENT6=$(cat "$POLICY6/scaling_governor") || die "cannot read policy6 governor"
    valid_word "$CURRENT4" && valid_word "$CURRENT6" || die "invalid current CPU governor"
    has_governor "$POLICY4" "$CURRENT4" && has_governor "$POLICY6" "$CURRENT6" || die "current CPU governor is not available"
}
field() {
    printf '  %s: ' "$2"
    if [ -r "$1/$3" ]; then cat "$1/$3"; else printf 'unavailable\n'; fi
}
status() {
    discover
    for POLICY in "$POLICY4" "$POLICY6"; do
        printf 'Big-core policy: %s\n' "$POLICY"
        field "$POLICY" 'Related CPUs' related_cpus
        field "$POLICY" 'Current frequency (kHz; snapshot)' scaling_cur_freq
        field "$POLICY" 'Minimum frequency (kHz)' scaling_min_freq
        field "$POLICY" 'Maximum frequency (kHz)' scaling_max_freq
        field "$POLICY" 'Available frequencies (kHz)' scaling_available_frequencies
        field "$POLICY" 'Governor' scaling_governor
        field "$POLICY" 'Available governors' scaling_available_governors
    done
    printf 'Scope: policies 4 and 6 only; policy0, frequency limits and thermal policy retained.\n'
}
apply_transaction() {
    ROLLBACK4=$CURRENT4
    ROLLBACK6=$CURRENT6
    TRANSACTION_ACTIVE=1
    write_governor "$POLICY4" "$1" || die "policy4 governor change failed; restoring previous values"
    write_governor "$POLICY6" "$2" || die "policy6 governor change failed; restoring previous values"
    TRANSACTION_ACTIVE=0
}
enable() {
    discover
    has_governor "$POLICY4" performance && has_governor "$POLICY6" performance || die "both big-core policies must offer performance; nothing changed"
    current_governors
    prepare_state
    if [ -e "$STATE/backup" ] || [ -L "$STATE/backup" ]; then
        read_backup
    else
        TEMP_BACKUP=$STATE/.backup.$$
        mkdir "$TEMP_BACKUP" || die "cannot prepare CPU backup"
        printf '%s\n' "$TARGET4" > "$TEMP_BACKUP/policy4.target"
        printf '%s\n' "$TARGET6" > "$TEMP_BACKUP/policy6.target"
        printf '%s\n' "$CURRENT4" > "$TEMP_BACKUP/policy4.governor"
        printf '%s\n' "$CURRENT6" > "$TEMP_BACKUP/policy6.governor"
        mv "$TEMP_BACKUP" "$STATE/backup" || die "cannot save CPU governor backup"
        TEMP_BACKUP=
    fi
    apply_transaction performance performance
    printf 'CPU big-core governors: performance (system frequency limits and thermal policy retained)\n'
    status
}
restore() {
    if [ ! -e "$STATE" ] && [ ! -L "$STATE" ]; then
        printf 'CPU big performance: no saved governors to restore\n'; return
    fi
    prepare_state
    if [ ! -e "$STATE/backup" ] && [ ! -L "$STATE/backup" ]; then
        printf 'CPU big performance: no saved governors to restore\n'; return
    fi
    discover
    read_backup
    current_governors
    apply_transaction "$SAVED4" "$SAVED6"
    rm -f "$STATE/backup/policy4.target" "$STATE/backup/policy4.governor" \
          "$STATE/backup/policy6.target" "$STATE/backup/policy6.governor"
    rmdir "$STATE/backup" || die "governors restored, but backup contains unexpected files"
    printf 'CPU big-core governors restored: policy4=%s policy6=%s\n' "$SAVED4" "$SAVED6"
    status
}

case "${1:-status}" in
    status) [ "$#" -le 1 ] || die "usage: $0 status|enable|restore"; status;;
    enable) [ "$#" -eq 1 ] || die "usage: $0 status|enable|restore"; enable;;
    restore) [ "$#" -eq 1 ] || die "usage: $0 status|enable|restore"; restore;;
    *) die "usage: $0 status|enable|restore";;
esac
