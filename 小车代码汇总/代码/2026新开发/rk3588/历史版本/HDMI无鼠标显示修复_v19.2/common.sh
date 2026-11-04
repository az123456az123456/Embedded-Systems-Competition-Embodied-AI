#!/bin/sh
# Shared, POSIX / BusyBox-compatible installer functions. This file is not a launcher.
set -eu
BASELINE_SHA=c36444374e98274d9ef6d048d1671a33c9a3ab0f8602b578c828c307d05fd959
die() { printf '%s\n' "$*" >&2; exit 1; }
hash() { sha256sum "$1" | awk '{print $1}'; }
valid_hash() {
    [ "${#1}" = 64 ] || return 1
    case "$1" in *[!0-9a-f]*) return 1 ;; esac
}
setup_paths() {
    [ "$(id -u)" = 0 ] || die 'Run as root on the RK3588.'
    [ "$(uname -m)" = aarch64 ] || die 'This update is for an aarch64 RK3588.'
    # A nonempty prefix is exclusively a staging / fixture root, never a bypass
    # for an application under the real /root directory.
    if [ -n "${HDMI_FIX_DESTDIR:-}" ]; then
        case "$HDMI_FIX_DESTDIR" in /*) ;; *) die 'HDMI_FIX_DESTDIR must be absolute.' ;; esac
        PREFIX=$(CDPATH= cd -- "$HDMI_FIX_DESTDIR" && pwd -P) || die 'Invalid staging root.'
        [ "$PREFIX" != / ] || die 'Do not use / as a staging root.'
        PROC=$PREFIX/proc
    else
        PREFIX=
        PROC=/proc
    fi
    BASE=$PREFIX/root
    [ -d "$BASE/robot-perception" ] || die 'Install the existing robot-perception package first.'
    APPDIR=$(CDPATH= cd -- "$BASE/robot-perception" && pwd -P) || die 'Cannot resolve application directory.'
    if [ -n "$PREFIX" ]; then
        case "$APPDIR/" in "$PREFIX/"*) ;; *) die 'Staging application resolves outside its fixture root.' ;; esac
    fi
    TARGET=$APPDIR/perception-dashboard.aarch64
    [ -f "$TARGET" ] && [ ! -L "$TARGET" ] || die 'The dashboard binary must be a regular file, not a symlink.'
    for runtime in run-dashboard.sh run-perception.sh lib/librknnrt.so lib/libwayland-client.so.0; do
        [ -s "$APPDIR/$runtime" ] || die "Missing existing runtime file: $runtime"
    done
    LOCK=$BASE/.robot-display-hdmi-v19.2.lock
    STAGE=
    HAVE_LOCK=0
}
verify_package() {
    [ -f "$HERE/SHA256SUMS" ] || die 'Missing package SHA256SUMS.'
    (cd "$HERE" && sha256sum -c SHA256SUMS) || die 'Package checksum verification failed.'
    [ -f "$HERE/dashboard.sha256" ] || die 'Missing dashboard SHA256.'
    NEW_SHA=$(cat "$HERE/dashboard.sha256")
    valid_hash "$NEW_SHA" || die 'Invalid dashboard SHA256.'
    [ "$NEW_SHA" != "$BASELINE_SHA" ] || die 'The payload is still the old dashboard.'
    [ -f "$HERE/perception-dashboard.aarch64" ] && [ ! -L "$HERE/perception-dashboard.aarch64" ] || die 'Missing regular dashboard payload.'
    [ "$(hash "$HERE/perception-dashboard.aarch64")" = "$NEW_SHA" ] || die 'Dashboard payload does not match its SHA256.'
    elf_head=$(dd if="$HERE/perception-dashboard.aarch64" bs=1 count=6 2>/dev/null | od -An -tx1 | tr -d ' \n')
    elf_machine=$(dd if="$HERE/perception-dashboard.aarch64" bs=1 skip=18 count=2 2>/dev/null | od -An -tx1 | tr -d ' \n')
    [ "$elf_head" = 7f454c460201 ] && [ "$elf_machine" = b700 ] || die 'Payload is not a little-endian ELF64 AArch64 program.'
}
assert_stopped() {
    for exe in "$PROC"/[0-9]*/exe; do
        [ -e "$exe" ] || [ -L "$exe" ] || continue
        running=$(readlink "$exe" 2>/dev/null || :)
        case "$running" in
            "$TARGET"|"$TARGET (deleted)"|"$BASE/robot-perception/perception-dashboard.aarch64"|"$BASE/robot-perception/perception-dashboard.aarch64 (deleted)")
                die 'Exit the running dashboard first (Ctrl+C in its terminal), then retry.' ;;
        esac
        if [ "$exe" -ef "$TARGET" ]; then
            die 'Exit the running dashboard first (Ctrl+C in its terminal), then retry.'
        fi
    done
}
cleanup() {
    [ -z "${STAGE:-}" ] || rm -f -- "$STAGE"
    [ "${HAVE_LOCK:-0}" = 0 ] || rmdir "$LOCK" 2>/dev/null || :
}
take_lock() {
    mkdir "$LOCK" 2>/dev/null || die 'Another display update is active, or a previous update was interrupted. Inspect the lock before retrying.'
    HAVE_LOCK=1
    trap cleanup EXIT
    trap 'exit 130' HUP INT TERM
}
make_stage() {
    STAGE=$(mktemp "$APPDIR/.display-hdmi-v19.2.XXXXXX") || die 'Cannot create application staging file.'
}
metadata_value() { sed -n "s/^$1=//p" "$2/receipt.txt"; }
