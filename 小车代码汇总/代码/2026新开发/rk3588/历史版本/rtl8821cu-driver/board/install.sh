#!/bin/sh
set -eu
PKG=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
. "$PKG/common.sh"
mode=${1:-install}
case "$mode" in install|--check) ;; *) fail 'Usage: sh install.sh [--check]' ;; esac
verify_package
check_kernel
check_loaded
check_conflicts
check_destination "$PKG/payload/8821cu.ko" "$MODFILE"
check_destination "$PKG/payload/S35rtl8821cu" "$INITFILE"
check_destination "$PKG/payload/acg138-rtl8821cu.conf" "$CONFFILE"
command -v insmod >/dev/null 2>&1 || fail 'insmod is required.'
echo 'PASS: ARM64 / Linux 6.1.75 / kernel config / package checksums.'
[ "$mode" != --check ] || { echo 'Check only: no files changed, no module loaded.'; exit 0; }
[ "$(id -u)" = 0 ] || fail 'Run as root.'
[ -d /etc/init.d ] || fail 'Expected Buildroot /etc/init.d is missing.'
# Never overwrite existing differing files. Roll back only files created here.
new_module=0 new_init=0 new_conf=0 committed=0 tmpfile=
cleanup() {
    code=$?
    trap - EXIT HUP INT TERM
    [ -z "$tmpfile" ] || rm -f "$tmpfile"
    if [ "$committed" = 0 ]; then
        [ "$new_init" = 0 ] || rm -f "$INITFILE"
        [ "$new_conf" = 0 ] || rm -f "$CONFFILE"
        [ "$new_module" = 0 ] || rm -f "$MODFILE"
        refresh_index
        echo 'Install did not complete; new package files were removed.' >&2
    fi
    exit "$code"
}
trap cleanup EXIT
trap 'exit 130' HUP INT TERM
copy_new() {
    source_file=$1 target_file=$2 perms=$3
    tmpfile=$(mktemp "$target_file.tmp.XXXXXX")
    cp "$source_file" "$tmpfile"
    chmod "$perms" "$tmpfile"
    mv "$tmpfile" "$target_file"
    tmpfile=
}
mkdir -p "$MODDIR" /etc/modprobe.d
if [ ! -f "$MODFILE" ]; then new_module=1; copy_new "$PKG/payload/8821cu.ko" "$MODFILE" 644; fi
if [ ! -f "$CONFFILE" ]; then new_conf=1; copy_new "$PKG/payload/acg138-rtl8821cu.conf" "$CONFFILE" 644; fi
if [ ! -f "$INITFILE" ]; then new_init=1; copy_new "$PKG/payload/S35rtl8821cu" "$INITFILE" 755; fi
# cfg80211 and USB are built into this exact kernel, so depmod is optional.
sh "$INITFILE" start || { dmesg | tail -n 30 >&2; fail 'Module load failed; see the kernel messages above.'; }
committed=1
refresh_index
echo 'INSTALLED: 8821cu.ko; boot auto-load enabled. No reboot required.'
sh "$PKG/status.sh"
