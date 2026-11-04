#!/bin/sh
# POSIX / BusyBox: paired dashboard/navigation update; retain all configuration.
set -eu
die() { printf '%s\n' "$*" >&2; exit 1; }
hash() { sha256sum "$1" | awk '{print $1}'; }
valid_hash() { [ "${#1}" = 64 ] || return 1; case "$1" in *[!0-9a-f]*) return 1 ;; esac; }
known() { awk -v role="$1" -v sha="$2" '$1==sha && $2==role {n++} END {exit n!=1}' "$HERE/baselines.sha256"; }
known_pair() { awk -v dash="$1" -v nav="$2" '$1==dash && $2==nav {n++} END {exit n!=1}' "$HERE/baseline-pairs.sha256"; }
metadata() { sed -n "s/^$1=//p" "$2/receipt.txt"; }
regular() { [ -f "$1" ] && [ ! -L "$1" ] || die "Missing regular file (symlinks refused): $1"; }
setup_paths() {
    [ "$(id -u)" = 0 ] || die 'Run as root on the RK3588.'
    [ "$(uname -m)" = aarch64 ] || die 'This update requires an aarch64 RK3588.'
    PREFIX=
    if [ -n "${ROBOT_PHONE_V28_1_DESTDIR:-}" ]; then
        case "$ROBOT_PHONE_V28_1_DESTDIR" in /*) ;; *) die 'DESTDIR must be absolute.' ;; esac
        PREFIX=$(CDPATH= cd -- "$ROBOT_PHONE_V28_1_DESTDIR" && pwd -P) || die 'Invalid fixture root.'
        [ "$PREFIX" != / ] || die 'Do not use / as a staging root.'
    fi
    PROC=${PREFIX}/proc
    BASE=$(CDPATH= cd -- "$PREFIX/root" && pwd -P) || die 'Missing /root installation.'
    APPDIR=$(CDPATH= cd -- "$BASE/robot-perception" && pwd -P) || die 'Existing unified robot-perception runtime required.'
    NAVDIR=$(CDPATH= cd -- "$BASE/robot-navigation" && pwd -P) || die 'Existing unified robot-navigation runtime required.'
    for path in "$APPDIR" "$NAVDIR"; do case "$path/" in "$BASE/"*) ;; *) die 'Application directories must remain inside /root.' ;; esac; done
    if [ -n "$PREFIX" ]; then
        for path in "$BASE" "$APPDIR" "$NAVDIR"; do case "$path/" in "$PREFIX/"*) ;; *) die 'Application escapes the staging root.' ;; esac; done
    fi
    [ "$APPDIR" != "$NAVDIR" ] || die 'Dashboard/navigation directories must differ.'
    ENTRYDIR=$(CDPATH= cd -- "$BASE/perception-dashboard" && pwd -P) || die 'The public dashboard entry is missing.'
    [ "$ENTRYDIR" = "$APPDIR" ] || die 'perception-dashboard must resolve to the same robot-perception directory.'
    DASH=$APPDIR/perception-dashboard.aarch64; PAIR=$NAVDIR/dashboard.sha256
    NAV=$NAVDIR/robot-navigation.aarch64; CONFIG=$NAVDIR/config.json
    ENTRY=$APPDIR/run-dashboard.sh; ORCH=$NAVDIR/run-navigation.sh
    for path in "$DASH" "$PAIR" "$NAV" "$CONFIG" "$ENTRY" "$ORCH"; do regular "$path"; done
    [ -x "$DASH" ] && [ -x "$NAV" ] || die 'Existing executable permissions missing.'
    for path in "$CONFIG" "$APPDIR/run-perception.sh" "$APPDIR/lib/librknnrt.so" "$APPDIR/lib/libwayland-client.so.0"; do [ -s "$path" ] || die "Missing runtime: $path"; done
    LOCK=$BASE/.robot-control-v21.lock
    HAVE_LOCK=0; IN_TX=0; STAGE_DASH=; STAGE_NAV=; STAGE_PAIR=; PROFILE_STAGE=; RECOVERY=
}
assert_stopped() {
    for file in "$PROC"/[0-9]*/cmdline; do
        [ -r "$file" ] || continue
        if tr '\000' '\n' < "$file" | awk -v a="$ENTRY" -v b="$ORCH" -v c="$BASE/perception-dashboard/run-dashboard.sh" -v d="$BASE/robot-navigation/run-navigation.sh" '
            NR==1 {n=split($0,x,"/"); shell=(x[n]~/^(sh|bash|ash|dash|busybox)$/)}
            shell && ($0==a||$0==b||$0==c||$0==d||$0=="run-dashboard.sh"||$0=="run-navigation.sh") {found=1}
            END {exit !found}'
        then die 'Exit dashboard/navigation launcher shells before updating.'; fi
    done
    for exe in "$PROC"/[0-9]*/exe; do
        [ -e "$exe" ] || [ -L "$exe" ] || continue
        running=$(readlink "$exe" 2>/dev/null || :)
        case "$running" in */encoder-calibrate|*/encoder-calibrate.aarch64|*/encoder-calibrate\ \(deleted\)|*/encoder-calibrate.aarch64\ \(deleted\)) die 'Exit encoder calibration before updating.' ;; esac
        for target in "$DASH" "$NAV" "$BASE/robot-perception/perception-dashboard.aarch64" "$BASE/perception-dashboard/perception-dashboard.aarch64" "$BASE/robot-navigation/robot-navigation.aarch64"; do
            case "$running" in "$target"|"$target (deleted)") die 'Exit BOTH dashboard and navigation programs (Ctrl+C), then retry.' ;; esac
        done
        if [ "$exe" -ef "$DASH" ] || [ "$exe" -ef "$NAV" ]; then die 'Exit BOTH dashboard and navigation programs (Ctrl+C), then retry.'; fi
    done
}
verify_elf() {
    regular "$1"
    [ -x "$1" ] && [ "$(hash "$1")" = "$2" ] || die 'Payload checksum/permission mismatch.'
    elf_head=$(dd if="$1" bs=1 count=6 2>/dev/null | od -An -tx1 | tr -d ' \n')
    machine=$(dd if="$1" bs=1 skip=18 count=2 2>/dev/null | od -An -tx1 | tr -d ' \n')
    [ "$elf_head" = 7f454c460201 ] && [ "$machine" = b700 ] || die 'Payload is not little-endian ELF64 AArch64.'
}
verify_package() {
    for file in SHA256SUMS baselines.sha256 baseline-pairs.sha256 payload.sha256 package-mode.txt; do regular "$HERE/$file"; done
    (cd "$HERE" && sha256sum -c SHA256SUMS) || die 'Package checksum verification failed.'
    MODE=$(cat "$HERE/package-mode.txt")
    [ "$MODE" = dashboard-navigation ] || die 'This release requires the fixed three-file dashboard/navigation transaction.'
    UPDATE_NAV=1
    NEW_DASH=$(awk '$2=="payload/perception-dashboard.aarch64" {print $1}' "$HERE/payload.sha256")
    valid_hash "$NEW_DASH" || die 'Invalid/duplicate dashboard payload hash.'
    DASH_SOURCE=$HERE/payload/perception-dashboard.aarch64
    verify_elf "$DASH_SOURCE" "$NEW_DASH"
    known dashboard "$NEW_DASH" && die 'Dashboard payload is still a baseline version.'
    NEW_PAIR=$(printf '%s\n' "$NEW_DASH" | sha256sum | awk '{print $1}')
    NEW_NAV=$(awk '$2=="payload/robot-navigation.aarch64" {print $1}' "$HERE/payload.sha256")
    valid_hash "$NEW_NAV" || die 'Invalid/duplicate navigation payload hash.'
    NAV_SOURCE=$HERE/payload/robot-navigation.aarch64
    verify_elf "$NAV_SOURCE" "$NEW_NAV"
    known navigation "$NEW_NAV" && die 'Navigation payload is still a baseline version.'
    [ "$(awk 'END {print NR}' "$HERE/payload.sha256")" = 2 ] || die 'Unexpected payload members.'

}
read_current() { CUR_DASH=$(hash "$DASH"); CUR_PAIR=$(hash "$PAIR"); CUR_NAV=$(hash "$NAV"); CUR_CONFIG=$(hash "$CONFIG"); CUR_ENTRY=$(hash "$ENTRY"); CUR_ORCH=$(hash "$ORCH"); }
assert_launchers() { known dashboard_launcher "$CUR_ENTRY" && known navigation_launcher "$CUR_ORCH" || die 'Known unified launchers are required. Launcher contents differ from verified baselines; no files changed. Preserve your modifications and compare with the supplied launcher reference before retrying.'; }
assert_pair() {
    pair=$(printf '%s\n' "$CUR_DASH" | sha256sum | awk '{print $1}')
    [ "$CUR_PAIR" = "$pair" ] || die 'Dashboard/navigation pairing checksum mismatch.'
}
new_set() {
    [ "$CUR_DASH" = "$NEW_DASH" ] && [ "$CUR_PAIR" = "$NEW_PAIR" ] || return 1
    [ "$CUR_NAV" = "$NEW_NAV" ]
}
old_set() { known dashboard "$CUR_DASH" && known navigation "$CUR_NAV" && known_pair "$CUR_DASH" "$CUR_NAV" || die 'A complete known v30/v29.2/v29/v28.1/v28/v27.2/v27.1/v27/v26.4/v26.3/v26.2/v26.1/v26/v25/v24 pair, or known v21/v21.1 dashboard with v23/v23.2 navigation, is required; unknown/newer/mixed files refused.'; assert_pair; }
snapshot() { EXPECT_DASH=$CUR_DASH; EXPECT_PAIR=$CUR_PAIR; EXPECT_NAV=$CUR_NAV; EXPECT_CONFIG=$CUR_CONFIG; EXPECT_ENTRY=$CUR_ENTRY; EXPECT_ORCH=$CUR_ORCH; }
assert_current() {
    read_current
    [ "$CUR_DASH" = "$EXPECT_DASH" ] && [ "$CUR_PAIR" = "$EXPECT_PAIR" ] && [ "$CUR_NAV" = "$EXPECT_NAV" ] && [ "$CUR_CONFIG" = "$EXPECT_CONFIG" ] && [ "$CUR_ENTRY" = "$EXPECT_ENTRY" ] && [ "$CUR_ORCH" = "$EXPECT_ORCH" ] || die 'Runtime/config changed during transaction.'
}
unchanged_files() { [ "$(hash "$ENTRY")" = "$EXPECT_ENTRY" ] && [ "$(hash "$ORCH")" = "$EXPECT_ORCH" ]; }
remove_stages() {
    for stage in "$STAGE_DASH" "$STAGE_NAV" "$STAGE_PAIR" "$PROFILE_STAGE"; do [ -z "$stage" ] || rm -f -- "$stage"; done
    STAGE_DASH=; STAGE_NAV=; STAGE_PAIR=; PROFILE_STAGE=
}
make_stages() {
    STAGE_DASH=$(mktemp "$APPDIR/.navigation-sonar-v32-live-sonar-20261028.XXXXXX") || die 'Cannot stage dashboard.'
    STAGE_PAIR=$(mktemp "$NAVDIR/.navigation-sonar-v32-live-sonar-20261028.XXXXXX") || die 'Cannot stage pairing metadata.'
    STAGE_NAV=$(mktemp "$NAVDIR/.navigation-sonar-v32-live-sonar-20261028.XXXXXX") || die 'Cannot stage navigation.'
}
copy_current() {
    cp -p "$DASH" "$1/perception-dashboard.aarch64"
    cp -p "$PAIR" "$1/dashboard.sha256"
    cp -p "$NAV" "$1/robot-navigation.aarch64"
}
verify_copy() {
    [ "$(hash "$1/perception-dashboard.aarch64")" = "$2" ] && [ "$(hash "$1/dashboard.sha256")" = "$3" ] || return 1
    [ "$(hash "$1/robot-navigation.aarch64")" = "$4" ]
}
restore_transaction() {
    remove_stages
    STAGE_DASH=$(mktemp "$APPDIR/.navigation-sonar-v32-live-sonar-20261028.XXXXXX") || return 1
    STAGE_PAIR=$(mktemp "$NAVDIR/.navigation-sonar-v32-live-sonar-20261028.XXXXXX") || return 1
    cp -p "$RECOVERY/perception-dashboard.aarch64" "$STAGE_DASH" || return 1
    cp -p "$RECOVERY/dashboard.sha256" "$STAGE_PAIR" || return 1
    STAGE_NAV=$(mktemp "$NAVDIR/.navigation-sonar-v32-live-sonar-20261028.XXXXXX") || return 1
    cp -p "$RECOVERY/robot-navigation.aarch64" "$STAGE_NAV" || return 1
    verify_copy "$RECOVERY" "$RECOVER_DASH" "$RECOVER_PAIR" "$RECOVER_NAV" || return 1
    mv -f "$STAGE_DASH" "$DASH" || return 1; STAGE_DASH=
    mv -f "$STAGE_NAV" "$NAV" || return 1; STAGE_NAV=
    mv -f "$STAGE_PAIR" "$PAIR" || return 1; STAGE_PAIR=
    [ "$(hash "$DASH")" = "$RECOVER_DASH" ] && [ "$(hash "$PAIR")" = "$RECOVER_PAIR" ] && [ "$(hash "$NAV")" = "$RECOVER_NAV" ] && unchanged_files
}
cleanup() {
    code=$?; trap - EXIT HUP INT TERM
    if [ "$IN_TX" = 1 ]; then
        set +e
        if restore_transaction; then printf 'Transaction failed; previous complete dashboard/navigation pair restored. Config was never replaced. Backup: %s\n' "$BACKUP" >&2
        else printf 'Restoration incomplete; keep both programs stopped. Recovery: %s\n' "$RECOVERY" >&2; fi
        [ "$code" != 0 ] || code=1
    fi
    remove_stages
    if [ "$HAVE_LOCK" = 1 ]; then rmdir "$LOCK" 2>/dev/null || :; fi
    exit "$code"
}
take_lock() { mkdir "$LOCK" 2>/dev/null || die 'Another update is active or an interrupted lock needs inspection.'; HAVE_LOCK=1; trap cleanup EXIT; trap 'exit 130' HUP INT TERM; }
replace_stages() {
    mv -f "$STAGE_DASH" "$DASH"; STAGE_DASH=
    [ "$(hash "$DASH")" = "$DEST_DASH" ] || die 'Dashboard replacement verification failed.'
    mv -f "$STAGE_NAV" "$NAV"; STAGE_NAV=
    [ "$(hash "$NAV")" = "$DEST_NAV" ] || die 'Navigation replacement verification failed.'
    # Pairing metadata switches last; no process is allowed during this window.
    mv -f "$STAGE_PAIR" "$PAIR"; STAGE_PAIR=
    [ "$(hash "$PAIR")" = "$DEST_PAIR" ] && unchanged_files && [ "$(hash "$CONFIG")" = "$EXPECT_CONFIG" ] || die 'Pair verification failed or config/launchers changed externally.'
}
