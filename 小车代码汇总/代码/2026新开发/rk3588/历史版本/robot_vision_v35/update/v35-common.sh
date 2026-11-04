#!/bin/sh
# Fixed-role, recoverable model + executable + launcher transaction.
set -eu
die() { printf '%s\n' "$*" >&2; exit 1; }
hash() { sha256sum "$1" | awk '{print $1}'; }
regular() { [ -f "$1" ] && [ ! -L "$1" ] || die "Missing regular file: $1"; }
safe_hash() { [ "${#1}" = 64 ] || return 1; case "$1" in *[!0-9a-f]*) return 1 ;; esac; }
get() { awk -v k="$1" '$1==k {sub(/^[^[:space:]]+[[:space:]]+/, "");print;n++} END {if(n!=1)exit 1}' "$2"; }
roles='model labels manifest dashboard navigation launcher pair'
setup() {
 [ "$(id -u)" = 0 ] && [ "$(uname -m)" = aarch64 ] || die 'Run as root on RK3588 aarch64.'
 PREFIX=${ROBOT_V35_DESTDIR:-}
 if [ -n "$PREFIX" ]; then
  case "$PREFIX" in /*) ;; *) die 'Absolute fixture root required.' ;; esac
  PREFIX=$(CDPATH= cd -- "$PREFIX" && pwd -P)
  [ "$PREFIX" != / ] || die 'Do not use / as fixture root.'
 fi
 BASE=$(CDPATH= cd -- "$PREFIX/root" && pwd -P)
 APP=$(CDPATH= cd -- "$BASE/robot-perception" && pwd -P)
 NAV=$(CDPATH= cd -- "$BASE/robot-navigation" && pwd -P)
 ENTRY=$(CDPATH= cd -- "$BASE/perception-dashboard" && pwd -P)
 [ "$ENTRY" = "$APP" ] && [ "$APP" != "$NAV" ] || die 'Unified dashboard entry required.'
 for p in "$APP" "$NAV"; do case "$p/" in "$BASE/"*) ;; *) die 'Runtime directory escapes /root.' ;; esac; done
 MODELDIR=$(CDPATH= cd -- "$APP/model" && pwd -P)
 case "$MODELDIR/" in "$APP/"*) ;; *) die 'Model directory escapes application.' ;; esac
 for f in "$APP/perception-dashboard.aarch64" "$NAV/robot-navigation.aarch64" "$NAV/dashboard.sha256" "$APP/run-perception.sh" "$APP/run-dashboard.sh" "$NAV/run-navigation.sh" "$NAV/config.json"; do regular "$f"; done
 for f in "$APP/lib/librknnrt.so" "$APP/lib/libwayland-client.so.0"; do [ -s "$f" ] || die "Missing runtime: $f"; done
 LOCK=$BASE/.robot-control-v21.lock; HAVE_LOCK=0; IN_TX=0; STAGES=; RECOVERY=; SWAP=
 ENTRY_HASH=$(hash "$APP/run-dashboard.sh"); ORCH_HASH=$(hash "$NAV/run-navigation.sh")
 CONFIG_HASH=$(hash "$NAV/config.json")
}
dest() {
 case "$1" in
 model) printf '%s\n' "$MODELDIR/acg138-best-e7f26253-int8-rk3588.rknn" ;;
 labels) printf '%s\n' "$MODELDIR/acg138-best-e7f26253-labels.txt" ;;
 manifest) printf '%s\n' "$MODELDIR/acg138-best-e7f26253-manifest.json" ;;
 dashboard) printf '%s\n' "$APP/perception-dashboard.aarch64" ;;
 navigation) printf '%s\n' "$NAV/robot-navigation.aarch64" ;;
 launcher) printf '%s\n' "$APP/run-perception.sh" ;;
 pair) printf '%s\n' "$NAV/dashboard.sha256" ;;
 *) die 'Unknown transaction role.' ;; esac
}
state_hash() { p=$(dest "$1"); if [ -e "$p" ] || [ -L "$p" ]; then regular "$p"; hash "$p"; else printf '%s\n' absent; fi; }
snapshot() { for r in $roles; do printf '%s %s\n' "$r" "$(state_hash "$r")"; done; }
assert_stopped() {
 for e in "$PREFIX"/proc/[0-9]*/exe; do
  [ -e "$e" ] || [ -L "$e" ] || continue
  x=$(readlink "$e" 2>/dev/null || :)
  case "$x" in */perception-dashboard*|*/robot-navigation*|*/encoder-calibrate*) die 'Exit dashboard, navigation and calibration first (Ctrl+C).' ;; esac
 done
 for f in "$PREFIX"/proc/[0-9]*/cmdline; do
  [ -r "$f" ] || continue
  if tr '\000' '\n' < "$f" | awk 'NR==1{n=split($0,x,"/");s=x[n]~/^(sh|bash|ash|dash|busybox)$/} s&&($0~/\/(run-dashboard|run-navigation)\.sh$/||$0=="run-dashboard.sh"||$0=="run-navigation.sh"){a=1} END{exit !a}'; then die 'Exit unified launcher shells first.'; fi
 done
}
assert_preserved() {
 [ "$(hash "$APP/run-dashboard.sh")" = "$ENTRY_HASH" ] && [ "$(hash "$NAV/run-navigation.sh")" = "$ORCH_HASH" ] && [ "$(hash "$NAV/config.json")" = "$CONFIG_HASH" ] || die 'Launchers/config changed during transaction.'
}
verify_package() {
 regular "$HERE/SHA256SUMS"
 (cd "$HERE" && sha256sum -c SHA256SUMS) >/dev/null || die 'Package checksums failed.'
 for r in model labels manifest dashboard navigation; do regular "$HERE/payload/$r"; done
 for r in dashboard navigation; do
  p=$HERE/payload/$r
  [ "$(dd if="$p" bs=1 count=6 2>/dev/null | od -An -tx1 | tr -d ' \n')" = 7f454c460201 ] && [ "$(dd if="$p" bs=1 skip=18 count=2 2>/dev/null | od -An -tx1 | tr -d ' \n')" = b700 ] || die 'Payload must be ELF64 AArch64.'
 done
}
known_baseline() {
 d=$(state_hash dashboard); n=$(state_hash navigation)
 awk -v d="$d" -v n="$n" '$1==d&&$2==n{ok=1} END{exit !ok}' "$HERE/baseline-pairs.sha256" || die 'Unknown or mixed dashboard/navigation pair. No files changed.'
 [ "$(cat "$NAV/dashboard.sha256")" = "$d" ] || die 'Installed dashboard pairing checksum mismatch.'
}
stage_payload() {
 STAGES=$(mktemp -d "$BASE/.robot-perception-v35-stage.XXXXXX")
 for r in model labels manifest dashboard navigation; do cp "$HERE/payload/$r" "$STAGES/$r"; done
 awk -f "$HERE/transform-launcher.awk" "$APP/run-perception.sh" > "$STAGES/launcher" || die 'Cannot preserve current camera launcher with new defaults.'
 sh -n "$STAGES/launcher" || die 'Invalid transformed camera launcher.'
 printf '%s\n' "$(hash "$STAGES/dashboard")" > "$STAGES/pair"
 chmod 755 "$STAGES/dashboard" "$STAGES/navigation" "$STAGES/launcher"
 chmod 644 "$STAGES/model" "$STAGES/labels" "$STAGES/manifest" "$STAGES/pair"
 for r in $roles; do printf '%s %s\n' "$r" "$(hash "$STAGES/$r")"; done > "$STAGES/new.sha256"
}
verify_backup() {
 for r in $roles; do
  old=$(get "$r" "$1/old.sha256")
  if [ "$old" = absent ]; then [ ! -e "$1/$r" ] || return 1
  else safe_hash "$old" && regular "$1/$r" && [ "$(hash "$1/$r")" = "$old" ] || return 1; fi
 done
}
backup() {
 BACKUP=$(mktemp -d "$BASE/robot-perception-v35-backup.XXXXXX")
 snapshot > "$BACKUP/old.sha256"
 for r in $roles; do [ "$(get "$r" "$BACKUP/old.sha256")" = absent ] || cp -p "$(dest "$r")" "$BACKUP/$r"; done
 cp "$STAGES/new.sha256" "$BACKUP/new.sha256"
 printf '%s\n' robot-perception-v35 > "$BACKUP/format"
 for r in $roles; do printf '%s %s\n' "$r" "$(dest "$r")"; done > "$BACKUP/targets"
 verify_backup "$BACKUP" || die 'Backup verification failed.'
}
restore() {
 verify_backup "$RECOVERY" || return 1
 for r in $roles; do
  dst=$(dest "$r"); h=$(get "$r" "$RECOVERY/old.sha256")
  if [ "$h" = absent ]; then rm -f -- "$dst" || return 1
  else
   SWAP=$(mktemp "$(dirname -- "$dst")/.v35-restore.XXXXXX") || return 1
   cp -p "$RECOVERY/$r" "$SWAP" && mv -f "$SWAP" "$dst" || return 1
   SWAP=
  fi
 done
 snapshot > "$STAGES/restored.sha256" || return 1
 cmp -s "$STAGES/restored.sha256" "$RECOVERY/old.sha256"
}
cleanup() {
 code=$?; trap - EXIT HUP INT TERM
 [ -z "$SWAP" ] || rm -f -- "$SWAP"
 SWAP=
 if [ "$IN_TX" = 1 ]; then
  set +e
  if restore; then printf 'Update failed; previous files restored. Backup: %s\n' "$RECOVERY" >&2
  else printf 'Recovery incomplete; keep programs stopped. Backup: %s\n' "$RECOVERY" >&2; fi
  [ "$code" != 0 ] || code=1
 fi
 [ -z "$SWAP" ] || rm -f -- "$SWAP"
 [ -z "$STAGES" ] || rm -rf -- "$STAGES"
 [ "$HAVE_LOCK" != 1 ] || rmdir "$LOCK" 2>/dev/null || :
 exit "$code"
}
lock() { mkdir "$LOCK" 2>/dev/null || die 'An updater is active or its interrupted lock needs inspection.'; HAVE_LOCK=1; trap cleanup EXIT; trap 'exit 130' HUP INT TERM; }
replace() {
 for r in $roles; do
  dst=$(dest "$r")
  SWAP=$(mktemp "$(dirname -- "$dst")/.v35-swap.XXXXXX")
  cp -p "$STAGES/$r" "$SWAP"; mv -f "$SWAP" "$dst"; SWAP=
  [ "$(state_hash "$r")" = "$(get "$r" "$STAGES/new.sha256")" ] || die 'Replacement checksum mismatch.'
 done
 assert_preserved
}
