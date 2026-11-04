#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
. "$HERE/v35-common.sh"
CHECK=0
case "${1:-}" in '') ;; --check-only) CHECK=1 ;; *) die 'Usage: sh install.sh [--check-only]' ;; esac
[ "$#" -le 1 ] || die 'Unexpected argument.'
setup; verify_package; assert_stopped
lock; stage_payload
snapshot > "$STAGES/before.sha256"
if cmp -s "$STAGES/new.sha256" "$STAGES/before.sha256"; then echo 'v35 model and runtime already installed; no changes.'; exit 0; fi
known_baseline
for r in model labels manifest; do
 h=$(state_hash "$r")
 [ "$h" = absent ] || [ "$h" = "$(get "$r" "$STAGES/new.sha256")" ] || die 'A different file occupies the versioned model path.'
done
assert_preserved; assert_stopped
if [ "$CHECK" = 1 ]; then echo 'PASS: package, model defaults, runtime pair and stopped-process checks. Installed files unchanged.'; exit 0; fi
backup
snapshot > "$STAGES/current.sha256"
cmp -s "$STAGES/current.sha256" "$STAGES/before.sha256" || die 'Files changed during preparation.'
assert_preserved; assert_stopped
RECOVERY=$BACKUP; IN_TX=1
replace
IN_TX=0
printf 'Installed new YOLOv8n-scale INT8 model and v35 FPGA-RGB pipeline.\nBackup: %s\nRollback: sh %s/rollback.sh %s\n' "$BACKUP" "$HERE" "$BACKUP"
echo 'Start: sh /root/perception-dashboard/run-dashboard.sh -0 (or -90). Requires matching robot_vision_v35_40fps.fs. APK/Wi-Fi/config/token retained; no motor started.'
