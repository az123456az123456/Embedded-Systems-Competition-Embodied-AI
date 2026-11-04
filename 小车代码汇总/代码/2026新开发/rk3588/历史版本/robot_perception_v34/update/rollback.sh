#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
. "$HERE/v34-common.sh"
[ "$#" = 1 ] || die 'Usage: sh rollback.sh /root/robot-perception-v34-backup.XXXXXX'
setup; verify_package; assert_stopped
BACKUP=$1
[ -d "$BACKUP" ] && [ ! -L "$BACKUP" ] || die 'Backup must be a real directory.'
BACKUP=$(CDPATH= cd -- "$BACKUP" && pwd -P)
case "$BACKUP" in "$BASE"/robot-perception-v34-backup.*) ;; *) die 'Wrong backup location.' ;; esac
for f in format old.sha256 new.sha256 targets; do regular "$BACKUP/$f"; done
[ "$(cat "$BACKUP/format")" = robot-perception-v34 ] || die 'Wrong backup format.'
for r in $roles; do [ "$(get "$r" "$BACKUP/targets")" = "$(dest "$r")" ] || die 'Backup target path mismatch.'; done
verify_backup "$BACKUP" || die 'Backup checksum mismatch.'
lock; stage_payload
snapshot > "$STAGES/current.sha256"
if cmp -s "$STAGES/current.sha256" "$BACKUP/old.sha256"; then echo 'Original files already restored.'; exit 0; fi
cmp -s "$STAGES/current.sha256" "$BACKUP/new.sha256" || die 'Runtime/model changed after v34; refusing to overwrite.'
# A rollback can itself fail; retain a complete forward recovery first.
FORWARD=$(mktemp -d "$BACKUP/.forward.XXXXXX")
cp "$STAGES/current.sha256" "$FORWARD/old.sha256"
for r in $roles; do cp -p "$(dest "$r")" "$FORWARD/$r"; done
verify_backup "$FORWARD" || die 'Forward recovery verification failed.'
RECOVERY=$FORWARD; IN_TX=1
for r in $roles; do
 dst=$(dest "$r"); h=$(get "$r" "$BACKUP/old.sha256")
 if [ "$h" = absent ]; then rm -f -- "$dst"
 else
  SWAP=$(mktemp "$(dirname -- "$dst")/.v34-rollback.XXXXXX")
  cp -p "$BACKUP/$r" "$SWAP"; mv -f "$SWAP" "$dst"; SWAP=
 fi
done
snapshot > "$STAGES/restored.sha256"
cmp -s "$STAGES/restored.sha256" "$BACKUP/old.sha256" || die 'Rollback verification failed.'
assert_preserved
IN_TX=0
rm -rf -- "$FORWARD"
echo 'Original dashboard, navigation, model defaults and labels restored. Current config/pairing/maps retained; no program started.'
