#!/bin/sh
# Change only the verified model/label defaults; preserve the original launcher.
set -eu
APP=/root/robot-perception
TEST=/root/model-validation-last-20261102/board-validation
EXPECTED_OLD=a559b71229997b7040cbc82576594a4f4661aa3d13fe8c415126f298104c479c
EXPECTED_MODEL=ae4598bc0065d8f411c7307e76d80d1bc6394d83cc4ab0221777fbb68ee158e3
if pidof perception-dashboard.aarch64 perception-dashboard >/dev/null 2>&1; then
    echo 'Stop the dashboard before changing model defaults.' >&2; exit 1
fi
actual=$(sha256sum "$APP/run-perception.sh");actual=${actual%% *}
if [ "$actual" != "$EXPECTED_OLD" ]; then
    echo 'Launcher changed since readback; refusing to overwrite.' >&2; exit 1
fi
model=$(sha256sum "$TEST/model-int8.rknn");model=${model%% *}
[ "$model" = "$EXPECTED_MODEL" ]
[ "$(wc -l < "$TEST/labels.txt")" -eq 9 ]
[ ! -e "$APP/run-perception.sh.pre-last-e0e54560" ]
cp -p "$APP/run-perception.sh" "$APP/run-perception.sh.pre-last-e0e54560"
cp "$TEST/model-int8.rknn" "$APP/model/acg138-last-e0e54560-int8-rk3588.rknn.part"
cp "$TEST/labels.txt" "$APP/model/acg138-last-e0e54560-labels.txt.part"
mv "$APP/model/acg138-last-e0e54560-int8-rk3588.rknn.part" "$APP/model/acg138-last-e0e54560-int8-rk3588.rknn"
mv "$APP/model/acg138-last-e0e54560-labels.txt.part" "$APP/model/acg138-last-e0e54560-labels.txt"
sed 's/acg138-best-e7f26253-int8-rk3588.rknn/acg138-last-e0e54560-int8-rk3588.rknn/g; s/acg138-best-e7f26253-labels.txt/acg138-last-e0e54560-labels.txt/g' \
    "$APP/run-perception.sh.pre-last-e0e54560" > "$APP/run-perception.sh.last-new"
sh -n "$APP/run-perception.sh.last-new"
chmod 755 "$APP/run-perception.sh.last-new"
mv "$APP/run-perception.sh.last-new" "$APP/run-perception.sh"
sync
printf 'MODEL_DEFAULT_APPLIED no_program_started=yes no_motor_commands=yes\n'
sha256sum "$APP/run-perception.sh" "$APP/model/acg138-last-e0e54560-int8-rk3588.rknn" "$APP/model/acg138-last-e0e54560-labels.txt"
