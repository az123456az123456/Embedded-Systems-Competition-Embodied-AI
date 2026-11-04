#!/bin/sh
# One attempt at a time. Retry only before a verified application entry.
# After entry, deliberate exit or a later failure never restarts the robot.
set -eu
umask 077
STATE=${ROBOT_BOOT_STATE:-/run/robot-stack-autostart}
READY=${ROBOT_STARTUP_READY_FILE:-$STATE/app-ready}
ENTRY=${ROBOT_DH_ENTRY:-/root/run-yolo.sh}
mkdir -p "$STATE"
exec 7>"$STATE/retry.lock"
flock -n 7 || { echo 'A dh startup retrier is already running'; exit 0; }
CHILD=
STOPPING=0
ATTEMPT=0
DELAY=2
BOOT_ID=$(cat /proc/sys/kernel/random/boot_id)
LATCH=$STATE/attempt-entered
status() {
    printf 'phase=%s\nattempt=%s\nchild=%s\nlast_exit=%s\n' "$1" "$ATTEMPT" "${CHILD:-}" "${2:-}" > "$STATE/retry-status.tmp"
    mv -f "$STATE/retry-status.tmp" "$STATE/retry-status.txt"
}
alive() {
    [ -r "/proc/$1/stat" ] || return 1
    [ "$(awk '{sub(/^.*\) /, ""); print $1}' "/proc/$1/stat")" != Z ] && kill -0 "$1" 2>/dev/null
}
valid_ready() {
    [ -r "$READY" ] || return 1
    read -r READY_PID READY_TICKS READY_BOOT < "$READY" || return 1
    case "$READY_PID:$READY_TICKS" in *[!0-9:]*|:|*:) return 1;; esac
    [ "$READY_PID" -gt 1 ] && [ "$READY_BOOT" = "$(cat /proc/sys/kernel/random/boot_id)" ] || return 1
    alive "$READY_PID" || return 1
    [ "$(awk '{sub(/^.*\) /, ""); print $20}' "/proc/$READY_PID/stat")" = "$READY_TICKS" ]
}
entered_this_attempt() {
    [ -r "$LATCH" ] || return 1
    read -r entered_pid entered_ticks entered_boot entered_attempt < "$LATCH" || return 1
    [ "$entered_pid" = "$CHILD" ] && [ "$entered_boot" = "$BOOT_ID" ] && [ "$entered_attempt" = "$ATTEMPT_TAG" ]
}
stop() {
    STOPPING=1
    trap '' INT TERM HUP
    if [ -n "$CHILD" ]; then kill -TERM "$CHILD" 2>/dev/null || :; wait "$CHILD" 2>/dev/null || :; fi
    status STOPPED
    exit 0
}
trap stop INT TERM HUP
export ROBOT_STARTUP_READY_FILE=$READY
while [ "$STOPPING" = 0 ]; do
    if valid_ready; then
        status ALREADY_ENTERED
        echo 'Existing dh application has entered; no duplicate launched'
        exit 0
    fi
    ATTEMPT=$((ATTEMPT+1))
    ATTEMPT_TAG=$$:$ATTEMPT:$BOOT_ID
    export ROBOT_STARTUP_ENTERED_FILE=$LATCH ROBOT_STARTUP_ATTEMPT=$ATTEMPT_TAG
    rm -f "$LATCH"
    [ ! -f "$STATE/attempt.log" ] || mv -f "$STATE/attempt.log" "$STATE/attempt.previous.log"
    # Same command as the board's dh alias, including its camera/interface flags.
    # FD7 belongs only to this supervisor, never to its children.
    sh "$ENTRY" --phone-fps auto --motor-interface eth0 --imu-interface eth0 "$@" 7>&- > "$STATE/attempt.log" 2>&1 &
    CHILD=$!
    status STARTING
    while alive "$CHILD"; do
        if valid_ready; then
            if [ "$READY_PID" = "$CHILD" ]; then
                status ENTERED
                echo "dh entered the application on attempt $ATTEMPT; retries disabled"
                code=0; wait "$CHILD" || code=$?
                CHILD=
                status FINISHED "$code"
                exit "$code"
            fi
            # A separately launched application won the stack lock. Only stop
            # our own failing attempt, then leave that application untouched.
            kill -TERM "$CHILD" 2>/dev/null || :
            wait "$CHILD" 2>/dev/null || :
            CHILD=
            status ALREADY_ENTERED
            exit 0
        fi
        sleep 1
    done
    code=0; wait "$CHILD" || code=$?
    if entered_this_attempt; then
        CHILD=
        status FINISHED "$code"
        echo 'dh entered and exited before the next poll; retries disabled'
        exit "$code"
    fi
    CHILD=
    status RETRY_WAIT "$code"
    sleep "$DELAY"
    [ "$DELAY" -ge 10 ] || DELAY=$((DELAY+2))
done
