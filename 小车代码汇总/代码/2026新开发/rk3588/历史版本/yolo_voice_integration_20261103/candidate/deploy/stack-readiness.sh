#!/bin/sh
# Source from the existing stack supervisor; never opens hardware or HTTP.
stack_readiness_init() {
    STACK_READY_FILE=${ROBOT_STARTUP_READY_FILE:-${ROBOT_BOOT_STATE:-/run/robot-stack-autostart}/app-ready}
    STACK_READY_WRITTEN=0
}

stack_readiness_cleanup() {
    if [ "$STACK_READY_WRITTEN" = 1 ] && [ -r "$STACK_READY_FILE" ]; then
        read -r ready_pid ready_ticks ready_boot < "$STACK_READY_FILE" || return 0
        if [ "$ready_pid" = "$$" ]; then rm -f "$STACK_READY_FILE"; fi
    fi
}

stack_try_ready() {
    [ "$STACK_READY_WRITTEN" = 0 ] || return 0
    # The launcher truncates this log before its own UI starts. Old logs do not
    # count. Require both the phone service and an actual rendered UI interval.
    grep -q '^PHONE unified service on 127\.0\.0\.1:8082' "$1" 2>/dev/null || return 0
    awk '/^STAGE ui_hz=/ {split($2,a,"="); if(a[2]+0>0)found=1} END {exit !found}' "$1" 2>/dev/null || return 0
    ready_dir=${STACK_READY_FILE%/*}
    (umask 077; mkdir -p "$ready_dir") || return 0
    ready_ticks=$(awk '{sub(/^.*\) /, ""); print $20}' "/proc/$$/stat") || return 0
    ready_boot=$(cat /proc/sys/kernel/random/boot_id) || return 0
    ready_tmp=$STACK_READY_FILE.tmp.$$
    if (umask 077; printf '%s %s %s\n' "$$" "$ready_ticks" "$ready_boot" > "$ready_tmp") &&
        mv -f "$ready_tmp" "$STACK_READY_FILE"; then
        # Keep a per-attempt entry latch even if the user closes the UI before
        # the boot worker's next poll. Cleanup removes only the live marker.
        if [ -n "${ROBOT_STARTUP_ENTERED_FILE:-}" ]; then
            entered_tmp=$ROBOT_STARTUP_ENTERED_FILE.tmp.$$
            (umask 077; printf '%s %s %s %s\n' "$$" "$ready_ticks" "$ready_boot" "${ROBOT_STARTUP_ATTEMPT:-manual}" > "$entered_tmp") &&
                mv -f "$entered_tmp" "$ROBOT_STARTUP_ENTERED_FILE" || rm -f "$entered_tmp"
        fi
        STACK_READY_WRITTEN=1
        echo 'ROBOT_STACK_READY: UI and phone service entered; no motor enable requested'
    else
        rm -f "$ready_tmp"
    fi
}
