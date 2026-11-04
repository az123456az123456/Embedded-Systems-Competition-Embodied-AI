#!/bin/sh
# Read-only. In particular, do not wake the screen using synthetic input, modify
# Weston settings, kill any process, or launch a second dashboard.
set -u
echo 'RK3588 HDMI / Wayland display snapshot (read-only)'
printf '  kernel='; uname -r
printf '  architecture='; uname -m
echo 'Weston and dashboard processes'
for exe in /proc/[0-9]*/exe; do
    path=$(readlink "$exe" 2>/dev/null || :)
    case "$path" in *weston*|*perception-dashboard.aarch64*)
        pid=${exe#/proc/}; pid=${pid%/exe}
        printf '  pid=%s executable=%s\n' "$pid" "$path"
        printf '  command='; tr '\000' ' ' < "/proc/$pid/cmdline" 2>/dev/null || :; echo
        if [ -r "/proc/$pid/environ" ]; then
            tr '\000' '\n' < "/proc/$pid/environ" | sed -n '/^XDG_RUNTIME_DIR=/p;/^WAYLAND_DISPLAY=/p;/^PERCEPTION_OUTPUT=/p;/^PERCEPTION_WESTON_ACTIVATE=/p'
        fi ;;
    esac
done
echo 'Wayland sockets'
for directory in /run /run/robot-wayland /run/user/0 /tmp; do
    for socket in "$directory"/wayland-*; do
        [ -S "$socket" ] || continue
        ls -l "$socket"
    done
done
echo 'DRM connector status'
for connector in /sys/class/drm/card*-*; do
    [ -f "$connector/status" ] || continue
    printf '  %s: ' "${connector##*/}"; cat "$connector/status"
    if [ -f "$connector/enabled" ]; then printf '    enabled='; cat "$connector/enabled"; fi
    if [ -r "$connector/modes" ]; then sed -n '1,8s/^/    mode=/p' "$connector/modes"; fi
done
echo 'Current dashboard SHA256'
sha256sum /root/robot-perception/perception-dashboard.aarch64 2>/dev/null || :
echo 'Recent Weston log (last 60 lines)'
for log in /var/log/weston.log /tmp/weston.log /tmp/robot-weston.log; do
    if [ -f "$log" ]; then printf '  %s\n' "$log"; tail -n 60 "$log"; fi
done
echo 'DISPLAY / PRESENT diagnostic lines in available perception logs'
for log in /tmp/perception-dashboard.log /tmp/perception.log /tmp/robot-perception.log /tmp/perception-display.log; do
    if [ -f "$log" ]; then printf '  %s\n' "$log"; tail -n 200 "$log" | sed -n '/DISPLAY/p;/PRESENT/p;/STAGE ui_hz=/p'; fi
done
echo 'If launched interactively, also copy its DISPLAY startup lines and latest STAGE ui_hz line.'
