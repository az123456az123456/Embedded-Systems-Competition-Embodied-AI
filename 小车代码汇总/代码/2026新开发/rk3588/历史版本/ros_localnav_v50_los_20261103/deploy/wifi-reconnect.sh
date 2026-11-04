#!/bin/sh
# Application-owned WLAN reconnect loop. No FPGA interface or motion commands.
set -eu
umask 077
CONFIG=${ROBOT_WIFI_CONFIG:-/root/.config/acg138/wifi-networks.conf}
RUN=${ROBOT_WIFI_RUN_DIR:-/run/acg138-wifi}
LOG=${ROBOT_WIFI_LOG:-/tmp/robot-wifi.log}
IFACE=wlan0
mkdir -p "$RUN"
chmod 700 "$RUN"
exec 8>"$RUN/owner.lock"
flock -n 8 || exit 0
WPA_PID= DHCP_PID=
log() {
    if [ -f "$LOG" ] && [ "$(wc -c < "$LOG")" -ge 65536 ]; then mv -f "$LOG" "$LOG.previous"; fi
    printf '%s %s\n' "$(date '+%Y-%m-%dT%H:%M:%S')" "$*" >> "$LOG"
}
stop_dhcp() {
    if [ -n "$DHCP_PID" ]; then kill "$DHCP_PID" 2>/dev/null || :; wait "$DHCP_PID" 2>/dev/null || :; DHCP_PID=; fi
}
stop_wpa() {
    if [ -n "$WPA_PID" ]; then kill "$WPA_PID" 2>/dev/null || :; wait "$WPA_PID" 2>/dev/null || :; WPA_PID=; fi
}
cleanup() { trap - EXIT INT TERM HUP; stop_dhcp; stop_wpa; log stopped; }
trap cleanup EXIT
trap 'exit 0' INT TERM HUP
wpa() { timeout 2 wpa_cli -p "$RUN/ctrl" -i "$IFACE" "$@" 2>/dev/null; }
field() { printf '%s\n' "$STATUS" | sed -n "s/^$1=//p"; }
address() { ip -o -4 addr show dev "$IFACE" scope global 2>/dev/null | awk 'NR==1 {print $4}'; }
for cmd in wpa_supplicant wpa_cli udhcpc ip timeout; do
    command -v "$cmd" >/dev/null || { log "missing_tool=$cmd"; exit 1; }
done
log 'started priority=Ciallo,REDMI_K90 connected_network_is_kept'
while :; do
    if [ ! -f "$CONFIG" ] || [ -L "$CONFIG" ] || [ "$(stat -c %a "$CONFIG")" != 600 ] || [ "$(stat -c %u "$CONFIG")" != "$(id -u)" ]; then
        log 'private_configuration_missing_or_wrong_permissions'; sleep 30; continue
    fi
    if ! ip link show "$IFACE" >/dev/null 2>&1; then log 'waiting_for_wlan0'; sleep 5; continue; fi
    # Never kill or take over a system/third-party network manager.
    if pidof wpa_supplicant connmand NetworkManager udhcpc >/dev/null 2>&1; then
        log 'external_network_manager_present_waiting'; sleep 15; continue
    fi
    ip link set "$IFACE" up
    mkdir -p "$RUN/ctrl"
    # Copy only to this private runtime, replacing the control directory so tests
    # and independent installs cannot accidentally address a different daemon.
    { printf 'ctrl_interface=%s/ctrl\nupdate_config=0\n' "$RUN"; cat "$CONFIG"; } > "$RUN/wpa.conf"
    wpa_supplicant -i "$IFACE" -c "$RUN/wpa.conf" 8>&- > /dev/null 2>&1 &
    WPA_PID=$!
    ready=0; n=0
    while [ "$n" -lt 10 ] && kill -0 "$WPA_PID" 2>/dev/null; do
        if wpa ping | grep -qx PONG; then ready=1; break; fi
        sleep 1; n=$((n+1))
    done
    if [ "$ready" != 1 ]; then log 'supplicant_start_failed_retrying'; stop_wpa; sleep 5; continue; fi
    while kill -0 "$WPA_PID" 2>/dev/null; do
        for network in 0 1; do
            if [ "$network" = 0 ]; then name=Ciallo; else name='REDMI K90'; fi
            stop_dhcp
            # A lease from the previous hotspot must not qualify a new connection.
            ip -4 addr flush dev "$IFACE"
            ip -4 route flush dev "$IFACE"
            log "trying ssid=$name"
            if ! wpa select_network "$network" | grep -qx OK; then break; fi
            linked=0; n=0
            while [ "$n" -lt 15 ] && kill -0 "$WPA_PID" 2>/dev/null; do
                STATUS=$(wpa status || true)
                if [ "$(field wpa_state)" = COMPLETED ] && [ "$(field id)" = "$network" ] && [ "$(field ssid)" = "$name" ]; then linked=1; break; fi
                sleep 1; n=$((n+1))
            done
            if [ "$linked" != 1 ]; then log "association_timeout ssid=$name"; continue; fi
            # Keep the client in the foreground so its lifetime and lease renewal
            # are supervised; use the board's existing DHCP event script.
            udhcpc -f -n -i "$IFACE" -t 4 -T 2 -A 2 8>&- > /dev/null 2>&1 &
            DHCP_PID=$!
            leased=0; n=0
            while [ "$n" -lt 15 ] && kill -0 "$DHCP_PID" 2>/dev/null; do
                STATUS=$(wpa status || true); addr=$(address)
                if [ "$(field wpa_state)" != COMPLETED ] || [ "$(field id)" != "$network" ]; then break; fi
                if [ -n "$addr" ]; then leased=1; break; fi
                sleep 1; n=$((n+1))
            done
            if [ "$leased" != 1 ]; then log "dhcp_failed ssid=$name"; stop_dhcp; continue; fi
            log "connected ssid=$name ipv4=$addr"
            # No periodic SELECT_NETWORK/scan while a connection is healthy.
            while kill -0 "$WPA_PID" 2>/dev/null && kill -0 "$DHCP_PID" 2>/dev/null; do
                sleep 1
                STATUS=$(wpa status || true)
                if [ "$(field wpa_state)" != COMPLETED ] || [ "$(field id)" != "$network" ] || [ -z "$(address)" ]; then break; fi
            done
            log "disconnected ssid=$name retrying"
            stop_dhcp
            # After a successful connection drops, retry from the first choice.
            break
        done
        sleep 2
    done
    stop_dhcp; stop_wpa; sleep 5
done
