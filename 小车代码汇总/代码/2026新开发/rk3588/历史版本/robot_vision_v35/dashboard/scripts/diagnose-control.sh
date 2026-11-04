#!/bin/sh
# Read-only. No interface configuration, arm, PWM, calibration or motor commands.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PORT=${PHONE_PORT:-8080}
TOKEN_FILE=${PAIR_TOKEN_FILE:-$APP_DIR/pair-token.txt}
while [ "$#" -gt 0 ]; do
    case "$1" in
        --port) [ "$#" -ge 2 ] || exit 2; PORT=$2; shift 2 ;;
        --token-file) [ "$#" -ge 2 ] || exit 2; TOKEN_FILE=$2; shift 2 ;;
        --help) printf '%s\n' 'Usage: sh scripts/diagnose-control.sh [--port 8080] [--token-file FILE]' 'Only reads network state and authenticated GET /api/state; never arms motors.'; exit 0 ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    esac
done
case "$PORT" in ''|*[!0-9]*) printf '%s\n' 'Invalid HTTP port' >&2; exit 2 ;; esac
if [ "$PORT" -lt 1024 ] || [ "$PORT" -gt 65535 ]; then exit 2; fi
printf '%s\n' 'ACG138 control/IMU diagnostic (read-only; missing files are reported)' 'Kernel and identity'
uname -a
id
printf '\n%s\n' 'Network devices (FPGA internal MAC should resolve to fe1c0000.ethernet, often eth1)'
for net in /sys/class/net/*; do
    [ -d "$net" ] || continue
    name=${net##*/}
    device=$(readlink -f "$net/device" 2>/dev/null || true)
    printf '  %s device=%s\n' "$name" "${device:-virtual/unknown}"
    for entry in ifindex address operstate carrier speed mtu flags; do
        if value=$(cat "$net/$entry" 2>/dev/null); then printf '    %s=%s\n' "$entry" "$value"; fi
    done
    for entry in rx_packets rx_errors rx_dropped tx_packets tx_errors tx_dropped; do
        if value=$(cat "$net/statistics/$entry" 2>/dev/null); then printf '    %s=%s\n' "$entry" "$value"; fi
    done
done
if command -v ip >/dev/null 2>&1; then ip -4 addr show 2>/dev/null || true; fi
printf '\n%s\n' 'Raw packet sockets (88b5=IMU, 88b6=motor, 88b7=passive NAV1; Iface is the device ifindex above)'
if [ -r /proc/net/packet ]; then cat /proc/net/packet; else printf '%s\n' '  /proc/net/packet unavailable'; fi
printf '\n%s\n' 'Running internal GMAC device tree (no delay values are changed)'
found=0
for node in /sys/firmware/devicetree/base/ethernet@fe1c0000 /proc/device-tree/ethernet@fe1c0000; do
    [ -d "$node" ] || continue
    found=1
    for property in status phy-mode; do
        if [ -r "$node/$property" ]; then printf '  %s=' "$property"; tr '\000' ' ' < "$node/$property"; printf '\n'; fi
    done
    for property in tx_delay rx_delay fixed-link/speed; do
        if [ -r "$node/$property" ]; then printf '  %s (big-endian hex)=' "$property"; od -An -tx1 "$node/$property"; fi
    done
    break
done
[ "$found" = 1 ] || printf '%s\n' '  direct device-tree node unavailable; inspect the device path above'
printf '\n%s\n' 'Unified application state (GET only; pairing code is not printed)'
if [ ! -r "$TOKEN_FILE" ]; then
    printf '  Pairing file missing or unreadable: %s\n' "$TOKEN_FILE"
    printf '%s\n' '  Start the unified program, or pass --token-file for its installed directory.'
    exit 0
fi
token=$(cat "$TOKEN_FILE")
url="http://127.0.0.1:$PORT/api/state"
if command -v curl >/dev/null 2>&1; then
    # Send authentication via stdin, not command-line arguments or printed logs.
    escaped=$(printf '%s' "$token" | sed 's/\\/\\\\/g;s/"/\\"/g')
    if ! printf 'header = "Authorization: Bearer %s"\n' "$escaped" | curl --silent --show-error --fail --max-time 3 --config - "$url"; then
        printf '\n%s\n' '  Cannot read application state. Check that the unified process is running on this port.'
    fi
elif command -v wget >/dev/null 2>&1; then
    if ! wget -q -T 3 -O - --header="Authorization: Bearer $token" "$url"; then
        printf '\n%s\n' '  Cannot read application state (or wget lacks --header support).'
    fi
else
    printf '%s\n' '  curl/wget unavailable; use the app terminal NETWORK / IMU_NET / PHONE_CTRL / PHONE_PERF logs.'
fi
unset token
printf '\n\n%s\n' 'Interpretation:'
printf '%s\n' \
 '  IMU rx_frames=0: no matching Ethernet frames reached the selected raw socket.' \
 '  IMU rx>0 but decoded=0: frame layout/version/length mismatch; preserve last_frame_bytes.' \
 '  IMU packet_received=true but flags bit0=0: sensor/I2C data invalid; inspect WHO_AM_I, errors and age.' \
 '  Motor tx_frames>0, ack_frames=0: check FPGA RX/ACK, selected interface and matching bitstream.' \
 '  Motor left/right=0: no active motion intent (or a safety interlock stopped it).' \
 '  Motor left/right!=0, ack_left/right=0: FPGA reports zero side outputs; inspect ACK age/sequence and armed.' \
 '  Both sent and ACK PWM nonzero: check driver/power/wiring; ACK is not encoder wheel-speed feedback.' \
 '  Heading hold requires fresh, initialized, calibrated IMU; manual drive does not require IMU.'
