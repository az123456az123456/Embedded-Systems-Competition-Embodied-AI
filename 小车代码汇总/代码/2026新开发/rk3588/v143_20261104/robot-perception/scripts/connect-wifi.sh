#!/bin/sh
# Optional interactive setup; no hotspot password is included in the release.
set -eu
[ "$(id -u)" = 0 ] || { echo 'Run as root.' >&2; exit 2; }
APP=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
for tool in wpa_supplicant wpa_passphrase udhcpc ip ifconfig; do
 command -v "$tool" >/dev/null 2>&1 || { echo "Missing BSP Wi-Fi tool: $tool" >&2; exit 1; }
done
IFACE=${1:-}
if [ -z "$IFACE" ]; then
 for path in /sys/class/net/*; do
  if [ -d "$path/wireless" ] || [ -L "$path/phy80211" ]; then IFACE=${path##*/}; break; fi
 done
fi
[ -n "$IFACE" ] && { [ -d "/sys/class/net/$IFACE/wireless" ] || [ -L "/sys/class/net/$IFACE/phy80211" ]; } || { echo 'No wireless interface. Install the matching USB Wi-Fi driver first.' >&2; exit 1; }
if ps -ef | grep '[w]pa_supplicant' >/dev/null 2>&1; then
 echo 'A wpa_supplicant is already running. Use the existing BSP network setup to connect; this helper will not replace it.' >&2; exit 1
fi
printf 'Hotspot SSID: '; IFS= read -r ssid
[ -n "$ssid" ] || exit 2
secret=; hidden=0
cleanup() { [ "$hidden" = 0 ] || stty echo; unset secret; }
trap cleanup 0
trap 'exit 130' HUP INT TERM
printf 'Hotspot password: '
if [ -t 0 ]; then stty -echo; hidden=1; fi
IFS= read -r secret
if [ "$hidden" = 1 ]; then stty echo; hidden=0; fi
printf '\n'
umask 077
tmp=$(mktemp /etc/.robot-wifi.XXXXXX)
if ! printf '%s\n' "$secret" | wpa_passphrase "$ssid" | sed '/^[[:space:]]*#psk=/d' > "$tmp"; then rm -f "$tmp"; exit 1; fi
unset secret
grep '^[[:space:]]*psk=' "$tmp" >/dev/null || { rm -f "$tmp"; echo 'Invalid Wi-Fi password.' >&2; exit 1; }
mv "$tmp" /etc/wpa_supplicant-robot.conf
ip link set "$IFACE" up
wpa_supplicant -B -i "$IFACE" -c /etc/wpa_supplicant-robot.conf -P /run/robot-wpa.pid
udhcpc -i "$IFACE" -q -n -s "$APP/scripts/wifi-dhcp-hook.sh"
ip -4 addr show dev "$IFACE"
echo 'Enter this Wi-Fi IPv4 address with port 8080 in the phone APK. This helper configures local Wi-Fi only, without replacing default routes, DNS, or the BSP boot network service.'
