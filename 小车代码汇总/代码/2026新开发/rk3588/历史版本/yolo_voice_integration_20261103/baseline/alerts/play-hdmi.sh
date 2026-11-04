#!/bin/sh
set -eu
WAV=${1:?WAV file required}
[ -r "$WAV" ] || exit 2
# Follow hotplug; only HDMI sinks are eligible, never the analogue codec.
CARD=
for spec in '1:rockchiphdmi0' '2:rockchiphdmi1'; do
    connector=${spec%:*}
    for status in /sys/class/drm/card*-HDMI-A-"$connector"/status; do
        [ -r "$status" ] || continue
        if [ "$(cat "$status")" = connected ]; then CARD=${spec#*:}; break; fi
    done
    [ -z "$CARD" ] || break
done
if [ -z "$CARD" ]; then echo 'PPE_VOICE: no connected HDMI output' >&2; exit 3; fi
export PULSE_SERVER=${PULSE_SERVER:-unix:/tmp/pulse-socket}
if command -v pactl >/dev/null && command -v paplay >/dev/null; then
    SINK=$(pactl list short sinks 2>/dev/null | awk -v card="$CARD" 'index($2,card) {print $2;exit}')
    if [ -z "$SINK" ]; then
        SINK=ppe_$CARD
        # This dedicated sink does not change the system default output.
        if ! pactl load-module module-alsa-sink device="plughw:CARD=$CARD,DEV=0" sink_name="$SINK" channels=2 rate=48000 >/dev/null 2>&1; then SINK=; fi
    fi
    if [ -n "$SINK" ]; then
        exec paplay --device="$SINK" --volume=52428 "$WAV"
    fi
fi
exec aplay -q -D "plughw:CARD=$CARD,DEV=0" "$WAV"
