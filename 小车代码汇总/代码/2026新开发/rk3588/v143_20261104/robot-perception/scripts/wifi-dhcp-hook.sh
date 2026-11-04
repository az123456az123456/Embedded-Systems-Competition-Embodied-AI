#!/bin/sh
# Local hotspot link only: do not consume DHCP router/DNS options.
set -eu
case "${1:-}" in
 bound|renew)
  [ -n "${interface:-}" ] && [ -n "${ip:-}" ] && [ -n "${subnet:-}" ] || exit 1
  ifconfig "$interface" "$ip" netmask "$subnet" up
 ;;
 deconfig) :;;
esac
