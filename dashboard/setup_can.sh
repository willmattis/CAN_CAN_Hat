#!/usr/bin/env bash
# Bring up the MCP2515 CAN HAT interfaces on the Raspberry Pi.
#
#   sudo ./setup_can.sh            # can0 + can1 @ 500 kbps
#   sudo ./setup_can.sh 250000     # different bit rate
#   sudo ./setup_can.sh 500000 can0 can1
#
# Robust for boot ordering: waits for each interface to appear, settles the
# controller between down/up, retries, and never hard-fails the whole unit on
# one interface (so a slow/missing HAT can't take down the service). The kernel
# must enumerate the interfaces via the mcp251x overlays in
# /boot/firmware/config.txt (see dashboard/README.md).

BITRATE="${1:-500000}"
shift || true
IFACES=(${@:-can0 can1})       # unquoted on purpose: split "can0 can1" into array

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo (needs to configure network interfaces)." >&2
  exit 1
fi

up_one() {
  local IF="$1" i
  # Wait up to ~10 s for the interface to appear (the SPI chip can probe late).
  for i in $(seq 1 20); do
    ip link show "$IF" >/dev/null 2>&1 && break
    sleep 0.5
  done
  if ! ip link show "$IF" >/dev/null 2>&1; then
    echo "!! $IF never appeared (mcp251x overlay loaded? HAT seated? check dmesg)" >&2
    return 1
  fi
  # Bring up, retrying a few times: a down->up done back-to-back can race the
  # mcp251x driver (the settle sleep is what the manual command does implicitly).
  for i in 1 2 3; do
    ip link set "$IF" down 2>/dev/null || true
    sleep 0.3
    if ip link set "$IF" up type can bitrate "$BITRATE" restart-ms 100; then
      ip link set "$IF" txqueuelen 1000 2>/dev/null || true
      echo "brought up $IF @ ${BITRATE} bps"
      return 0
    fi
    echo "retry $IF ($i)..." >&2
    sleep 0.5
  done
  echo "!! failed to bring up $IF after retries" >&2
  return 1
}

for IF in "${IFACES[@]}"; do
  up_one "$IF" || true          # one bad interface must not fail the whole unit
done

echo
for IF in "${IFACES[@]}"; do
  ip -details -statistics link show "$IF" 2>/dev/null | sed -n '1,3p' || true
done
exit 0                          # always succeed: brought up whatever was present
