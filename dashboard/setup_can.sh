#!/usr/bin/env bash
# Bring up both MCP2515 CAN HAT interfaces on the Raspberry Pi.
#
#   sudo ./setup_can.sh            # can0 + can1 @ 500 kbps
#   sudo ./setup_can.sh 250000     # different bit rate
#   sudo ./setup_can.sh 500000 can0 can1
#
# The kernel must already enumerate the interfaces (mcp251x overlays in
# /boot/firmware/config.txt — see dashboard/README.md). Check with `ip link`.
set -euo pipefail

BITRATE="${1:-500000}"
shift || true
IFACES=("${@:-can0 can1}")
# allow "can0 can1" passed as one arg
IFACES=(${IFACES[@]})

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo (needs to configure network interfaces)." >&2
  exit 1
fi

for IF in "${IFACES[@]}"; do
  if ! ip link show "$IF" >/dev/null 2>&1; then
    echo "!! $IF not found. Is the mcp251x overlay loaded? Check /boot/firmware/config.txt and dmesg." >&2
    continue
  fi
  ip link set "$IF" down 2>/dev/null || true
  ip link set "$IF" up type can bitrate "$BITRATE" restart-ms 100
  ip link set "$IF" txqueuelen 1000
  echo "brought up $IF @ ${BITRATE} bps"
done

echo
for IF in "${IFACES[@]}"; do
  ip -details -statistics link show "$IF" 2>/dev/null | sed -n '1,3p' || true
done
