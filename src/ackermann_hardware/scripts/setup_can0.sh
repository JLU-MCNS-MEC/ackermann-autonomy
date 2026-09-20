#!/usr/bin/env bash
set -euo pipefail

interface_name="${1:-can0}"
bitrate="${2:-500000}"

if [[ ! "$interface_name" =~ ^can[0-9]+$ ]]; then
  echo "invalid CAN interface: $interface_name" >&2
  exit 2
fi
if [[ ! "$bitrate" =~ ^[0-9]+$ ]] || (( bitrate <= 0 )); then
  echo "invalid CAN bitrate: $bitrate" >&2
  exit 2
fi

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set "$interface_name" down 2>/dev/null || true
sudo ip link set "$interface_name" type can bitrate "$bitrate" berr-reporting on
sudo ip link set "$interface_name" up
ip -details link show "$interface_name"
