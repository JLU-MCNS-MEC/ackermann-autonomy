#!/usr/bin/env bash
# Load mttcan and bring up one CAN interface on a Jetson.
#
# Usage: setup_can0.sh [interface] [bitrate]
#   ros2 run ackermann_hardware setup_can0.sh can0 500000
#
# The ackermann-can systemd service calls this same script, so manual and boot-time
# configuration cannot drift apart. When already running as root no sudo prefix is used.
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

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if (( EUID == 0 )); then
  SUDO=()
else
  SUDO=(sudo)
fi

# mttcan only ships with the Jetson L4T kernel.
if ! modinfo -n mttcan >/dev/null 2>&1; then
  echo "error: kernel module mttcan not found" >&2
  echo "this module only exists in the Jetson L4T kernel; CAN cannot be configured here" >&2
  exit 3
fi

"${SUDO[@]}" modprobe can
"${SUDO[@]}" modprobe can_raw
"${SUDO[@]}" modprobe mttcan

# The controller is missing when the device tree does not enable it.
if ! ip link show "$interface_name" >/dev/null 2>&1; then
  echo "error: interface $interface_name still missing after loading mttcan" >&2
  echo "check that the device tree exposes an enabled mttcan node for this controller" >&2
  exit 3
fi

"${SUDO[@]}" ip link set "$interface_name" down 2>/dev/null || true

if ! configure_error=$(
  "${SUDO[@]}" ip link set "$interface_name" type can bitrate "$bitrate" berr-reporting on 2>&1
); then
  # Separate a privilege problem from a genuine configuration failure.
  if [[ "$configure_error" == *"ermission denied"* ||
        "$configure_error" == *"terminal"* ||
        "$configure_error" == *"askpass"* ]]; then
    echo "error: not permitted to configure $interface_name and cannot elevate without a prompt" >&2
    echo "run this from an interactive terminal, or install the boot service once:" >&2
    echo "  sudo $script_dir/install_can_service.sh" >&2
    exit 4
  fi
  echo "error: configuring $interface_name failed: $configure_error" >&2
  exit 4
fi

"${SUDO[@]}" ip link set "$interface_name" up

ip -details link show "$interface_name"

# A controller that cannot reach the active state points at the physical layer.
can_state="$(ip -details link show "$interface_name" | grep -oE 'can state [A-Z-]+' | head -1 || true)"
case "$can_state" in
  *"ERROR-PASSIVE"* | *"BUS-OFF"*)
    echo "warning: $interface_name is ${can_state#can state }; the bus may be unterminated" >&2
    echo "check the 3.3 V transceiver wiring (40-pin header pins 29 and 31) and both" >&2
    echo "120 ohm terminators." >&2
    ;;
esac
