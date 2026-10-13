#!/usr/bin/env bash
# Install the ackermann-can systemd service so the Jetson loads mttcan and brings the
# CAN interface up at boot. After installation neither ROS launch nor the operator needs
# sudo to reach the chassis bus.
#
# Usage (requires root):
#   sudo "$(ros2 pkg prefix ackermann_hardware)/lib/ackermann_hardware/install_can_service.sh"
#
# Re-running is safe: the unit is regenerated and /etc/default/ackermann-can is kept.
set -euo pipefail

unit_name='ackermann-can.service'
unit_target="/etc/systemd/system/$unit_name"
env_target='/etc/default/ackermann-can'

if (( EUID != 0 )); then
  echo "error: installing a systemd service requires root" >&2
  echo '  sudo "$(ros2 pkg prefix ackermann_hardware)/lib/ackermann_hardware/install_can_service.sh"' >&2
  exit 1
fi

model_path='/proc/device-tree/model'
if [[ ! -r "$model_path" ]] || ! tr -d '\0' <"$model_path" | grep -q 'Jetson'; then
  echo "error: no Jetson platform detected" >&2
  echo "this service needs the mttcan module from the Jetson L4T kernel" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setup_script="$script_dir/setup_can0.sh"
share_dir="$(cd "$script_dir/../.." && pwd)/share/ackermann_hardware"
unit_template="$share_dir/systemd/${unit_name}.in"
env_template="$share_dir/systemd/ackermann-can.env"

for required in "$setup_script" "$unit_template" "$env_template"; do
  if [[ ! -f "$required" ]]; then
    echo "error: missing $required" >&2
    echo "build the workspace with colcon build --symlink-install and source the install tree" >&2
    exit 3
  fi
done

# Both paths are baked into the unit so it never depends on the workspace or shell env.
sed -e "s|@SETUP_CAN_SCRIPT@|$setup_script|g" \
  -e "s|@SHARE_DIR@|$share_dir|g" \
  "$unit_template" >"$unit_target"
chmod 644 "$unit_target"
echo "wrote unit: $unit_target"

if [[ -f "$env_target" ]]; then
  echo "kept existing configuration: $env_target"
else
  install -m 644 "$env_template" "$env_target"
  echo "wrote default configuration: $env_target"
fi

systemctl daemon-reload
systemctl enable "$unit_name" >/dev/null
echo "enabled at boot: $unit_name"

# Try once now, but report the status instead of aborting when it fails.
if systemctl start "$unit_name"; then
  echo "service started"
else
  echo "service failed to start; see the status below" >&2
fi

echo
systemctl status "$unit_name" --no-pager || true

echo
echo "verify with:"
echo "  ip -details link show $(sed -n 's/^CAN_INTERFACE=//p' "$env_target" | head -1)"
echo "  ros2 run ackermann_hardware hardware_preflight"
