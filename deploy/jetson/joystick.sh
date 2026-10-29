#!/usr/bin/env bash
# Preview by default; --execute explicitly enables the live Teensy transport.
set -euo pipefail
joystick_release="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
joystick_transport=preview
if [[ "${1:-}" == --execute ]]; then
  joystick_transport=teensy_usb
  shift
fi
source "${joystick_release}/env.sh"
exec ros2 launch robosub_bringup robot.launch.py \
  mode:=real vehicle:=current_m5 vision:=false \
  teleop:=browser tuning:=true actuators:="${joystick_transport}" "$@"
