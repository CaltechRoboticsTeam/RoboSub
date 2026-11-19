#!/usr/bin/env bash
# Start (or attach to) the RoboSub ROS 2 Humble + Gazebo Harmonic container.
set -euo pipefail
cd "$(dirname "$0")/docker"

# SSH shells can reuse the logged-in desktop's display and authentication.
if [[ -z "${DISPLAY:-}" ]] && command -v systemctl >/dev/null; then
  while IFS='=' read -r name value; do
    case "$name" in
      DISPLAY|XAUTHORITY) export "$name=$value" ;;
    esac
  done < <(systemctl --user show-environment)
fi
: "${DISPLAY:?Run from a desktop session or set DISPLAY and XAUTHORITY.}"

# Keep the X cookie private and refresh it after desktop logins.
cookie_directory="${XDG_RUNTIME_DIR:-$HOME/.cache}"
mkdir -p "$cookie_directory"
export ROBOSUB_XAUTHORITY="$cookie_directory/robosub-$(id -u).xauth"
touch "$ROBOSUB_XAUTHORITY"
chmod 600 "$ROBOSUB_XAUTHORITY"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' \
  | xauth -f "$ROBOSUB_XAUTHORITY" nmerge -

exec_flags=(-i)
compose_flags=()
if [[ -t 0 && -t 1 ]]; then
  exec_flags+=(-t)
else
  compose_flags+=(-T)
fi

container_command=(bash)
if [[ $# -gt 0 ]]; then
  container_command=(bash -lc '
    source /opt/ros/humble/setup.bash
    if [[ -f /ws/install/setup.bash ]]; then
      source /ws/install/setup.bash
    fi
    exec "$@"
  ' bash "$@")
fi

if [[ -n "$(docker ps -q -f name=^robosub$)" ]]; then
  echo ">> attaching to running container"
  exec docker exec "${exec_flags[@]}" robosub "${container_command[@]}"
fi

compose_files=(-f compose.yaml)
if [[ "${ROBOSUB_GAMEPAD:-0}" == "1" ]]; then
  if [[ ! -d /dev/input ]]; then
    echo "Local controller input requires a Linux host with /dev/input." >&2
    exit 1
  fi
  compose_files+=(-f compose.gamepad.yaml)
fi
exec docker compose "${compose_files[@]}" run "${compose_flags[@]}" \
  --rm --name robosub robosub "${container_command[@]}"
