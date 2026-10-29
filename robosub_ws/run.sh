#!/usr/bin/env bash
# Start (or attach to) the RoboSub ROS 2 Humble + Gazebo Harmonic container.
set -euo pipefail
cd "$(dirname "$0")/docker"

# Hand the container an X cookie so GUI apps (Gazebo, RViz) can draw.
XAUTH=/tmp/.docker.xauth
if [ ! -f "$XAUTH" ]; then
  touch "$XAUTH"
  xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
  chmod 644 "$XAUTH"
fi

if [ "$(docker ps -q -f name=^robosub$)" ]; then
  echo ">> attaching to running container"
  exec docker exec -it robosub bash
fi

compose_files=(-f compose.yaml)
if [[ "${ROBOSUB_GAMEPAD:-0}" == "1" ]]; then
  if [[ ! -d /dev/input ]]; then
    echo "Local controller input requires a Linux host with /dev/input." >&2
    exit 1
  fi
  compose_files+=(-f compose.gamepad.yaml)
fi
docker compose "${compose_files[@]}" run --rm --name robosub robosub bash
