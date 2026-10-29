#!/usr/bin/env bash
set -euo pipefail
teensy_release="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${teensy_release}/env.sh"
teensy_logs="${teensy_release}/../../logs"
mkdir -p "${teensy_logs}"
if [[ "${1:-}" == relay ]]; then
  shift
  exec ros2 run robosub_bringup teensy_relay "$@"
fi
teensy_report="${teensy_logs}/teensy-$(date -u +%Y%m%dT%H%M%SZ)-$$.json"
exec ros2 run robosub_bringup teensy_test "$@" --output "${teensy_report}"
