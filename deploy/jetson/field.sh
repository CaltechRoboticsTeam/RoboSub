#!/usr/bin/env bash
set -euo pipefail
field_release_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${field_release_dir}/env.sh"
field_logs="${field_release_dir}/../../logs"
mkdir -p "${field_logs}"
field_report="${field_logs}/$(date -u +%Y%m%dT%H%M%SZ)-$$.json"
exec ros2 run robosub_bringup field_test "$@" --output "${field_report}"
