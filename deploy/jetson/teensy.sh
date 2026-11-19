#!/usr/bin/env bash
set -euo pipefail
teensy_release="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${teensy_release}/env.sh"
teensy_logs="${teensy_release}/../../logs"
mkdir -p "${teensy_logs}"
teensy_report="${teensy_logs}/teensy-$(date -u +%Y%m%dT%H%M%SZ)-$$.json"
exec python3 "${teensy_release}/tools/diagnostics/teensy_test.py" "$@" --output "${teensy_report}"
