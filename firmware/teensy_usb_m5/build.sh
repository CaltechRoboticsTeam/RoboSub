#!/usr/bin/env bash
# Compile only. This script never uploads firmware or opens a device.
set -euo pipefail
teensy_mode="${1:-usb-only}"
teensy_passthrough=0
case "${teensy_mode}" in
  usb-only) teensy_uart=0 ;;
  rs485-serial1) teensy_uart=1 ;;
  rs485-serial2) teensy_uart=2 ;;
  passthrough-serial1) teensy_uart=1; teensy_passthrough=1 ;;
  passthrough-serial2) teensy_uart=2; teensy_passthrough=1 ;;
  *)
    echo "Usage: $0 [usb-only|rs485-serial{1,2}|passthrough-serial{1,2}]" >&2
    exit 2
    ;;
esac
if [[ $# -gt 1 ]]; then
  echo "Only one build mode may be selected" >&2
  exit 2
fi
teensy_source="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
teensy_root="$(cd -- "${teensy_source}/../.." && pwd -P)"
teensy_stage="$(mktemp -d "${TMPDIR:-/tmp}/robosub-teensy.XXXXXX")"
trap 'rm -rf -- "${teensy_stage}"' EXIT
mkdir -p "${teensy_stage}/teensy_usb_m5"
cp "${teensy_source}/teensy_usb_m5.ino" "${teensy_source}/protocol.h" \
  "${teensy_source}/hardware_config.h" "${teensy_stage}/teensy_usb_m5/"
python3 - "${teensy_stage}/teensy_usb_m5/hardware_config.h" \
  "${teensy_uart}" "${teensy_passthrough}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
path.write_text(path.read_text().replace(
    '#define ROBOSUB_RS485_SERIAL 0',
    '#define ROBOSUB_RS485_SERIAL ' + sys.argv[2],
).replace(
    '#define ROBOSUB_RS485_PASSTHROUGH 0',
    '#define ROBOSUB_RS485_PASSTHROUGH ' + sys.argv[3],
))
PY
teensy_build="${teensy_root}/.cache/teensy-build-${teensy_mode}"
teensy_output="${teensy_root}/firmware/build/${teensy_mode}"
arduino-cli compile --fqbn teensy:avr:teensy41 --warnings all \
  --build-property 'recipe.hooks.postbuild.1.pattern=' \
  --build-path "${teensy_build}" "${teensy_stage}/teensy_usb_m5"
mkdir -p "${teensy_output}"
cp "${teensy_build}/teensy_usb_m5.ino.hex" "${teensy_output}/"
python3 - "${teensy_stage}/teensy_usb_m5" "${teensy_output}" \
  "${teensy_uart}" "${teensy_passthrough}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

source, output = map(Path, sys.argv[1:3])
firmware = output / 'teensy_usb_m5.ino.hex'
manifest = {
    'board': 'Teensy 4.1', 'fqbn': 'teensy:avr:teensy41',
    'rs485_serial': int(sys.argv[3]),
    'rs485_passthrough': bool(int(sys.argv[4])),
    'firmware_sha256': hashlib.sha256(firmware.read_bytes()).hexdigest(),
    'source_sha256': {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(source.iterdir()) if path.is_file()
    },
}
(output / 'BUILD.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Compiled only:', firmware)
print('RS-485 UART:', manifest['rs485_serial'], '(0 means no bus transmission)')
print('Terminal passthrough:', manifest['rs485_passthrough'])
PY
