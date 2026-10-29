#!/usr/bin/env bash
# Create a reviewable release; leave existing robot programs and checkouts alone.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 user@jetson" >&2
  exit 2
fi
field_target="$1"
field_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
field_stamp="$(date -u +%Y%m%dT%H%M%SZ)"
field_revision="$(git -C "${field_root}" rev-parse --short=10 HEAD)"
field_stage="$(mktemp -d "${TMPDIR:-/tmp}/robosub-deploy.XXXXXX")"
trap 'rm -rf -- "${field_stage}"' EXIT
field_ssh=(-o ConnectTimeout=8)
if [[ -n "${ROBOSUB_SSH_CONTROL_PATH:-}" ]]; then
  field_ssh+=(-S "${ROBOSUB_SSH_CONTROL_PATH}")
fi
# Copy the selected runtime sources, guides and tests into a fresh local stage.
for field_path in robosub_ws/src docs tests/control tools deploy/jetson firmware; do
  mkdir -p "${field_stage}/$(dirname -- "${field_path}")"
  rsync -a --exclude __pycache__ --exclude '*.pyc' --exclude .DS_Store \
    "${field_root}/${field_path}" \
    "${field_stage}/$(dirname -- "${field_path}")/"
done
mkdir -p "${field_stage}/reference"
cp "${field_root}/prototypes/SummerTesting/MAVROSPixhawkDVLTest/my_mav_node/flo_startup_file" \
  "${field_stage}/reference/flo_startup_file"
cp "${field_root}/prototypes/eddie/Thruster_MavLink.ino" \
  "${field_stage}/reference/Thruster_MavLink.ino"
cp "${field_root}/mkdocs.yml" "${field_stage}/mkdocs.yml"
cp "${field_root}/README.md" "${field_stage}/README.md"
cp "${field_root}/AGENTS.md" "${field_stage}/AGENTS.md"
cp "${field_root}/deploy/jetson/env.sh" "${field_stage}/env.sh"
cp "${field_root}/deploy/jetson/field.sh" "${field_stage}/field.sh"
cp "${field_root}/deploy/jetson/teensy.sh" "${field_stage}/teensy.sh"
cp "${field_root}/deploy/jetson/joystick.sh" "${field_stage}/joystick.sh"
python3 - "${field_stage}" "${field_revision}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
files = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
         for path in sorted(root.rglob('*')) if path.is_file()}
(root / 'SNAPSHOT.json').write_text(json.dumps({
    'base_revision': sys.argv[2], 'includes_uncommitted_files': True,
    'sha256': files,
}, indent=2) + '\n')
PY
field_remote="$(ssh "${field_ssh[@]}" "${field_target}" \
  'mkdir -p "$HOME/robosub-field/releases" && mktemp -d '"\"\$HOME/robosub-field/releases/${field_stamp}-${field_revision}-XXXXXX\"")"
# mktemp produces an absolute path. Reject unexpected shell/path characters.
if [[ ! "${field_remote}" =~ ^/home/[a-zA-Z0-9_-]+/robosub-field/releases/[a-zA-Z0-9_-]+$ ]]; then
  echo "Unexpected deployment path: ${field_remote}" >&2
  exit 1
fi
field_transport="ssh -o ConnectTimeout=8"
if [[ -n "${ROBOSUB_SSH_CONTROL_PATH:-}" ]]; then
  if [[ ! "${ROBOSUB_SSH_CONTROL_PATH}" =~ ^/[a-zA-Z0-9_./-]+$ ]]; then
    echo "Invalid SSH control path" >&2
    exit 1
  fi
  field_transport+=" -S ${ROBOSUB_SSH_CONTROL_PATH}"
fi
rsync -a -e "${field_transport}" "${field_stage}/" \
  "${field_target}:${field_remote}/"
ssh "${field_ssh[@]}" "${field_target}" \
  "bash -s -- '${field_remote}'" <<'SH_REMOTE'
set -euo pipefail
field_release="$1"
python3 - "${field_release}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
manifest = json.loads((root / 'SNAPSHOT.json').read_text())
for name, expected in manifest['sha256'].items():
    actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit('Transfer checksum mismatch: ' + name)
print('Verified transferred files:', len(manifest['sha256']))
PY
# Preserve the actual legacy Jetson sources as release-local reference data.
field_legacy="$HOME/Robosub/prototypes/SummerTesting/MAVROSPixhawkDVLTest/my_mav_node"
if [[ -d "${field_legacy}" ]]; then
  mkdir -p "${field_release}/reference"
  cp -a "${field_legacy}" "${field_release}/reference/legacy_jetson"
fi
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${field_release}/robosub_ws"
colcon build --symlink-install --packages-select robosub_sim robosub_bringup
set +u
source install/setup.bash
set -u
cd "${field_release}"
python3 - <<'PY_DOMAIN'
from pathlib import Path

for process in Path('/proc').iterdir():
    if not process.name.isdecimal():
        continue
    try:
        argv = (process / 'cmdline').read_bytes().decode().split('\0')
        if not argv or not argv[0].endswith('/mavros_node'):
            continue
        values = dict(item.split('=', 1) for item in
                      (process / 'environ').read_bytes().decode().split('\0')
                      if '=' in item)
        if int(values.get('ROS_DOMAIN_ID', '0')) == 198:
            raise SystemExit('MAVROS uses test domain 198; refusing fake tests')
    except (OSError, UnicodeError):
        continue
PY_DOMAIN
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 \
  python3 -m unittest discover -s tests/control -v
python3 - "${field_release}" <<'PY'
import os
from pathlib import Path
import sys

release = Path(sys.argv[1])
current = release.parents[1] / 'current'
if current.exists() and not current.is_symlink():
    raise SystemExit('Refusing to replace a non-symlink current path')
temporary = current.with_name('current-' + release.name)
os.symlink(release, temporary)
os.replace(temporary, current)
print('Current release:', current, '->', release)
print('No robot programs were started or restarted.')
PY
SH_REMOTE
printf 'Field tools: %s/field.sh\n' "${field_remote}"
