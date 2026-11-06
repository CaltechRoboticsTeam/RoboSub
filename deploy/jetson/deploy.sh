#!/usr/bin/env bash
# Create a reviewable release; leave existing robot programs and checkouts alone.
# Remote commands intentionally receive the validated local path and boolean.
# shellcheck disable=SC2029
set -euo pipefail

usage() {
  echo "Usage: $0 [--deploy-anyway] user@jetson"
  echo "Tests always run; --deploy-anyway allows activation if they fail."
}

field_target=""
field_deploy_anyway=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --deploy-anyway) field_deploy_anyway=true ;;
    -h|--help) usage; exit 0 ;;
    -*) usage >&2; exit 2 ;;
    *)
      if [[ -n "${field_target}" ]]; then
        usage >&2
        exit 2
      fi
      field_target="$1"
      ;;
  esac
  shift
done
if [[ -z "${field_target}" ]]; then
  usage >&2
  exit 2
fi
field_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
printf 'Control test sources on this Mac: %s/tests/control\n' "${field_root}"
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
cp "${field_root}/mkdocs.yml" "${field_stage}/mkdocs.yml"
cp "${field_root}/README.md" "${field_stage}/README.md"
cp "${field_root}/AGENTS.md" "${field_stage}/AGENTS.md"
cp "${field_root}/deploy/jetson/env.sh" "${field_stage}/env.sh"
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
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${field_release}/robosub_ws"
colcon build --symlink-install --packages-select \
  robosub_control robosub_sim robosub_bringup
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
printf 'Running tests from: %s/tests/control\n' "${field_release}"
printf 'Saving test output to: %s/control-tests.log\n' "${field_release}"
set +e
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 \
  python3 -m unittest discover -s tests/control -v 2>&1 \
  | tee "${field_release}/control-tests.log"
field_test_results=("${PIPESTATUS[@]}")
set -e
if [[ "${field_test_results[1]}" -ne 0 ]]; then
  exit "${field_test_results[1]}"
fi
python3 - "${field_release}" "${field_test_results[0]}" <<'PY'
import json
from pathlib import Path
import sys

report = {
    'test_exit_code': int(sys.argv[2]),
    'test_failures_overridden': False,
    'test_log': 'control-tests.log',
}
(Path(sys.argv[1]) / 'DEPLOYMENT.json').write_text(
    json.dumps(report, indent=2) + '\n')
PY
SH_REMOTE

ssh "${field_ssh[@]}" "${field_target}" \
  "cat '${field_remote}/DEPLOYMENT.json'" > "${field_stage}/DEPLOYMENT.json"
field_test_status="$(python3 - "${field_stage}/DEPLOYMENT.json" <<'PY'
import json
import sys

with open(sys.argv[1]) as source:
    print(json.load(source)['test_exit_code'])
PY
)"
if [[ "${field_test_status}" -ne 0 ]]; then
  printf 'Tests FAILED (exit %s). Full log: %s:%s/control-tests.log\n' \
    "${field_test_status}" "${field_target}" "${field_remote}"
  if [[ "${field_deploy_anyway}" != true ]]; then
    field_answer=""
    if [[ -t 0 ]]; then
      read -r -p 'Deploy this built release anyway? [y/N] ' field_answer || true
    fi
    case "${field_answer}" in
      [yY]|[yY][eE][sS]) field_deploy_anyway=true ;;
      *)
        echo "Not deployed; the current release is unchanged." >&2
        echo "Use --deploy-anyway to accept failed tests non-interactively." >&2
        exit 1
        ;;
    esac
  fi
  echo "Deploying with failed tests explicitly accepted."
fi

ssh "${field_ssh[@]}" "${field_target}" \
  "bash -s -- '${field_remote}' '${field_deploy_anyway}'" <<'SH_ACTIVATE'
set -euo pipefail
field_release="$1"
python3 - "${field_release}" "$2" <<'PY'
import json
import os
from pathlib import Path
import sys

release = Path(sys.argv[1])
report_path = release / 'DEPLOYMENT.json'
report = json.loads(report_path.read_text())
override = sys.argv[2] == 'true' and report['test_exit_code'] != 0
if report['test_exit_code'] != 0 and not override:
    raise SystemExit('Failed tests were not accepted; refusing activation')
current = release.parents[1] / 'current'
if current.exists() and not current.is_symlink():
    raise SystemExit('Refusing to replace a non-symlink current path')
report['test_failures_overridden'] = override
report_path.write_text(json.dumps(report, indent=2) + '\n')
temporary = current.with_name('current-' + release.name)
os.symlink(release, temporary)
os.replace(temporary, current)
print('Current release:', current, '->', release)
print('No robot programs were started or restarted.')
PY
SH_ACTIVATE
