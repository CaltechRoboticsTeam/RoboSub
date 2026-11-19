#!/usr/bin/env bash
# Keep all management commands on the installed project's files and volumes.
set -euo pipefail

INSTALL_DIR=/opt/robosub-cvat

if [[ $# -eq 0 || "$1" == --help || "$1" == -h ]]; then
  cat <<'EOF'
Usage: sudo ./cvat.sh COMPOSE_COMMAND [ARGS...]

Examples:
  sudo ./cvat.sh ps
  sudo ./cvat.sh logs --tail 100 -f cvat_server
  sudo ./cvat.sh exec cvat_server python /opt/cvat/manage.py createsuperuser
  sudo ./cvat.sh stop
  sudo ./cvat.sh up -d

Uses /opt/robosub-cvat and the fixed Compose project robosub-cvat.
EOF
  exit 0
fi

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo." >&2
  exit 1
fi
if [[ ! -f "$INSTALL_DIR/.env" ]]; then
  echo "Run sudo ./setup.sh first." >&2
  exit 1
fi

# Saved deployment settings take precedence over the caller's environment.
unset CVAT_VERSION CVAT_HOST CVAT_PORT CVAT_LISTEN_IP
exec docker --host unix:///var/run/docker.sock compose \
  --project-name robosub-cvat \
  --project-directory "$INSTALL_DIR/source" \
  --env-file "$INSTALL_DIR/.env" \
  -f "$INSTALL_DIR/source/docker-compose.yml" \
  -f "$INSTALL_DIR/compose.override.yaml" "$@"
