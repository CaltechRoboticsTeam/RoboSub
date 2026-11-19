#!/usr/bin/env bash
# Install the team's CVAT deployment on a systemd Linux server.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
INSTALL_DIR=/opt/robosub-cvat
SOURCE_DIR="$INSTALL_DIR/source"
CVAT_VERSION=v2.80.0
CVAT_COMMIT=bb0a873fe071e70d71458cd82127052025e05807
DOWNLOAD_DIR=
HOST_VALUE=
PORT_VALUE=
BIND_VALUE=

usage() {
  cat <<'EOF'
Usage: sudo ./setup.sh [--host NAME_OR_IPV4] [--port PORT] [--bind IPV4]

Installs Docker Engine/Compose and CVAT on Ubuntu 22.04/24.04 or Debian 12/13
(amd64, systemd). Existing working Docker installations are reused.

Without --host, a first install serves localhost on 127.0.0.1:8080.
With --host, a first install listens on all IPv4 interfaces by default.
Reruns preserve saved settings unless the corresponding option is supplied.

Example: sudo ./setup.sh --host 192.168.1.50
EOF
}

fail() {
  echo "Error: $*" >&2
  exit 1
}

cleanup() {
  if [[ -n "$DOWNLOAD_DIR" ]]; then
    rm -rf -- "$DOWNLOAD_DIR"
  fi
}
trap cleanup EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host|--port|--bind)
      [[ $# -ge 2 && -n "$2" ]] || fail "$1 needs a value."
      case "$1" in
        --host) HOST_VALUE=$2 ;;
        --port) PORT_VALUE=$2 ;;
        --bind) BIND_VALUE=$2 ;;
      esac
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; fail "Unknown option: $1" ;;
  esac
done

[[ "$(uname -s)" == Linux ]] || fail "Run this on the Linux server."
[[ $EUID -eq 0 ]] || fail "Run with sudo."
[[ -f /etc/os-release ]] || fail "/etc/os-release is missing."
# shellcheck source=/dev/null
source /etc/os-release
case "$ID:$VERSION_ID" in
  ubuntu:22.04|ubuntu:24.04|debian:12|debian:13) ;;
  *) fail "Use Ubuntu 22.04/24.04 or Debian 12/13." ;;
esac
[[ "$(dpkg --print-architecture)" == amd64 ]] ||
  fail "This deployment targets an amd64 server, not the Jetson."
[[ -d /run/systemd/system ]] || fail "systemd must be running."

umask 027
install -d -m 0750 "$INSTALL_DIR"
exec 9>"$INSTALL_DIR/setup.lock"
flock -n 9 || fail "Another setup is running."

read_setting() {
  if [[ -f "$INSTALL_DIR/.env" ]]; then
    sed -n "s/^$1=//p" "$INSTALL_DIR/.env"
  fi
}

HOST_VALUE=${HOST_VALUE:-$(read_setting CVAT_HOST)}
HOST_VALUE=${HOST_VALUE:-localhost}
PORT_VALUE=${PORT_VALUE:-$(read_setting CVAT_PORT)}
PORT_VALUE=${PORT_VALUE:-8080}
BIND_VALUE=${BIND_VALUE:-$(read_setting CVAT_LISTEN_IP)}
if [[ -z "$BIND_VALUE" ]]; then
  if [[ "$HOST_VALUE" == localhost || "$HOST_VALUE" == 127.0.0.1 ]]; then
    BIND_VALUE=127.0.0.1
  else
    BIND_VALUE=0.0.0.0
  fi
fi

[[ "$HOST_VALUE" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ ]] ||
  fail "--host must be a hostname or IPv4 address, without scheme or port."
[[ "$PORT_VALUE" =~ ^[0-9]{1,5}$ ]] || fail "Invalid port."
PORT_VALUE=$((10#$PORT_VALUE))
((PORT_VALUE >= 1 && PORT_VALUE <= 65535)) || fail "Invalid port."
[[ "$BIND_VALUE" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] ||
  fail "--bind must be an IPv4 address."
IFS=. read -r -a bind_octets <<< "$BIND_VALUE"
for octet in "${bind_octets[@]}"; do
  ((10#$octet <= 255)) || fail "Invalid bind address."
done

install_docker=false
if ! command -v docker >/dev/null || ! docker compose version >/dev/null 2>&1;
then
  install_docker=true
  for package in docker.io docker-compose docker-doc podman-docker \
    containerd runc; do
    if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" \
      == 'install ok installed' ]]; then
      fail "Conflicting package $package is installed. See README.md."
    fi
  done
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y ca-certificates curl git
if [[ "$install_docker" == true ]]; then
  install -d -m 0755 /etc/apt/keyrings
  curl --fail --silent --show-error --location \
    "https://download.docker.com/linux/$ID/gpg" \
    -o /etc/apt/keyrings/docker.asc
  chmod 0644 /etc/apt/keyrings/docker.asc
  cat > /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/$ID
Suites: $VERSION_CODENAME
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
  apt-get update
  apt-get install -y docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin
fi
systemctl enable --now docker.service containerd.service
docker --host unix:///var/run/docker.sock info >/dev/null

if [[ -d "$SOURCE_DIR" ]]; then
  [[ "$(git -C "$SOURCE_DIR" rev-parse HEAD)" == "$CVAT_COMMIT" ]] ||
    fail "Installed CVAT version differs. Back up before a manual upgrade."
  git -C "$SOURCE_DIR" diff --quiet HEAD -- ||
    fail "Installed upstream files have edits; preserve them before setup."
else
  DOWNLOAD_DIR=$(mktemp -d "$INSTALL_DIR/download.XXXXXX")
  # Non-root containers need to read public bind-mounted upstream configs.
  (
    umask 022
    git -c advice.detachedHead=false clone --depth 1 --branch "$CVAT_VERSION" \
      https://github.com/cvat-ai/cvat.git "$DOWNLOAD_DIR/source"
  )
  [[ "$(git -C "$DOWNLOAD_DIR/source" rev-parse HEAD)" == "$CVAT_COMMIT" ]] ||
    fail "Upstream tag no longer matches the pinned commit."
  mv -- "$DOWNLOAD_DIR/source" "$SOURCE_DIR"
fi

cat > "$INSTALL_DIR/.env" <<EOF
CVAT_VERSION=$CVAT_VERSION
CVAT_HOST=$HOST_VALUE
CVAT_PORT=$PORT_VALUE
CVAT_LISTEN_IP=$BIND_VALUE
EOF
chmod 0600 "$INSTALL_DIR/.env"
install -m 0644 "$SCRIPT_DIR/compose.override.yaml" "$INSTALL_DIR/"
install -m 0755 "$SCRIPT_DIR/cvat.sh" "$INSTALL_DIR/"

echo "Checking configuration (Compose 2.24.4 or newer is required)..."
"$INSTALL_DIR/cvat.sh" config --quiet
"$INSTALL_DIR/cvat.sh" pull
"$INSTALL_DIR/cvat.sh" up -d

probe_host=$BIND_VALUE
if [[ "$probe_host" == 0.0.0.0 ]]; then
  probe_host=127.0.0.1
fi
echo "Waiting for CVAT's API and database migrations (about ten minutes)..."
ready=false
deadline=$((SECONDS + 600))
while ((SECONDS < deadline)); do
  if curl --fail --silent --location --output /dev/null --connect-timeout 2 \
    --max-time 5 --header "Host: $HOST_VALUE" \
    "http://$probe_host:$PORT_VALUE/api/server/about"; then
    ready=true
    break
  fi
  sleep 5
done
if [[ "$ready" != true ]]; then
  "$INSTALL_DIR/cvat.sh" ps
  fail "API not ready. Inspect: sudo ./cvat.sh logs --tail 100 cvat_server"
fi

cat <<EOF

CVAT is ready at http://$HOST_VALUE:$PORT_VALUE
Docker starts at boot; the containers restart unless manually stopped.
Data stays in Docker volumes. Installed files: $INSTALL_DIR

Create the first administrator (interactive username/password prompt):
  sudo ./cvat.sh exec cvat_server python /opt/cvat/manage.py createsuperuser

Then create the team organization and follow docs/software/cvat.md.
EOF
