# CVAT server

Run this on a dedicated **Ubuntu 24.04 amd64 server** with systemd and internet
access. Ubuntu 22.04 and Debian 12/13 are also accepted. This is a labeling
server, separate from the Jetson; a GPU is not needed for manual annotation.
For a team, start with several CPU cores, 16 GB RAM, and SSD space for the
recordings, extracted images, exports, and backups.

## Install

From a clone of this repository on the Linux server:

```bash
cd deploy/cvat
sudo ./setup.sh --host 192.168.1.50
```

Replace the example IP with the server's actual LAN IPv4 address or DNS name.
Use a stable address that students can reach. Open `http://192.168.1.50:8080`
in Chrome after setup reports ready. Use that same hostname/IP in the browser;
CVAT routes requests by hostname. DNS names must resolve to this server.

The script installs Docker Engine and Compose from Docker's signed apt
repository if needed, enables Docker at boot, downloads CVAT **v2.80.0** at a
checked commit, pulls its images, and starts the official multi-container
deployment. No SAM or automatic annotation models are installed.

Create an administrator with an interactive password prompt:

```bash
sudo ./cvat.sh exec cvat_server python /opt/cvat/manage.py createsuperuser
```

Log in, create the team organization/project, and invite students with their
own accounts. Continue with the [labeling guide](../../docs/software/cvat.md).

Options:

```bash
# Local access only; useful before arranging network access.
sudo ./setup.sh
# Custom port and a specific server interface.
sudo ./setup.sh --host 192.168.1.50 --bind 192.168.1.50 --port 8088
```

On a first install, `--host` enables listening on all IPv4 interfaces;
otherwise it binds to localhost. Reruns preserve saved options, so changing
from localhost to LAN access also needs `--bind 0.0.0.0`. Settings live in
`/opt/robosub-cvat/.env`. Rerunning setup keeps the same volumes and CVAT
release; it does not create another admin or erase annotations.

This serves **HTTP for a trusted LAN/VPN**. For public internet access, add
HTTPS using the [upstream deployment instructions][install] before exposing
the service. Docker-published ports can bypass UFW rules; use the bind address
and upstream network filtering to control access. The script does not change
firewall rules or configure a domain, router, or VPN.

## Restarts and storage

Docker starts when Linux boots. Every service uses `unless-stopped`, so Docker
restarts exited containers and brings them back after a host reboot. Manually
stopped containers stay stopped until `up -d`. This follows Docker's
[restart policy][restart], without a second process manager fighting Docker.

This does not reboot a frozen Linux host or switch on an unpowered machine.
For recovery after a power outage, enable the machine's restore-on-AC-power
setting in its BIOS/firmware where available. Save labeling work regularly.

Uploads, annotations/database, keys, queues, and event data use named Docker
volumes with the fixed `robosub-cvat` project prefix. They persist across
reboots, container recreation, and repository moves. Docker manages the volume
location; inspect it with `sudo docker volume ls`. Deployment files stay in
`/opt/robosub-cvat`, so keep that directory as well. Container stdout/stderr
logs are rotated; CVAT's separate application logs, data, and exports still
need disk-space monitoring.

## Manage and back up

Run these from this folder, or use `/opt/robosub-cvat/cvat.sh` directly:

```bash
sudo ./cvat.sh ps
sudo ./cvat.sh logs --tail 100 -f cvat_server
sudo ./cvat.sh restart cvat_server
sudo ./cvat.sh stop
sudo ./cvat.sh up -d
sudo docker system df
df -h
```

Make regular project/task **backups from CVAT**, including images, and copy
them to another machine or disk alongside the original recordings. Dataset
exports are useful training releases; keep native backups for restoration.
For full server recovery, follow the [CVAT backup guide][backup] for the
database and data/key volumes, using this deployment's volume names. Preserve
the installed `.env`, source commit, and override too, and check a restore on
a separate server. Persistent volumes alone are not a backup.

Do not use `down --volumes` or prune this deployment's volumes unless you
intend to delete its data. Upgrades are manual: back up, read CVAT's release
and migration instructions, and review the pinned commit/override together.
There is no automatic image updater or scheduled host reboot.

## If setup stops

- On a fresh server, the initial image downloads and migrations can take
  several minutes. A readiness timeout leaves the containers running for
  diagnosis; check `ps` and the server logs, then rerun setup.
- An existing working Docker/Compose installation is reused. Compose must be
  **2.24.4 or newer** for the port override. If Docker is missing or incomplete
  and conflicting distro packages are installed, setup stops without removing
  them. Resolve the package conflict with Docker's [Ubuntu][ubuntu] or
  [Debian][debian] instructions, then rerun.
- If local access works but students cannot connect, check the configured
  hostname, bind address, port, routing, and campus/server firewall policy.
- All services should show `Up`. A restart policy handles a process that exits;
  it does not detect every hung or unhealthy application. Investigate repeated
  restarts or a nonresponsive API using the logs.

[install]: https://docs.cvat.ai/docs/administration/community/basics/installation/
[backup]: https://docs.cvat.ai/docs/administration/community/advanced/backup_guide/
[restart]: https://docs.docker.com/engine/containers/start-containers-automatically/
[ubuntu]: https://docs.docker.com/engine/install/ubuntu/
[debian]: https://docs.docker.com/engine/install/debian/
