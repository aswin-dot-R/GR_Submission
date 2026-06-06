#!/usr/bin/env bash
# Host prep + container bring-up for Section 1 (run in a REAL terminal — needs sudo).
#
# Why this exists:
#   /tmp/.docker.xauth must be a FILE holding the X11 cookie. A previous failed
#   `docker compose up` left it as an empty root-owned DIRECTORY, so the bind
#   mount in docker-compose.yml fails with:
#     "not a directory: Are you trying to mount a directory onto a file..."
#   This script removes the stray dir, reseeds the cookie file, allows local
#   X access, and starts the humble container.
#
# Usage:
#   bash scripts/host_up.sh
#
# After it finishes, Claude (or you) can drive the sweep with plain `docker
# exec` calls — no sudo needed for those.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f ${REPO_DIR}/docker/docker-compose.yml"
DISP="${DISPLAY:-:1}"

grn=$'\e[32m'; red=$'\e[31m'; off=$'\e[0m'
log()  { echo "${grn}[host_up]${off} $*"; }
warn() { echo "${red}[host_up]${off} $*" >&2; }

log "[1/4] Fixing /tmp/.docker.xauth (needs sudo)"
if [ -d /tmp/.docker.xauth ]; then
  warn "  /tmp/.docker.xauth is a directory — removing it"
fi
sudo rm -rf /tmp/.docker.xauth
sudo touch /tmp/.docker.xauth
sudo chmod 644 /tmp/.docker.xauth
# Seed the X cookie with a wildcard hostname so any container host can use it.
xauth nlist "$DISP" | sed -e 's/^..../ffff/' | sudo xauth -f /tmp/.docker.xauth nmerge - || \
  warn "  xauth nmerge failed (GUI apps may not display, but headless sweep is fine)"
ls -la /tmp/.docker.xauth

log "[2/4] Allowing local Docker containers to reach the X server"
xhost +local:docker || warn "  xhost failed (only matters for RViz GUI)"

log "[3/4] Starting humble container"
${COMPOSE} up -d humble

log "[4/4] Verifying ROS inside container"
sleep 3
${COMPOSE} exec -T humble bash -c '
  source /opt/ros/humble/setup.bash
  echo "  ROS_DISTRO=$ROS_DISTRO"
  if [ -f /home/dev/ros2_ws/install/setup.bash ]; then
    echo "  workspace: BUILT"
  else
    echo "  workspace: NOT BUILT (run colcon build inside the container)"
  fi
' || warn "  could not exec into container"

echo
log "Done. Container 'gr_humble' is up."
log "Next: re-run Section 1 with  ->  bash scripts/section1.sh"
log "  (or let Claude drive the sweep + diagnostics via docker exec)"
