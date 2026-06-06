#!/usr/bin/env bash
# Section 2: generate raster + spiral coverage paths, convert to joint
# trajectories via /compute_cartesian_path, and report metrics + plots.
#
# Requires move_group running (Section 1 brings it up). If not, this script
# will launch piper_no_gripper_moveit/demo.launch.py itself.
#
# Usage:   bash scripts/section2.sh
# Outputs: data/coverage_path_{counter,mirror}.csv
#          data/coverage_path.png
#          data/coverage_trajectory_{counter,mirror}.yaml

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f ${REPO_DIR}/docker/docker-compose.yml"
EXEC="${COMPOSE} exec -T humble bash -c"
EXEC_D="${COMPOSE} exec -T -d humble bash -c"

c_grn=$'\e[32m'; c_red=$'\e[31m'; c_dim=$'\e[2m'; c_off=$'\e[0m'
log()  { echo "${c_grn}[section2]${c_off} $*"; }
warn() { echo "${c_red}[section2]${c_off} $*" >&2; }

ensure_container() {
  if ! ${COMPOSE} ps --services --filter "status=running" | grep -q '^humble$'; then
    log "starting humble container"
    ${COMPOSE} up -d humble >/dev/null
    sleep 2
  fi
}

is_service_up() {
  ${EXEC} "source /opt/ros/humble/setup.bash; ros2 service list 2>/dev/null | grep -q '^$1$'"
}

wait_for_service() {
  local svc="$1" max="${2:-30}" i=0
  while ! is_service_up "$svc"; do
    i=$((i+1))
    if [ "$i" -ge "$max" ]; then warn "timed out waiting for $svc"; return 1; fi
    sleep 2; printf "."
  done
  echo " ok (${i}*2s)"
}

ensure_workspace_built() {
  ${EXEC} "test -f /home/dev/ros2_ws/install/gr_coverage/share/gr_coverage/config/coverage.yaml" \
    && return 0
  log "building gr_coverage"
  ${EXEC} "source /opt/ros/humble/setup.bash && cd /home/dev/ros2_ws && colcon build --symlink-install --packages-select gr_coverage" \
    >/dev/null
}

start_movegroup_if_needed() {
  if is_service_up /compute_cartesian_path; then
    log "move_group already running — skipping"
    return 0
  fi
  log "launching piper_no_gripper_moveit/demo.launch.py"
  ${EXEC_D} "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros2_ws/install/setup.bash
    ros2 launch piper_no_gripper_moveit demo.launch.py > /home/dev/data/demo.log 2>&1
  "
  printf "  waiting for /compute_cartesian_path"
  wait_for_service /compute_cartesian_path 30
}

push_scene_if_needed() {
  log "pushing planning scene (countertop, faucet, mirror)"
  ${EXEC} "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros2_ws/install/setup.bash
    ros2 run gr_scene scene_loader --ros-args --params-file \
      /home/dev/ros2_ws/install/gr_scene/share/gr_scene/config/scene.yaml \
      -r __node:=scene_loader
  " 2>&1 | grep -E "Loaded|ERROR" || true
}

run_planner() {
  log "running coverage planner (raster countertop + spiral mirror)"
  ${EXEC} "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros2_ws/install/setup.bash
    ros2 run gr_coverage coverage_planner \
      --ros-args --params-file /home/dev/ros2_ws/install/gr_coverage/share/gr_coverage/config/coverage.yaml \
      -r __node:=coverage_planner
  " 2>&1 | grep -E "coverage=|Wrote|ERROR"
}

summarize() {
  local dir="${REPO_DIR}/data"
  log "outputs:"
  for f in coverage_path.png coverage_path_counter.csv coverage_path_mirror.csv \
           coverage_trajectory_counter.yaml coverage_trajectory_mirror.yaml; do
    [ -f "$dir/$f" ] && echo "         ${c_dim}$dir/$f${c_off}"
  done
}

main() {
  ensure_container
  ensure_workspace_built
  start_movegroup_if_needed
  push_scene_if_needed
  run_planner
  summarize
}

main "$@"
