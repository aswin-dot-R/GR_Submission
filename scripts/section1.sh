#!/usr/bin/env bash
# Section 1: bring up MoveIt + scene + IK service, then run the reachability sweep.
# Runs everything inside the gr_humble container. Idempotent — re-running won't
# double-launch nodes that are already up.
#
# Usage:   bash scripts/section1.sh
# Outputs: data/reachability.csv, data/reachability.png, data/{demo,section1}.log

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f ${REPO_DIR}/docker/docker-compose.yml"
EXEC="${COMPOSE} exec -T humble bash -c"
EXEC_D="${COMPOSE} exec -T -d humble bash -c"

c_grn=$'\e[32m'; c_red=$'\e[31m'; c_dim=$'\e[2m'; c_off=$'\e[0m'
log()  { echo "${c_grn}[section1]${c_off} $*"; }
warn() { echo "${c_red}[section1]${c_off} $*" >&2; }

ensure_container() {
  if ! ${COMPOSE} ps --services --filter "status=running" | grep -q '^humble$'; then
    log "starting humble container"
    ${COMPOSE} up -d humble >/dev/null
    sleep 2
  fi
}

is_service_up() {
  # $1 = service name (e.g. /compute_ik). Returns 0 if visible to ros2.
  ${EXEC} "source /opt/ros/humble/setup.bash; ros2 service list 2>/dev/null | grep -q '^$1$'"
}

wait_for_service() {
  local svc="$1" max="${2:-30}" i=0
  while ! is_service_up "$svc"; do
    i=$((i+1))
    if [ "$i" -ge "$max" ]; then
      warn "timed out waiting for $svc (${max}*2s)"
      return 1
    fi
    sleep 2
    printf "."
  done
  echo " ok (${i}*2s)"
}

ensure_workspace_built() {
  ${EXEC} "test -f /home/dev/ros2_ws/install/gr_kinematics/share/gr_kinematics/config/reachability.yaml" \
    && return 0
  log "workspace not built — running colcon build (one-time)"
  ${EXEC} "source /opt/ros/humble/setup.bash && cd /home/dev/ros2_ws && colcon build --symlink-install --packages-skip piper" \
    >/dev/null
}

start_movegroup_if_needed() {
  if is_service_up /compute_ik; then
    log "move_group already running — skipping"
    return 0
  fi
  log "launching piper_no_gripper_moveit/demo.launch.py"
  ${EXEC_D} "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros2_ws/install/setup.bash
    ros2 launch piper_no_gripper_moveit demo.launch.py > /home/dev/data/demo.log 2>&1
  "
  printf "  waiting for /compute_ik"
  wait_for_service /compute_ik 30
}

start_section1_if_needed() {
  if is_service_up /gr_kinematics/solve_ik; then
    log "IK wrapper + scene already up — re-pushing scene objects"
    ${EXEC} "
      source /opt/ros/humble/setup.bash
      source /home/dev/ros2_ws/install/setup.bash
      ros2 run gr_scene scene_loader --ros-args --params-file \
        /home/dev/ros2_ws/install/gr_scene/share/gr_scene/config/scene.yaml \
        -r __node:=scene_loader
    " || true
    return 0
  fi
  log "launching gr_kinematics/section1.launch.py"
  ${EXEC_D} "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros2_ws/install/setup.bash
    ros2 launch gr_kinematics section1.launch.py > /home/dev/data/section1.log 2>&1
  "
  printf "  waiting for /gr_kinematics/solve_ik"
  wait_for_service /gr_kinematics/solve_ik 15
}

run_sweep() {
  log "running reachability sweep (961 cells, ~30 s)"
  ${EXEC} "
    source /opt/ros/humble/setup.bash
    source /home/dev/ros2_ws/install/setup.bash
    ros2 run gr_kinematics reachability \
      --ros-args --params-file /home/dev/ros2_ws/install/gr_kinematics/share/gr_kinematics/config/reachability.yaml \
      -r __node:=reachability_sweep
  " 2>&1 | grep -E "INFO|ERROR|Reachable|Wrote"
}

summarize() {
  local csv="${REPO_DIR}/data/reachability.csv"
  local png="${REPO_DIR}/data/reachability.png"
  [ -f "$csv" ] && [ -f "$png" ] || { warn "outputs missing — check data/section1.log"; return 1; }
  local total reach
  total=$(($(wc -l < "$csv") - 1))
  reach=$(awk -F, 'NR>1 && $3==1' "$csv" | wc -l)
  log "done — ${reach}/${total} cells reachable"
  log "outputs: ${c_dim}${csv}${c_off}"
  log "         ${c_dim}${png}${c_off}"
}

main() {
  ensure_container
  ensure_workspace_built
  start_movegroup_if_needed
  start_section1_if_needed
  run_sweep
  summarize
}

main "$@"
