#!/usr/bin/env bash
# Section 3: contact-aware wiping in Gazebo Classic.
#   - gazebo_wiping.launch.py : Gazebo + scene + Piper + wrist F/T + controllers
#   - wiping_controller       : admittance/force state machine (KDL diff-IK)
# All inside the gr_humble container.
#
# Usage:
#   bash scripts/section3.sh sim     # just the Gazebo bring-up (GUI if DISPLAY set)
#   bash scripts/section3.sh ctrl    # just the wiping controller (sim must be up)
#   bash scripts/section3.sh run     # sim (background) + controller (foreground)
# Outputs: data/wiping_log.csv, data/wiping_log.png

set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f ${REPO_DIR}/docker/docker-compose.yml"
EXEC="${COMPOSE} exec -T humble bash -c"
EXEC_D="${COMPOSE} exec -T -d humble bash -c"
WS="source /opt/ros/humble/setup.bash; source /home/dev/ros2_ws/install/setup.bash"
PARAMS="/home/dev/ros2_ws/install/gr_wiping_control/share/gr_wiping_control/config/wiping.yaml"

c_grn=$'\e[32m'; c_off=$'\e[0m'; log(){ echo "${c_grn}[section3]${c_off} $*"; }

ensure() {
  ${COMPOSE} ps --services --filter status=running | grep -q '^humble$' || ${COMPOSE} up -d humble >/dev/null
  ${EXEC} "test -f ${PARAMS}" || ${EXEC} "source /opt/ros/humble/setup.bash && cd /home/dev/ros2_ws && colcon build --symlink-install --packages-select gr_wiping_control" >/dev/null
}

wait_ready() {
  ${EXEC} "${WS}; for i in \$(seq 1 25); do ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active' && break; sleep 2; done; ros2 control list_controllers 2>/dev/null"
}

case "${1:-run}" in
  sim)
    ensure; log "launching Gazebo wiping scene"
    ${EXEC} "${WS}; ros2 launch gr_wiping_control gazebo_wiping.launch.py" ;;
  ctrl)
    ensure; log "launching wiping controller"
    ${EXEC} "${WS}; ros2 run gr_wiping_control wiping_controller --ros-args -p use_sim_time:=true --params-file ${PARAMS}" ;;
  run)
    ensure
    log "starting Gazebo (headless) -> data/section3_gz.log"
    ${EXEC_D} "${WS}; ros2 launch gr_wiping_control gazebo_wiping.launch.py gui:=false > /home/dev/data/section3_gz.log 2>&1"
    log "waiting for controllers..."; wait_ready
    log "running wiping controller (foreground)"
    ${EXEC} "${WS}; ros2 run gr_wiping_control wiping_controller --ros-args -p use_sim_time:=true --params-file ${PARAMS}"
    log "outputs: data/wiping_log.{csv,png}" ;;
  *) echo "usage: $0 {sim|ctrl|run}" ;;
esac
