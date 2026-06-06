#!/usr/bin/env bash
# Run ALL three sections in the Jazzy / Gazebo Harmonic container, then collect
# the deliverables into outputs/. One command:  bash scripts/run_all_jazzy.sh
#
# Sections 1 & 2 use MoveIt (move_group); Section 3 uses Gazebo Harmonic. They
# can't share the container at once, so they run sequentially with a clean
# teardown between.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f ${REPO}/docker/docker-compose.yml"
EXEC="${COMPOSE} exec -T jazzy bash -lc"
EXEC_D="${COMPOSE} exec -T -d jazzy bash -lc"
WS="source /opt/ros/jazzy/setup.bash; source /home/dev/ros2_ws/install_jazzy/setup.bash"
INST="/home/dev/ros2_ws/install_jazzy"

grn=$'\e[32m'; off=$'\e[0m'
log(){ echo "${grn}[run-all]${off} $*"; }

cleanup(){
  ${EXEC} "pkill -9 -f 'gz sim|move_group|ros2 launch|robot_state_publisher|wiping_controller|ruby|ros2_control_node|reachability_sweep|coverage_planner|parameter_bridge|traj_player' 2>/dev/null; sleep 2" >/dev/null 2>&1 || true
}
wait_svc(){ ${EXEC} "${WS}; for i in \$(seq 1 30); do ros2 service list 2>/dev/null | grep -q '^$1$' && exit 0; sleep 2; done; exit 1"; }

# ---------- setup ----------
log "ensuring Jazzy container + workspace build"
${COMPOSE} up -d jazzy >/dev/null
${EXEC} "test -d ${INST}/gr_kinematics" >/dev/null 2>&1 || \
  ${EXEC} "${WS%%;*}; cd /home/dev/ros2_ws; colcon build --symlink-install --build-base build_jazzy --install-base install_jazzy --packages-skip piper" >/dev/null

# ---------- Sections 1 & 2 (MoveIt) ----------
cleanup
log "launching move_group (MoveIt) ..."
${EXEC_D} "${WS}; ros2 launch piper_no_gripper_moveit demo.launch.py > /home/dev/data/jazzy_moveit.log 2>&1"
wait_svc /compute_ik && log "  /compute_ik up" || { log "move_group failed"; exit 1; }
${EXEC_D} "${WS}; ros2 launch gr_kinematics section1.launch.py > /home/dev/data/jazzy_section1.log 2>&1"
wait_svc /gr_kinematics/solve_ik && log "  IK service + scene up"

log "SECTION 1 — reachability sweep (~1-2 min) ..."
${EXEC} "${WS}; ros2 run gr_kinematics reachability --ros-args --params-file ${INST}/gr_kinematics/share/gr_kinematics/config/reachability.yaml -r __node:=reachability_sweep 2>&1 | grep -E 'Reachable|Wrote'"

log "SECTION 2 — coverage planner (raster + spiral) ..."
${EXEC} "${WS}; ros2 run gr_coverage coverage_planner --ros-args --params-file ${INST}/gr_coverage/share/gr_coverage/config/coverage.yaml -r __node:=coverage_planner 2>&1 | grep -E 'coverage=|Wrote'"

# ---------- Section 3 (MoveIt + software spring-damper F/T) ----------
# Primary approach: runs in the SAME move_group session as Sections 1 & 2 (the
# simulated wrist F/T sensor is an analytical spring-damper; IK via MoveIt). No
# Gazebo needed. The Gazebo physics demo is secondary (scripts/gui/wipe.sh).
WCFG="${INST}/gr_wiping_control/share/gr_wiping_control/config/wiping.yaml"
log "SECTION 3 — contact-aware wiping, counter (MoveIt + spring-damper, 10 N) ..."
${EXEC} "${WS}; ros2 run gr_wiping_control wiping_moveit --ros-args --params-file ${WCFG} -p log.csv_path:=/home/dev/data/wiping_log.csv -p log.png_path:=/home/dev/data/wiping_log.png -p trajectory_path:=/home/dev/data/wiping_trajectory.yaml 2>&1 | grep -E 'force-hold' || true"
log "SECTION 3 — contact-aware wiping, mirror (6 N) ..."
${EXEC} "${WS}; ros2 run gr_wiping_control wiping_moveit --ros-args --params-file ${WCFG} -p active_surface:=mirror -p log.csv_path:=/home/dev/data/wiping_log_mirror.csv -p log.png_path:=/home/dev/data/wiping_log_mirror.png -p trajectory_path:=/home/dev/data/wiping_trajectory_mirror.yaml 2>&1 | grep -E 'force-hold' || true"
cleanup

# ---------- collect outputs ----------
log "collecting deliverables into outputs/"
mkdir -p outputs/section1_reachability outputs/section2_coverage outputs/section3_wiping
cp -f "${REPO}"/data/reachability.csv "${REPO}"/data/reachability.png "${REPO}"/data/world_*.png outputs/section1_reachability/ 2>/dev/null || true
cp -f "${REPO}"/data/coverage_path*.csv "${REPO}"/data/coverage_path.png "${REPO}"/data/coverage_trajectory_*.yaml outputs/section2_coverage/ 2>/dev/null || true
cp -f "${REPO}"/data/wiping_log.csv "${REPO}"/data/wiping_log.png "${REPO}"/data/wiping_log_mirror.csv "${REPO}"/data/wiping_log_mirror.png "${REPO}"/data/wiping_trajectory*.yaml outputs/section3_wiping/ 2>/dev/null || true

echo
log "DONE. Outputs:"
find outputs -type f | sort | sed 's/^/    /'
log "(see outputs/README.md for the per-section deliverable map)"
