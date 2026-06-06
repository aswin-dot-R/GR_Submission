#!/usr/bin/env bash
# shared helpers for the demo GUI button scripts
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE="docker compose -f ${REPO}/docker/docker-compose.yml"
WS="source /opt/ros/jazzy/setup.bash; source /home/dev/ros2_ws/install_jazzy/setup.bash"
INST="/home/dev/ros2_ws/install_jazzy"

jz()   { ${COMPOSE} exec -T   jazzy bash -c "$1"; }   # run in jazzy (wait)
jz_d() { ${COMPOSE} exec -T -d jazzy bash -c "$1"; }  # run in jazzy (detached)

# humble container = Gazebo Classic sim for the Section-3 live force demo
HWS="source /opt/ros/humble/setup.bash; source /home/dev/ros2_ws/install/setup.bash"
hb()   { ${COMPOSE} exec -T   humble bash -c "$1"; }   # run in humble (wait)
hb_d() { ${COMPOSE} exec -T -d humble bash -c "$1"; }  # run in humble (detached)

# Ensure the Classic sim (humble) is up for the live force demo. Brings it up if not.
ensure_classic() {
  if hb "source /opt/ros/humble/setup.bash; ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active'" >/dev/null 2>&1; then
    return 0
  fi
  echo "[ensure] Classic Gazebo sim is down — starting it first (~40 s) ..."
  bash "$(dirname "${BASH_SOURCE[0]}")/start_classic.sh"
}

kill_all() {
  jz "pkill -9 -f 'gz sim|move_group|rviz2|ros2 launch|robot_state_publisher|wiping_controller|ruby|ros2_control_node|reachability_sweep|coverage_planner|parameter_bridge|traj_player|ik_service|scene_loader|wiping_moveit' 2>/dev/null; sleep 2" >/dev/null 2>&1 || true
}
wait_svc() { # $1 = service name, waits up to ~60s
  jz "${WS}; for i in \$(seq 1 30); do ros2 service list 2>/dev/null | grep -q '^$1\$' && exit 0; sleep 2; done; exit 1"
}

# Ensure move_group is up (it can die when the session sits idle). If the Cartesian
# service is missing, auto-run Start MoveIt so the wipe buttons are self-healing.
ensure_moveit() {
  # Check the NODE list, not the service list: when move_group dies its services
  # linger in the DDS graph (stale), but the /move_group node disappears.
  if jz "${WS}; ros2 node list 2>/dev/null | grep -q '^/move_group\$'" >/dev/null 2>&1; then
    return 0
  fi
  echo "[ensure] move_group is down — starting MoveIt first (~30 s) ..."
  bash "$(dirname "${BASH_SOURCE[0]}")/start_moveit.sh"
}

ensure_gazebo() {
  if jz "${WS}; ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active'" >/dev/null 2>&1; then
    return 0
  fi
  echo "[ensure] Gazebo arm_controller is down - starting Gazebo first (~20 s) ..."
  bash "$(dirname "${BASH_SOURCE[0]}")/start_gazebo.sh"
}
