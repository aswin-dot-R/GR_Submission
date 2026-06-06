#!/usr/bin/env bash
# Section 3 (SECONDARY): start the Gazebo CLASSIC sim (gr_humble container) for the
# live force-control demo. GUI opens on DISPLAY=:1 (nvidia GL). Has the real wrist
# F/T sensor + soft contact, so the live admittance loop has real physics to regulate.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"

echo "[classic] cleaning up old sims (Classic + Harmonic share the ROS domain)..."
hb "pkill -9 -f 'gzserver|gzclient|gazebo|robot_state_publisher|spawn_entity|ros2 launch gr_wiping|admittance_wipe|loopwipe' 2>/dev/null; sleep 2" >/dev/null 2>&1 || true
jz "pkill -9 -f 'gz sim|ruby.*gz|ros_gz|create|parameter_bridge' 2>/dev/null; sleep 1" >/dev/null 2>&1 || true

echo "[classic] launching Gazebo Classic GUI + controllers (window in ~30 s)..."
hb_d "export DISPLAY=:1; export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia; ${HWS}; ros2 launch gr_wiping_control gazebo_wiping.launch.py gui:=true > /home/dev/data/classic_sim.log 2>&1"

echo "[classic] waiting for controllers..."
hb "${HWS}; for i in \$(seq 1 30); do ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active' && break; sleep 2; done; ros2 control list_controllers 2>/dev/null"
echo "[classic] DONE — Gazebo Classic window should be open on :1."
