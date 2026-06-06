#!/usr/bin/env bash
# Start MoveIt (move_group + RViz) + IK service + scene loader in the Jazzy container.
# RViz window appears in ~25-30 s; the 3 obstacles load a few seconds after.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"

echo "[start_moveit] cleaning up old instances..."
kill_all

echo "[start_moveit] launching move_group + RViz (window opens in ~25-30 s)..."
jz_d "export DISPLAY=:1; ${WS}; ros2 launch piper_no_gripper_moveit demo.launch.py > /home/dev/data/jazzy_moveit.log 2>&1"

echo "[start_moveit] waiting for /compute_ik..."
if wait_svc /compute_ik; then echo "[start_moveit]   move_group up."; else echo "[start_moveit]   ERROR: move_group did not start"; exit 1; fi

echo "[start_moveit] launching IK service + scene loader (the 3 obstacles)..."
jz_d "${WS}; ros2 launch gr_kinematics section1.launch.py > /home/dev/data/jazzy_section1.log 2>&1"
sleep 6
jz "grep -i 'Loaded.*collision' /home/dev/data/jazzy_section1.log | tail -1" 2>/dev/null || true
echo "[start_moveit] DONE. In RViz, enable 'Scene Geometry' if the obstacle boxes aren't visible."
