#!/usr/bin/env bash
# Section 3 (method 3): start Gazebo HARMONIC (GUI) + controllers in Jazzy.
# Use `controller:=admittance_controller` for the proper ros2_control AdmittanceController.
# For the main live force demo use start_classic.sh (Gazebo Classic) + s3_secondary.sh.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
echo "[gazebo] cleaning up..."; kill_all
echo "[gazebo] launching Gazebo Harmonic GUI + controllers (window in ~20 s)..."
jz_d "export DISPLAY=:1; ${WS}; ros2 launch gr_wiping_control gz_wiping.launch.py gui:=true controller:=arm_controller > /home/dev/data/jazzy_gz.log 2>&1"
echo "[gazebo] waiting for controllers..."
jz "${WS}; for i in \$(seq 1 30); do ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active' && break; sleep 2; done; ros2 control list_controllers 2>/dev/null"
echo "[gazebo] DONE — Gazebo window should be open."
