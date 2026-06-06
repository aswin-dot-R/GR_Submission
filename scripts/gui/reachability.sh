#!/usr/bin/env bash
# Section 1 reachability sweep (needs Start MoveIt first). Streams live progress.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_moveit
echo "[reachability] running sweep (~1-2 min) — patch x_center=0.30 ..."
jz "${WS}; export RCUTILS_LOGGING_BUFFERED_STREAM=0; stdbuf -oL -eL ros2 run gr_kinematics reachability --ros-args --params-file ${INST}/gr_kinematics/share/gr_kinematics/config/reachability.yaml -r __node:=reachability_sweep 2>&1 | grep --line-buffered -E 'cells \(|Reachable|Wrote|Surface-aligned'"
echo "[reachability] DONE."
