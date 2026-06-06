#!/usr/bin/env bash
# Section 2 coverage planner (needs Start MoveIt first).
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_moveit
echo "[coverage] running raster + spiral coverage ..."
jz "${WS}; export RCUTILS_LOGGING_BUFFERED_STREAM=0; stdbuf -oL -eL ros2 run gr_coverage coverage_planner --ros-args --params-file ${INST}/gr_coverage/share/gr_coverage/config/coverage.yaml -r __node:=coverage_planner 2>&1 | grep --line-buffered -E 'coverage=|geom_coverage|Wrote'"
echo "[coverage] DONE."
