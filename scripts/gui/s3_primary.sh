#!/usr/bin/env bash
# Section 3 — PRIMARY method: MoveIt + software spring-damper F/T (meets spec).
# (1) Runs the contact-aware wipe on BOTH surfaces (MoveIt, no Gazebo) and writes the
#     force/velocity/dexterity plots — the spec deliverable.
# (2) Then visualizes the planned full-coverage path in the Gazebo Classic sim WITH the
#     coverage paint-trail (green=counter / blue=mirror tiles) so you can see what it wiped.
#
#   counter -> data/wiping_log.png         (10 ± 2 N, 0.15-0.25 m/s)
#   mirror  -> data/wiping_log_mirror.png  (6 ± 1.5 N, 0.10-0.20 m/s)
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_moveit

echo "[s3-primary] === COUNTERTOP wipe (10 N) ==="
jz "${WS}; stdbuf -oL ros2 run gr_wiping_control wiping_moveit --ros-args -p active_surface:=counter \
  -p log.csv_path:=/home/dev/data/wiping_log.csv -p log.png_path:=/home/dev/data/wiping_log.png \
  -p trajectory_path:=/home/dev/data/wiping_trajectory.yaml 2>&1 | grep --line-buffered -E 'force-hold|manipulability|wrote'"

echo "[s3-primary] === MIRROR wipe (6 N) ==="
jz "${WS}; stdbuf -oL ros2 run gr_wiping_control wiping_moveit --ros-args -p active_surface:=mirror \
  -p log.csv_path:=/home/dev/data/wiping_log_mirror.csv -p log.png_path:=/home/dev/data/wiping_log_mirror.png \
  -p trajectory_path:=/home/dev/data/wiping_trajectory_mirror.yaml 2>&1 | grep --line-buffered -E 'force-hold|manipulability|wrote'"

echo "[s3-primary] plots written: data/wiping_log.png + data/wiping_log_mirror.png"

# ---- visualize the planned coverage in Gazebo with the paint trail ----
SCRIPTS=/home/dev/ros2_ws/src/gr_assignment/gr_wiping_control/scripts
ensure_classic
echo "[s3-primary] visualizing the planned coverage in Gazebo with the paint-trail (watch :1)..."
hb "${HWS}; pkill -9 -f 'paint_trail' 2>/dev/null; sleep 1"
hb_d "${HWS}; python3 ${SCRIPTS}/paint_trail.py > /home/dev/data/paint_trail.log 2>&1"
sleep 3
hb "${HWS}; python3 ${SCRIPTS}/replay_wipe.py 1"
echo "[s3-primary] DONE — plots + coverage trail on :1"
