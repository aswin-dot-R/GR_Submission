#!/usr/bin/env bash
# Section 3 — SECONDARY method: LIVE force control in Gazebo Classic, WITH coverage trail.
# A software admittance loop runs the full-coverage wipe and regulates penetration from
# the REAL /wrist_ft sensor (real physical contact). A paint-trail node spawns green
# (counter) / blue (mirror) tiles where the pad touches, so the swept region builds up
# on :1. Auto-starts the Classic sim. The force/velocity plot is written on completion.
#
#   ./s3_secondary.sh        -> one live force-regulated wipe + trail -> data/admittance_log.png
#   ./s3_secondary.sh loop   -> continuous looping wipe + trail (motion demo, watch :1)
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
ensure_classic

SCRIPTS=/home/dev/ros2_ws/src/gr_assignment/gr_wiping_control/scripts
MODE="${1:-force}"

echo "[s3-secondary] starting coverage paint-trail (clears old tiles)..."
hb "${HWS}; pkill -9 -f 'paint_trail' 2>/dev/null; sleep 1"
hb_d "${HWS}; python3 ${SCRIPTS}/paint_trail.py > /home/dev/data/paint_trail.log 2>&1"
sleep 3

if [ "$MODE" = "loop" ]; then
  echo "[s3-secondary] looping the full-coverage wipe (watch the trail build on :1, Ctrl-C to stop)..."
  hb "${HWS}; python3 ${SCRIPTS}/replay_wipe.py 0"
else
  echo "[s3-secondary] LIVE admittance force wipe on real /wrist_ft (watch :1, ~50 s)..."
  hb "${HWS}; python3 ${SCRIPTS}/admittance_wipe.py 2>&1 | grep --line-buffered -E 'live admittance|done|wrote'"
  echo "[s3-secondary] DONE — force/velocity plot: data/admittance_log.png ; trail on :1"
fi
