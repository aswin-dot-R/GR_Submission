#!/usr/bin/env bash
# Stop all sim/launch processes in Jazzy.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_lib.sh"
echo "[stop] killing all sim/launch processes..."; kill_all; echo "[stop] done."
