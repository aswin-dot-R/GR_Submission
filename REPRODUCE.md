# Reproducing this assignment from a fresh clone

Everything runs in Docker — no native ROS install needed.

## Requirements (hard)
- **Linux host** with **Docker + docker compose**.
- **NVIDIA GPU + `nvidia-container-toolkit`.** `docker/docker-compose.yml`
  reserves an NVIDIA device, and Gazebo/RViz need GL. Without an NVIDIA GPU the
  container won't start as-is (you'd have to remove the `deploy.resources` block
  and accept software GL / no GUI).
- ~15 GB free disk for the images.
- An X server on `:1` if you want the Gazebo/RViz GUIs (headless also works).

The Docker images pin every dependency (MoveIt 2, ros2_control, gazebo_ros_pkgs,
gazebo_ros2_control, PyKDL, urdf_parser_py, xacro, scipy, …). `piper_ros` (AgileX) is
vendored in-tree.

## Containers
- **`gr_jazzy`** (ROS 2 Jazzy): MoveIt 2 + the IK / coverage / wiping nodes (Sections 1, 2,
  and the Section 3 primary).
- **`gr_humble`** (ROS 2 Humble): the Gazebo **Classic** sim (`gazebo_ros2_control`) for the
  Section 3 live-contact demo.

## Steps
```bash
git clone <repo> && cd GR_Assignment
bash docker/setup_host.sh                       # one-time: NVIDIA runtime + X11 (sudo)
docker compose -f docker/docker-compose.yml up -d jazzy humble

# build the workspace in each container (jazzy = install_jazzy, humble = install)
docker compose -f docker/docker-compose.yml exec jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /home/dev/ros2_ws && \
   colcon build --symlink-install --build-base build_jazzy --install-base install_jazzy'
docker compose -f docker/docker-compose.yml exec humble bash -lc \
  'source /opt/ros/humble/setup.bash && cd /home/dev/ros2_ws && colcon build --symlink-install'
```

### Run each section (writes to `data/`)
```bash
# --- Section 1: reachability + manipulability heatmap ---
bash scripts/gui/start_moveit.sh                # move_group + IK service + scene
bash scripts/gui/reachability.sh                # -> data/reachability.{png,csv}

# --- Section 2: raster + spiral coverage, usable-area metrics ---
docker compose -f docker/docker-compose.yml exec jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /home/dev/ros2_ws/install_jazzy/setup.bash && \
   ros2 launch gr_coverage section2.launch.py'   # -> data/coverage_path.{png,csv}, trajectories

# --- Section 3 (PRIMARY): MoveIt + software spring-damper F/T, force/velocity/dexterity plots ---
bash scripts/gui/s3_primary.sh                 # counter + mirror -> data/wiping_log.png
#   active_surface:=mirror for the mirror run  -> data/wiping_log_mirror.png

# --- Section 3 (SECONDARY, live demo, Gazebo Classic): software admittance loop on the
#     real /wrist_ft sensor, full counter + mirror coverage with the paint trail ---
bash scripts/gui/start_classic.sh              # Classic sim (gr_humble) on DISPLAY=:1
bash scripts/gui/s3_secondary.sh               # -> data/admittance_log.{png,csv}
```

## Expected outputs (also committed in `outputs/` for viewing without running)
| Section | Files |
|---|---|
| 1 | `reachability.csv` (961 cells, **70.8 % reachable / ~22 % usable**), `reachability.png` |
| 2 | `coverage_path.png` (red/green reachability + executed stroke), `coverage_path_{counter,mirror}.csv`, `coverage_trajectory_*.yaml` |
| 3 | `wiping_log{,_mirror}.png` (force / velocity / dexterity), `live_admittance_classic.png` (live Gazebo Classic demo) |

Per-section design notes: `gr_kinematics/REACHABILITY_NOTES.md`,
`gr_coverage/COVERAGE_NOTES.md`, `gr_wiping_control/WIPING_NOTES.md`,
`SECTION3_ATTEMPTS.md`.

## Known limitations (documented in the notes)
- **Reachable ≠ usable:** ~71 % of the counter patch is reachable but only ~22 % is
  well-conditioned (non-singular). Coverage and wiping are scored on the usable region.
- **Section 3 physics force-hold:** clean 10/6 N force-holding in a physics sim with a
  position-controlled arm is **not cleanly achievable** — the software spring-damper model
  (primary) hits spec; the live Gazebo Classic admittance loop is an honest motion/contact
  demo (holds ~8 N, ~96 % in the contact band). **2 of the 3 reference implementations use a
  software contact model too** (arm_takehome-main: RViz; piper_ws: a 650 N/m spring) **and
  hit spec**; only griffin_ws attempted real Gazebo physics and came in below spec (~1–2 N).
  So the software model is the consensus, spec-meeting delivery for a *simulated* F/T sensor
  (other approaches that were tried but not used are recorded in `SECTION3_ATTEMPTS.md`).
