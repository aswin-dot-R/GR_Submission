# GR Manipulation Take-Home — Solution Overview

One-stop explanation of the **three sections**: what each asked for, the **setup**
to run it, what was **built**, and the **results**. For a from-scratch ROS
refresher and a line-by-line code walk-through see
[`ASSIGNMENT_EXPLAINED.md`](ASSIGNMENT_EXPLAINED.md); for exact clone-to-run steps
see [`REPRODUCE.md`](REPRODUCE.md); per-section design notes live next to the code
(`*_NOTES.md`).

---

## The robot & scene (shared by all three sections)

- **Robot:** AgileX **Piper**, a **6-DOF** arm. URDF/MoveIt config vendored in
  `ros2_ws/src/piper_ros/` (AgileX official).
- **Scene** (defined once in `gr_scene/config/scene.yaml`, metres, `base_link` frame).
  The faucet and mirror were repositioned **into the arm's reach**:
  - **Countertop** — 1.20 × 0.60 m horizontal slab, top surface at `z = 0`.
  - **Faucet** — 0.06 × 0.06 × 0.30 m post at **(0.40, 0.00)** — the obstacle.
  - **Mirror** — 0.02 × 0.90 × 0.60 m vertical pane at **(0.45, 0.00)** (now wipeable).
  - Color-coded in the diagrams (`data/world_topdown.png`, `world_side.png`,
    rendered live from `scene.yaml`): **countertop = tan, mirror = blue, faucet = red**.
- **Four packages** under `ros2_ws/src/gr_assignment/`:
  `gr_scene` (collision objects), `gr_kinematics` (Sec 1), `gr_coverage` (Sec 2),
  `gr_wiping_control` (Sec 3).

---

## Setup (how to run everything)

Everything runs in **Docker** (no native ROS install). Requires a **Linux host
with an NVIDIA GPU + nvidia-container-toolkit** (the compose file reserves a GPU;
Gazebo/RViz need GL).

```bash
# 1. one-time host setup (NVIDIA runtime + X11)            [sudo]
bash docker/setup_host.sh
# 2. build image + start the container (handles the xauth fix)
cd docker && docker compose build humble && cd ..
bash scripts/host_up.sh
# 3. build the ROS workspace inside the container
docker compose -f docker/docker-compose.yml exec humble bash -lc \
  'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && \
   colcon build --symlink-install --packages-skip piper'
# 4. run each section (outputs land in data/)
bash scripts/section1.sh        # Section 1
bash scripts/section2.sh        # Section 2
bash scripts/section3.sh run    # Section 3 (Gazebo)
```

Key environment facts: **Humble** container, **Gazebo Classic** for Section 3
(Isaac Sim was dropped — it segfaulted on the driver). The `data/` folder is
bind-mounted into the container, so all CSVs/PNGs/YAMLs the nodes write appear in
the repo. Sourcing both `setup.bash` files before running is required.

---

## Section 1 — Kinematics & Reachability

### Requirements
- Use **MoveIt 2** with the 6-DOF arm; build the planning scene with the three
  collision objects.
- Implement an **IK service** that solves **surface-aligned** end-effector poses
  and **rejects infeasible** ones.
- Generate a **reachability heatmap** of a **60 × 60 cm** counter patch at **2 cm**
  resolution.
- **Deliverables:** URDF/config + IK node · reachability CSV + heatmap · write-up
  "where can/can't the arm reach, and why."

### What was built
- `gr_scene/scene_loader.py` — injects the 3 boxes into MoveIt via
  `/apply_planning_scene`.
- `gr_kinematics/ik_service.py` — wraps MoveIt's `/compute_ik` as
  `/gr_kinematics/solve_ik` (collision-aware, configurable timeout/retries,
  returns a failure code for infeasible poses). Uses the standard
  `MultiThreadedExecutor` + `ReentrantCallbackGroup` pattern for a
  service-calling-a-service without deadlock.
- `gr_kinematics/reachability.py` — sweeps the 31 × 31 = **961-cell** grid, tool
  pointing **down** onto the counter.
- **Key insight:** "surface-aligned" only fixes the tool's *approach axis*; the
  **yaw** of the flat pad about that axis is a **free DOF**. Pinning the full
  orientation gave a misleading **34/961 = 3.5 %**; searching yaw (12 angles,
  reachable if *any* works) gives the true number.

### Results
- **464 / 961 = 48.3 % reachable** (with the repositioned scene; was 53.9 % before
  the faucet+mirror were moved into the patch). Outputs: `data/reachability.csv`
  (`x, y, reachable, error_code, feasible_yaw_deg`) and `data/reachability.png`
  (heatmap: green = reachable). The drop is expected — the faucet (0.40, 0) and
  mirror pane (0.45, 0) now sit inside the patch and carve out keep-outs.
- Write-up: [`gr_kinematics/REACHABILITY_NOTES.md`](ros2_ws/src/gr_assignment/gr_kinematics/REACHABILITY_NOTES.md).

**Run:** `bash scripts/section1.sh`

---

## Section 2 — Surface Coverage Path Planning

### Requirements
- ≥ 1 **coverage strategy** (raster); **bonus: spiral** for the mirror.
- Convert Cartesian waypoints → **joint trajectories** with **time
  parameterization**.
- Report **coverage %, path length, estimated execution time**.
- Constraints: tool pad **100 × 50 mm**, tool normal within **±10°** of surface
  normal, **15 mm** keep-out, **10–20 %** overlap.
- **Deliverables:** planner node + visualization · example joint trajectory ·
  raster-vs-spiral comparison.

### What was built
- `gr_coverage/planners.py` (pure geometry, no ROS): `raster_path` (boustrophedon,
  row pitch = `tool_height·(1−overlap)`, edges inset by `margin + half-tool`),
  `spiral_path` (Archimedean), `path_length`, `coverage_fraction`.
- `gr_coverage/coverage_node.py`: orients the tool into the surface (±10° met),
  **searches yaw** for max reachable waypoints, finds the **longest contiguous
  reachable run**, calls MoveIt `/compute_cartesian_path`, and **retimes** the
  (un-timed) Cartesian path into a velocity-limited joint trajectory.
- **Honesty:** reports **geometric** coverage (path over the surface) *and*
  **executable** coverage (only waypoints the arm can actually reach).

### Results (repositioned scene)
- **Countertop (raster):** geometric 98.7 %, **executable 44.2 %**
  (267/788 waypoints reachable), path 15.5 m, a real **2.5 s** time-parameterized
  stroke (`cartesian_fraction = 100 %`). Lower than before — the faucet+mirror now
  obstruct more of the counter.
- **Mirror (spiral):** **now reachable! 37.5 % executable** (147/212 = 69 % of
  waypoints), a real **7.2 s** trajectory. Previously 0 % when the mirror sat at
  `x = 0.95 m`; moving it to `x = 0.45 m` brought it within the Piper's reach.
- Outputs: `data/coverage_path.png` (reachable=green / unreachable=red / executed
  stroke=navy), `coverage_path_{counter,mirror}.csv`,
  `coverage_trajectory_{counter,mirror}.yaml`.
- Comparison + notes: [`gr_coverage/COVERAGE_NOTES.md`](ros2_ws/src/gr_assignment/gr_coverage/COVERAGE_NOTES.md).

**Run:** `bash scripts/section2.sh`

---

## Section 3 — Contact-Aware Wiping Control

### Requirements
- Simulated **wrist F/T sensor**. Targets — counter **10 N ±2 N @ 0.15–0.25 m/s**,
  mirror **6 N ±1.5 N @ 0.1–0.2 m/s**.
- **Switch to force control** when `|Fz| > 2 N`.
- **Maintain target force** while following the path; **back off** if `|Fz| > 15 N`.
- **Handle the faucet** by skipping/replanning locally.
- **Log and plot** force-vs-time and velocity-vs-time.
- **Deliverables:** controller code + configs · force/velocity plots · short demo
  (video/gif or sim run).

### Setup (Gazebo)
Runs in **Gazebo Classic**. Two simulator facts drove the design:
1. Gazebo Classic's `gazebo_ros2_control` **can't expose an F/T sensor** as a
   control interface → the stock **`gazebo_ros_ft_sensor` plugin** on **joint6**
   publishes `/wrist_ft`.
2. Its **position** command interface is **kinematic** (the arm passes *through*
   the counter) → the arm is driven via the **effort** interface + per-joint PID
   so contact is physical.

Bring-up files in `gr_wiping_control/`: `description/piper_wiping.xacro` (pad +
F/T sensor + compliant contact), `worlds/wiping.world`,
`launch/gazebo_wiping.launch.py`, `config/wiping_controllers.yaml`,
`config/wiping.yaml`.

### What was built
- `gr_wiping_control/kdl_chain.py` — KDL FK / Jacobian / IK from the live URDF.
- `gr_wiping_control/controller.py` — a 50 Hz **state machine**:
  **APPROACH** (move above start, tare the sensor) → **DESCEND** (until `|Fn|>2 N`)
  → **CONTACT** (PI **admittance** law `depth += Kp·(F*−F) + Ki·∫err` regulating to
  the target) → **BACKOFF** (`|Fn|>15 N`) → **SKIP** (lift over the faucet).
  Commands stream via **differential (Jacobian) IK** (`q += clamp(J⁺·Δx)`) for
  continuity, contact height uses **known geometry**, force is read in world frame
  (tare-corrected).

### Results & a scene-geometry limitation
The controller itself is functional — in earlier runs (wipe stroke clear of
obstacles) it showed contact detection at 2 N, force regulation around the 10 N
target, the 15 N back-off, and the faucet skip, with the documented **contact
chatter** of a stiff effort-controlled arm on a near-rigid surface.

**With the repositioned scene, Section 3's wipe does not reliably engage
contact.** The tool descends to the counter at the stroke start but the pad
passes through without ODE forming a contact (max force ≈ 0.5 N, the run stays in
DESCEND/CONTACT with no force). This was investigated and is **not** the obstacle
positions: it reproduces at stroke `x = 0.30` and `x = 0.40`, with the mirror at
both `0.45` and `0.52` — all fail to generate a pad↔counter contact. The
controller demonstrated the correct behaviour (contact at 2 N, force regulation
around 10 N, 15 N back-off, faucet skip) in an earlier run, but the rigid,
effort-controlled contact in Gazebo Classic is **fragile and not reproducing**
after the scene changes. Robustly fixing it needs a more stable contact
formulation (softer joint gains / a true force inner loop) or the off-the-shelf
`admittance_controller`. Detailed in
[`gr_wiping_control/WIPING_NOTES.md`](ros2_ws/src/gr_assignment/gr_wiping_control/WIPING_NOTES.md).

**Run:** `bash scripts/section3.sh run`

---

## Deliverables checklist

| Section | Deliverable | Status | Where |
|---|---|---|---|
| 1 | IK node + config | ✅ | `gr_kinematics/`, `gr_scene/` |
| 1 | Reachability CSV + heatmap | ✅ | `data/reachability.{csv,png}` |
| 1 | Write-up (where/why) | ✅ | `REACHABILITY_NOTES.md` |
| 2 | Planner node + visualization | ✅ | `gr_coverage/`, `data/coverage_path.png` |
| 2 | Example joint trajectory | ✅ | `data/coverage_trajectory_*.yaml` |
| 2 | Raster-vs-spiral comparison | ✅ | `COVERAGE_NOTES.md` |
| 3 | Controller code + configs | ✅ | `gr_wiping_control/` |
| 3 | Force/velocity plots | ✅ (no contact in current scene — see limitation) | `data/wiping_log.png` |
| 3 | Short demo (video/gif or sim run) | ⚠️ sim run only; wipe can't engage at current obstacle spacing | `scripts/section3.sh run` |

## Repo map
```
docker/        Docker images, compose, host setup
ros2_ws/src/
  piper_ros/                 AgileX Piper (URDF, MoveIt, Gazebo) — vendored
  gr_assignment/
    gr_scene/                Sec 1 — collision objects
    gr_kinematics/           Sec 1 — IK service + reachability  (+ REACHABILITY_NOTES.md)
    gr_coverage/             Sec 2 — raster/spiral coverage      (+ COVERAGE_NOTES.md)
    gr_wiping_control/        Sec 3 — Gazebo contact wiping       (+ WIPING_NOTES.md)
scripts/       section{1,2,3}.sh, host_up.sh, setup_docker_creds.sh
data/          committed outputs (reachability / coverage / wiping)
ASSIGNMENT_EXPLAINED.md   from-scratch ROS refresher + per-file code walk-through
REPRODUCE.md              fresh-clone run steps + requirements + limitations
SOLUTION.md               (this file) requirements + setup + results overview
```
