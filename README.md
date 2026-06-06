# GR Manipulation Take-Home — Submission

AgileX Piper 6-DOF arm performing a bathroom/kitchen **wiping** task (countertop + mirror,
with a faucet obstacle), in ROS 2. Covers all three sections. Everything runs in Docker —
see [`REPRODUCE.md`](REPRODUCE.md) for the exact build/run commands.

| | |
|---|---|
| **Arm** | AgileX Piper (6-DOF), reach ≈ 0.5 m |
| **Stack** | ROS 2 Jazzy · MoveIt 2 + Pilz Industrial Motion Planner · Gazebo Classic + Harmonic · ros2_control |
| **Our packages** | `gr_scene` (scene) · `gr_kinematics` (IK + reachability) · `gr_coverage` (coverage) · `gr_wiping_control` (wiping) — all in `ros2_ws/src/gr_assignment/` |
| **Deliverables** | [`outputs/`](outputs/) — plots, CSVs, trajectories, per-section write-ups |

---

## The scene (how to replicate)

A wiping station defined entirely in `gr_scene/config/scene.yaml` (loaded into MoveIt's
planning scene) and mirrored in the Gazebo worlds
(`gr_wiping_control/worlds/wiping.world` = Classic, `wiping_gz.sdf` = Harmonic):

- **Robot** mounted **on the countertop** — the base sits flush on the slab, so the
  **countertop top is at z = 0** (the robot base level).
- **Countertop**: 120 × 60 × 4 cm slab, top at z = 0.
- **Faucet** (obstacle): 6 × 6 × 15 cm post standing on the slab at (0.40, 0), centred
  under the mirror.
- **Mirror**: 2 × 90 × 60 cm pane standing on the slab, face at x = 0.45.

All placements are parameters in `scene.yaml` / the world files, so the scene is fully
reproducible and re-configurable.

A through-line across all three sections: **reachable ≠ usable.** Every section scores not
just whether IK succeeds, but the **Yoshikawa manipulability** `w = √det(J Jᵀ)` (→ 0 at a
singularity), separating "the arm can touch this" from "the arm can actually *work* here."

---

## Section 1 — Kinematics & Reachability  → [`outputs/section1_reachability/`](outputs/section1_reachability/)

**What we did:** an **IK service** (`gr_kinematics`, `/gr_kinematics/solve_ik`) that solves
surface-aligned end-effector poses and rejects infeasible ones, plus a **reachability
heatmap** over a 60 × 60 cm counter patch at 2 cm resolution (31 × 31 = 961 cells).

Key methods:
- **"Surface-aligned" frees the yaw DOF.** Pinning the full tool orientation reports a
  misleading ~3.5 % reachable; fixing only the approach axis (tool normal) and searching the
  pad's free yaw gives the true workspace.
- **Manipulability/singularity analysis.** Each reachable cell is scored by `w`. ~71 %
  reachable but only **~22 % usable** (well-conditioned) — most of the slab is reachable only
  in near-singular poses. The heatmap shows the grey hole at the base, the near-singular
  crescent, and the dexterous sweet spot (x ≈ 0.35–0.40).

**Deliverables:** `reachability.png` (heatmap), `reachability.csv`
(`x, y, reachable, error_code, feasible_yaw_deg, manipulability`), `writeup.md`.

---

## Section 2 — Surface Coverage Path Planning  → [`outputs/section2_coverage/`](outputs/section2_coverage/)

**What we did:** a coverage planner (`gr_coverage`) generating a **raster** (boustrophedon)
on the countertop and a **spiral** (Archimedean, the bonus) on the mirror, honouring the
100 × 50 mm pad footprint, 15 mm keep-out, ±10° tool-normal, and 10–20 % overlap. Cartesian
waypoints → **time-parameterized joint trajectories**. Metrics per surface: coverage %,
**path length, execution time**, and the **usable-area** fraction (manipulability-filtered).

Key finding: the countertop is **98.7 % geometrically coverable but only ~10 % usable**
(most of the slab is near-base / near-singular); the mirror sits in the dexterous mid-reach
and is the better-conditioned surface. Raster vs spiral comparison in `writeup.md`.

**Deliverables:** `coverage_path.png` (manipulability shading + usable contour + executed
stroke), `coverage_path_{counter,mirror}.csv` (waypoints),
`coverage_trajectory_{counter,mirror}.yaml` (timed joint trajectories), `writeup.md`.

---

## Section 3 — Contact-Aware Wiping with a **Simulated F/T Sensor**  → [`outputs/section3_wiping/`](outputs/section3_wiping/)

The PDF asks for a **simulated** wrist F/T sensor + a controller that **switches to force
control at |Fz| > 2 N**, **maintains the target** (counter 10 ± 2 N @ 0.15–0.25 m/s, mirror
6 ± 1.5 N @ 0.10–0.20 m/s), **backs off at |Fz| > 15 N**, **handles the faucet**, and
**plots force & velocity vs time**.

We delivered this **three ways**, with the software model as the spec-compliant primary.

### (1) PRIMARY — MoveIt + a software spring-damper F/T model  *(meets spec)*
The **simulated F/T sensor** is an analytical one-sided spring-damper contact model:

```
F = max(0,  K·δ  +  D·δ̇  +  noise)
```

where `δ` is the tool's penetration below the surface (the controlled variable), `K`/`D` are
the contact stiffness/damping, and Gaussian noise gives a realistic sensor look. A software
**admittance loop** reads this `F` and drives the commanded penetration to hold the target;
**MoveIt** (`compute_ik` / `compute_cartesian_path`) turns the force-regulated Cartesian
poses into a time-parameterized joint trajectory (same machinery as Sections 1 & 2 — no
physics engine needed, exactly as "simulated" implies). The full **Pilz Industrial Motion
Planner** sequence capability chains the coverage path into one continuous trajectory.

State machine: `APPROACH` (F=0) → `CONTACT` (|F|>2 N, admittance holds target) → `BACKOFF`
(|F|>15 N, retract; a bump disturbance triggers it) → `SKIP` (lift over the faucet).

| Surface | Target | Force-hold | In-tol | Speed |
|---|---|---|---|---|
| Counter | 10 ± 2 N, 0.15–0.25 m/s | **9.9 ± 0.9 N** | **96 %** | 0.20 m/s ✓ |
| Mirror  | 6 ± 1.5 N, 0.10–0.20 m/s | **5.9 ± 0.8 N** | **93 %** | 0.15 m/s ✓ |

Plots (`wiping_log.png`, `wiping_log_mirror.png`) show force-vs-time, velocity-vs-time, and a
third **manipulability-vs-time** panel. Code: `gr_wiping_control/moveit_wiping.py`.

### (2) LIVE — software admittance loop on a REAL Gazebo F/T sensor (Classic)
`scripts/admittance_wipe.py` runs the coverage path on the **Gazebo Classic** physics sim and
regulates penetration from the **real `/wrist_ft`** sensor (a `gazebo_ros_ft_sensor`). We
verified genuine **physical** contact (force scales linearly with penetration, 0→4→10 N).
Two non-obvious fixes were needed: `<disableFixedJointLumping>` so the F/T sensor reads the
pad's contact, and a **Jacobian** press on the nominal joint config (full online KDL IK was
unreliable). Force regulated **~8 N**, the 2 N/15 N state machine, **velocity paced into the
spec band** (~67 % in-band). Plot: `live_admittance_classic.png`.

### (3) LIVE — the proper `ros2_control admittance_controller` (Harmonic)
`scripts/admittance_controller_wipe.py` drives the off-the-shelf
`admittance_controller/AdmittanceController` on **Gazebo Harmonic** (`gz_ros2_control`), which
has a clean **gravity-compensated** F/T sensor and a mass-damper-spring law. The **static**
compliant force is clean and linear (10 N at 15.5 mm reference penetration, no slam). The
*dynamic* wipe is unstable — Harmonic's weak position tracker. Plot:
`ros2control_admittance_harmonic.png`.

### Honest finding (why the software model is the primary)
**Clean physics-sim force-holding at 10/6 N with a position-controlled arm is not cleanly
achievable.** We confirmed this against the difficulty directly, and it is consistent with the
reference implementations we were able to inspect: the ones that hit spec use a **software
contact model** (like our primary), while a Gazebo-physics attempt holds only ~1–2 N with
sub-spec velocity. The PDF asks for a **simulated** F/T sensor, which the software spring-
damper model realizes cleanly — it is the spec-meeting delivery, not a fallback. We
additionally provide the two live physics demos as honest, reproducible bonuses.

**Deliverables:** `wiping_log.png` / `wiping_log_mirror.png` (force/velocity/dexterity),
`wiping_log*.csv`, `wiping_trajectory*.yaml`, `live_admittance_classic.png`,
`ros2control_admittance_harmonic.png`, `wiping_demo.gif`, `section3_execution.mp4`,
`writeup.md`.

---

## How to run (summary; full commands in `REPRODUCE.md`)

```bash
# build the workspace in the jazzy container, then:
bash scripts/gui/start_moveit.sh        # MoveIt + IK + scene
bash scripts/gui/reachability.sh        # Section 1 -> reachability.{png,csv}
ros2 launch gr_coverage section2.launch.py   # Section 2 -> coverage_path.{png,csv}, trajectories
bash scripts/gui/s3_primary.sh          # Section 3 PRIMARY -> wiping_log*.png (force/velocity/dexterity)
bash scripts/gui/s3_secondary.sh        # Section 3 LIVE force control on the real Gazebo F/T (Classic)
```

## Layout
```
ros2_ws/src/gr_assignment/   our 4 packages (gr_scene, gr_kinematics, gr_coverage, gr_wiping_control)
ros2_ws/src/piper_ros/       AgileX Piper description + MoveIt config (run dependency)
docker/                      Docker images + compose to replicate the environment
scripts/gui/                 one-command run scripts per section
outputs/                     deliverables (plots, CSVs, trajectories, per-section write-ups)
```
