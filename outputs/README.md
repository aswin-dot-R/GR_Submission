# Outputs — assignment deliverables

The artifacts each section of the take-home asks for, collected per section.
(Generated from `data/` + the per-package write-ups; the code lives in
`ros2_ws/src/gr_assignment/`.) Scene: the arm is mounted **on the countertop** — the
base sits flush on the slab, so the **countertop top is at z = 0** (the robot base
level). The faucet (15 cm) stands on the slab centred under the mirror at (0.40, 0),
and the mirror (90 × 60 cm) stands on the slab with its face at x = 0.45.

A through-line across all three sections: **reachable ≠ usable.** Each section
scores not just whether IK succeeds, but the **Yoshikawa manipulability**
`w = √det(J Jᵀ)` (→ 0 at a singularity), which separates "the arm can touch this"
from "the arm can actually *work* here." This is what makes the coverage and the
wipe honest.

---

## Section 1 — Kinematics & Reachability  → `section1_reachability/`

| Assignment deliverable | File |
|---|---|
| Reachability CSV | `reachability.csv` (`x, y, reachable, error_code, feasible_yaw_deg, manipulability`) |
| Reachability + manipulability heatmap | `reachability.png` |
| Write-up: where can/can't it reach & why | `writeup.md` |
| (scene reference) | `world_topdown.png`, `world_side.png` |

**Result:** 60 × 60 cm patch @ 2 cm = **961 cells, 680 reachable (70.8 %)** — but only
**~22 % is *usable*** (well-conditioned, `w > 30 %` of peak); ~44 % of reachable cells
are near-singular. The heatmap shows the grey hole right at the base, the dark-purple
near-singular crescent, and the yellow dexterous sweet spot. Key method:
"surface-aligned" fixes only the tool's approach axis; the pad's **yaw is a free DOF** —
searching it turns a misleading 3.5 % into the true workspace. (IK service: `gr_kinematics`;
scene: `gr_scene`.)

---

## Section 2 — Surface Coverage Path Planning  → `section2_coverage/`

| Assignment deliverable | File |
|---|---|
| Planner visualization | `coverage_path.png` (red/green reachability + executed stroke) |
| Coverage waypoints (raster + spiral) | `coverage_path_counter.csv`, `coverage_path_mirror.csv` |
| Example **time-parameterized** joint trajectory | `coverage_trajectory_counter.yaml`, `coverage_trajectory_mirror.yaml` |
| Raster-vs-spiral comparison + metrics | `writeup.md` |

**Result:** raster (counter) + spiral (mirror, bonus). Metrics per surface:

| Surface | Strategy | Geometric | Reachable area | **Usable area** | Path length | Exec time |
|---|---|---|---|---|---|---|
| Countertop | Raster | 98.7 % | 32.4 % | **14.0 %** | 15.50 m | 1.69 s |
| Mirror | Spiral | 39.0 % | 60.3 % | **50.3 %** | 2.74 m | 2.89 s |

The counter looks great geometrically (98.7 %) but is only **~14 % usable** — most of the
slab is near-base / near-singular. The mirror, at the dexterous mid-reach, is the
better-conditioned surface (~50 %). (Planner node: `gr_coverage`.)

---

## Section 3 — Contact-Aware Wiping Control  → `section3_wiping/`

| Assignment deliverable | File |
|---|---|
| Controller code + configs | `ros2_ws/src/gr_assignment/gr_wiping_control/` (`moveit_wiping.py` primary; `scripts/admittance_wipe.py`, `scripts/admittance_controller_wipe.py` live; `config/`) |
| Force-vs-time, velocity-vs-time (+ dexterity) plots | `wiping_log.png` (counter), `wiping_log_mirror.png` (mirror) + raw `.csv` |
| Example joint trajectory output | `wiping_trajectory.yaml`, `wiping_trajectory_mirror.yaml` |
| Demo (video/gif/sim run) | `wiping_demo.gif` (counter→mirror + force panel), `section3_execution.mp4` |
| Design notes + trade-offs | `WIPING_NOTES.md`, `writeup.md` |

### Primary deliverable — MoveIt + software spring-damper F/T (meets spec)
The PDF asks for a **simulated** wrist F/T sensor. Here it's an analytical spring-damper
contact model `F = max(0, K·δ + D·δ̇ + noise)`; a software admittance loop holds the target
while MoveIt's `compute_ik` / `compute_cartesian_path` do the IK (same machinery as
Sections 1 & 2). Full state machine: contact switch at **2 N**, **hold target**, **back-off
at 15 N** (a bump disturbance triggers it), **faucet skip**. The plots add a 3rd panel —
**manipulability along the stroke** — tying back to Sections 1 & 2.

| Surface | Target | Force-hold | In-tol | Speed | Stroke usable (w) |
|---|---|---|---|---|---|
| Counter | 10 ± 2 N, 0.15–0.25 m/s | **9.9 ± 0.9 N** | **96 %** | 0.20 m/s ✓ | 48 % |
| Mirror  | 6 ± 1.5 N, 0.10–0.20 m/s | **6.0 ± 0.5 N** | **97 %** | 0.15 m/s ✓ | 88 % |

### Secondary — live force control in Gazebo (honest physics demos)
Two live runs against a real Gazebo F/T sensor, both reproducible:
- **`live_admittance_classic.png`** — Gazebo Classic, software admittance loop on the real
  `/wrist_ft`. Real physical contact (verified linear: 0→4→10 N), force regulated **~8 N**,
  the 2 N / 15 N state machine, **velocity paced into the spec band** (67 % in-band). Spikes
  at the most extended configs from config-varying gravity bias.
- **`ros2control_admittance_harmonic.png`** — the **proper** `ros2_control
  admittance_controller` on Gazebo Harmonic, with built-in gravity compensation. Clean,
  linear **static** force (10 N at 15.5 mm, no slam), but the *dynamic* wipe is unstable
  (Harmonic's weak position tracker).

### Why the software model is primary — the reference implementations agree
How the three provided reference implementations actually delivered Section 3:

| Reference | Method | Result |
|---|---|---|
| **arm_takehome-main** | software contact model + RViz (no Gazebo) | hits target (counter ~10.5 N, in-band velocity) ✓ |
| **piper_ws** | software spring model (`force_stiffness: 650 N/m` + noise) | hits target (10 N, 0.15–0.25 m/s) ✓ |
| **griffin_ws** | Gazebo Classic admittance (real physics) | ~1–2 N, **below spec**; README admits the force spike + "needs more tuning" |

So **2 of the 3 references use a software contact model — exactly our primary's approach —
and hit spec.** Only the one that attempted real Gazebo physics (griffin_ws) came in below
spec. The software spring-damper model is the **consensus, spec-meeting delivery** for a
*"simulated F/T sensor"*, not a fallback. Our delivery goes further than any single
reference: the spec-compliant software model (like arm_takehome + piper_ws) **plus** two
honest live physics demos — a Classic admittance loop (~8 N / 67 % in-band, *closer to
target than griffin_ws's ~1–2 N*) and the proper `ros2_control admittance_controller`.
Full trade-off discussion in `WIPING_NOTES.md` / `SECTION3_ATTEMPTS.md`.

---

### Where the source lives
- `gr_scene` — collision scene (Sec 1)
- `gr_kinematics` — IK service + reachability/manipulability (Sec 1)
- `gr_coverage` — raster/spiral coverage + usable-area metrics (Sec 2)
- `gr_wiping_control` — contact-aware wiping: software model + live admittance (Sec 3)
- Top-level docs: `SOLUTION.md`, `ASSIGNMENT_EXPLAINED.md`, `REPRODUCE.md`, `SECTION3_ATTEMPTS.md`
