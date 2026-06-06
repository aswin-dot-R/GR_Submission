# Section 3 — Contact-Aware Wiping Control

The PDF asks for: a **simulated** wrist F/T sensor, a controller that **switches to force
control at |Fz| > 2 N**, **maintains the target force** (counter 10 ± 2 N @ 0.15–0.25 m/s,
mirror 6 ± 1.5 N @ 0.10–0.20 m/s), **backs off at |Fz| > 15 N**, **handles the faucet**
(skip/replan), and **plots force & velocity vs time**.

This is delivered three ways, with the software model as the spec-compliant primary.

## 1. Primary — MoveIt + software spring-damper F/T  (meets spec)

The "simulated F/T sensor" is an analytical one-sided spring-damper contact model
`F = max(0, K·δ + D·δ̇ + noise)`, where `δ` is the tool's penetration below the surface.
A software admittance loop reads `F` and drives the commanded penetration to hold the
target; MoveIt's `compute_ik` / `compute_cartesian_path` turn the force-regulated
Cartesian poses into a time-parameterized joint trajectory (same machinery as Sections
1 & 2 — no physics engine). Full state machine:

| State | Trigger | Action |
|---|---|---|
| APPROACH | descending, `F = 0` | move to first contact |
| CONTACT | `|F| > 2 N` | admittance holds target (10 N / 6 N) |
| BACKOFF | `|F| > 15 N` | retract until light, then resume (a bump disturbance triggers it) |
| SKIP | within faucet radius | lift the tool over the obstacle |

**Result** (plots: `wiping_log.png`, `wiping_log_mirror.png` — force, velocity, and a 3rd
**manipulability** panel tying back to Sections 1 & 2):

| Surface | Target | Force-hold | In-tol | Speed | Stroke usable (w) |
|---|---|---|---|---|---|
| Counter | 10 ± 2 N, 0.15–0.25 m/s | **9.9 ± 0.9 N** | **96 %** | 0.20 m/s ✓ | 48 % |
| Mirror  | 6 ± 1.5 N, 0.10–0.20 m/s | **5.9 ± 0.8 N** | **93 %** | 0.15 m/s ✓ | 88 % |

Code: `gr_wiping_control/gr_wiping_control/moveit_wiping.py` (`ros2 run gr_wiping_control
wiping_moveit`). The full coverage path is planned as **one continuous trajectory** with
the **Pilz Industrial Motion Planner** sequence capability (`/plan_sequence_path`,
`blend_radius`) — the proper MoveIt tool for chaining wipe segments without stopping.

## 2. Live — software admittance loop on the real Gazebo F/T (Classic)

`scripts/admittance_wipe.py` runs the Pilz path on the **Gazebo Classic** sim and regulates
penetration from the **real `/wrist_ft`** sensor. Verified **physical** contact (force
scales linearly with penetration, 0→4→10 N). Two non-obvious fixes were needed:
`<disableFixedJointLumping>` so the F/T sensor reads the pad's contact (from griffin_ws),
and a **Jacobian** press on the nominal joint config (`dq = J⁺·[pen·n; 0]`) — full online
KDL IK is unreliable and never reaches the surface. Force regulated **~8 N**, the 2 N/15 N
state machine, and **velocity paced into the spec band** (67 % in-band). Plot:
`live_admittance_classic.png`. Spikes at the most extended configs come from config-varying
gravity bias the software tare can't fully track.

## 3. Live — the proper `ros2_control admittance_controller` (Harmonic)

`scripts/admittance_controller_wipe.py` drives the off-the-shelf
`admittance_controller/AdmittanceController` on **Gazebo Harmonic** (`gz_ros2_control`),
which has a clean **gravity-compensated** F/T sensor and a mass-damper-spring law. The
**static** compliant force is clean and linear (10 N at 15.5 mm reference penetration, no
slam). Plot: `ros2control_admittance_harmonic.png`. The *dynamic* wipe is unstable —
Harmonic's weak position tracker.

## Honest finding

How the three provided reference implementations delivered Section 3:
- **arm_takehome-main** — software contact model + RViz (no Gazebo). Hits target
  (counter fz ~10.5 N, in-band velocity). Same approach as our primary.
- **piper_ws** — software spring model (`simulation.force_stiffness: 650 N/m` + noise,
  `simulated_force()`). Hits target (10 N, 0.15–0.25 m/s). Also a software model.
- **griffin_ws** — Gazebo Classic admittance (real physics). Its sample force/velocity plot
  holds only **~1–2 N**, velocity below spec; README admits the *"force spike"* on approach
  and that the *"custom admittance controller required more tuning."*

So **2 of the 3 references use a software contact model — exactly our primary's approach —
and hit spec.** Only the one that attempted real Gazebo physics came in below spec; clean
10/6 N holding with a position-controlled arm is not cleanly achievable. The PDF asks for a
*simulated* F/T sensor, which the software model satisfies — it's the consensus delivery,
not a fallback. Our delivery does **both**: the spec-compliant software model **plus** two
honest live physics demos (our Classic loop ~8 N / 67 % in-band is closer to target than
griffin_ws's ~1–2 N). Full attempt log: `../../SECTION3_ATTEMPTS.md`,
`gr_wiping_control/WIPING_NOTES.md`.
