## 🤖 PROMPT FOR CODEX (read this first)

You are working in the ROS 2 repo at `/home/ashie/GR_Assignment` (Docker only; see §2).
A robot-arm take-home "Section 3" (contact-aware wiping) is already **solved in software**
— a MoveIt + analytical spring-damper model holds the target force and meets spec (§3).

**Your task:** make the *physics* version actually hold force. Produce a **Gazebo
simulation** in which the arm wipes the countertop and the mirror while a **real contact
force/torque sensor** (a topic published by the sim, NOT the analytical `K·δ` model) reads
and the controller **holds 10 N ±2 N on the counter and 6 N ±1.5 N on the mirror** through
the sweep, with: force-control engaging at |Fz|>2 N, back-off at |Fz|>15 N, the faucet
obstacle skipped, and force-vs-time + velocity-vs-time logged to a plot.

**Definition of done:** a force-vs-time plot from a *physics* run (sourced from the sim's FT
topic, e.g. `/tcp_fts_broadcaster/wrench` in Harmonic or `/wrist_ft` in Classic) that stays
inside the target band for the wipe, plus reasonably smooth motion. The existing software
model (`moveit_wiping.py`) does NOT count — that part already works.

**Before coding, read §5 and §6.** They list exactly what's already been tried and ruled
out (rigid DART contact, ODE soft contact, compliant-tool geometry pinch, streaming diff-IK
stick-slip) and give ranked directions.

Two things are already done for you:
- The **asymmetric admittance controller** the reference uses (creep in with a tiny capped
  advance, retreat fast with a large capped pull-back, force low-pass, gravity tare) is
  **implemented** in `controller.py` as `contact_model:="asym"` (params `asym.*`). It just
  needs to be paired with a sim that gives a *soft* contact.
- **Gazebo Classic is DITCHED** — its `controller_manager` will not come up reliably (binds
  a stale `mock_components` robot_description and hangs). Do not spend time on it.

**Recommended path:** stay on **Gazebo Harmonic** but get a *soft* contact, because DART
(its default engine) ignores ODE `kp`/`kd`. Switch the physics engine to **Bullet /
Bullet-Featherstone**, which may honor surface contact stiffness:
`gz sim --physics-engine gz-physics-bullet-featherstone-plugin`, or in the world file:
`<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"><engine>
<filename>gz-physics-bullet-featherstone-plugin</filename></engine></plugin>`. Then set soft
`<surface><contact>` params and run the `asym` controller. (Verify the chosen engine
actually honors the softness — the gz docs don't confirm it; that's the open risk.)
Build/run instructions in §2; key files in §8.

If you conclude the physics force-hold isn't worth the effort, say so explicitly and why —
a reference implementation reached the same conclusion and used the software model (§1).

---

# Section 3 (Contact-Aware Wiping) — Attempts & Open Problem

Handoff doc. Self-contained record of every approach tried, what works, what doesn't,
the measured numbers, and the one **open problem** worth handing to another agent.

---

## 0. The task (assignment PDF, Section 3)

- **Objective:** force/impedance control in contact tasks.
- **Setup:** a **simulated** wrist force/torque sensor. Targets: **countertop 10 N ±2 N,
  0.15–0.25 m/s; mirror 6 N ±1.5 N, 0.10–0.20 m/s.**
- **Tasks:** (1) switch to force control when |Fz|>2 N; (2) maintain target force while
  following the path, back off if |Fz|>15 N; (3) handle an obstacle (faucet) by
  skipping/replanning; (4) log + plot force-vs-time and velocity-vs-time.
- **Deliverables:** controller code + configs; force/velocity plots; short demo (sim run).
- The word **"Gazebo" never appears** in the assignment. Section 1 says "MoveIt 2 **or
  equivalent**." So a *software* simulated sensor satisfies the spec; physics is optional.

## 1. TL;DR status

| Approach | Force-hold | Status |
|---|---|---|
| **MoveIt + software spring-damper** (`moveit_wiping.py`) | counter **9.8±1.2 N (92%)**, mirror **5.9±0.8 N (92%)** | ✅ **WORKS, meets spec** — this is the primary deliverable |
| Gazebo Harmonic physics + compliant tool (`controller.py`) | ~9 N briefly / 0.4 N under-press / 860 N slam (config-dependent) | ⚠️ motion + contact work, **force-hold never clean** |
| Gazebo Classic + soft ODE contact | force stayed ~0 | ❌ abandoned |
| Trajectory replay in Gazebo (`traj_player.py`) | n/a (visualization only) | ✅ smooth motion in Gazebo, but it's the *planned* path |

**The OPEN PROBLEM (section 6):** get a *clean, sustained* force-hold (10±2 N / 6±1.5 N)
in a real **Gazebo physics** sim with an actual contact F/T sensor — not the software model.

A reference implementation we were shown does Section 3 **the same way as our primary**
(software Hooke's-law contact + impedance controller, **no physics engine**), which is
strong evidence that the software model *is* the intended/standard solution and the
physics force-hold is an optional stretch.

## 2. Environment / how to run

- Repo: `/home/ashie/GR_Assignment`. ROS 2, Docker only (`docker/docker-compose.yml`).
- Containers: **`gr_jazzy`** (ROS 2 Jazzy + Gazebo **Harmonic**/gz-sim, DART physics) and
  **`gr_humble`** (ROS 2 Humble + Gazebo **Classic**/gazebo-11, ODE physics).
- Workspace mounted at `/home/dev/ros2_ws`; `data/` ↔ `/home/dev/data` (outputs).
- Build a pkg: `colcon build --packages-select gr_wiping_control --symlink-install
  --build-base build_jazzy --install-base install_jazzy` (drop the `_jazzy` suffixes in humble).
- Helper scripts in `scripts/gui/*.sh` (run on host, drive the container via `docker compose exec`).

---

## 3. Approach 1 — MoveIt + software spring-damper  ✅ PRIMARY (works)

**File:** `ros2_ws/src/gr_assignment/gr_wiping_control/gr_wiping_control/moveit_wiping.py`
(entry point `wiping_moveit`).

**Idea:** the "simulated F/T sensor" is an analytical Hooke spring-damper + noise; an
**admittance** loop (force in → penetration out) holds the target; **MoveIt** does the IK.

- Contact model (the sensor): `F = max(0, K_c·δ + D_c·δ̇ + noise)`, δ = tool penetration
  below the surface. **Per-surface soft K: counter 1000 N/m, mirror 800 N/m**
  (`{counter,mirror}.contact_stiffness_n_per_m`), `D_c=60`, `noise ~ N(0, 0.2 N)`
  (`contact.noise_std_n`). 10 N at ~10 mm, 6 N at ~7.5 mm.
- Controller: admittance `dvel = (F_target − F)/D_adm`, integrate to penetration, clamp.
  State machine APPROACH→CONTACT(@2 N)→BACKOFF(@15 N)→SKIP(faucet). A small surface
  **bump** disturbance is injected (`bump.*`) so the 15 N back-off visibly fires.
- IK / trajectory: builds the raster (counter) / spiral (mirror) path in a surface-normal
  frame (counter n=+z, mirror n=−x), simulates the force loop over it, then calls MoveIt
  `/compute_ik` + `/compute_cartesian_path` (with `avoid_collisions=false` because wiping
  *intends* contact) to emit a **time-parameterized joint trajectory**. Optional `execute:=true`
  runs it on the arm in RViz via the `/execute_trajectory` action.
- **Results:** counter 9.8±1.2 N (92% in 10±2 N), Cartesian 100%, 214 joint pts;
  mirror 5.9±0.8 N (92% in 6±1.5 N), Cartesian 47% (the 47% is the Piper reachability
  limit of the mirror band, not a control issue).
- **Run:** `scripts/gui/wipe_moveit.sh` (counter), `wipe_moveit_mirror.sh` (mirror),
  `wipe_moveit_both.sh` (counter→mirror sequence, executes in RViz). All auto-start
  move_group via `ensure_moveit` in `scripts/gui/_lib.sh`.
- Outputs: `data/wiping_log{,_mirror}.{csv,png}`, `data/wiping_trajectory{,_mirror}.yaml`.

This fully satisfies the spec. Everything below is the optional "real physics" attempt.

---

## 4. Approach 2 — Gazebo Harmonic physics + compliant tool  ⚠️ (force-hold never clean)

**Files:** `controller.py` (entry `wiping_controller`), `description/piper_wiping_gz.xacro`,
`worlds/wiping_gz.sdf`, `config/wiping_controllers_gz.yaml`, `launch/gz_wiping.launch.py`.
Run: `scripts/gui/start_gazebo.sh` then `scripts/gui/wipe.sh` (counter) / `wipe_mirror.sh`.

**Stack:** gz-sim Harmonic (DART), `gz_ros2_control/GazeboSimSystem`, a real
`force_torque` sensor → `force_torque_sensor_broadcaster` → `/tcp_fts_broadcaster/wrench`,
a `JointTrajectoryController` (position+velocity). `controller.py` is an admittance
controller using KDL **streaming differential IK** (`kdl_chain.py`): each tick it nudges
the commanded joint vector toward the Cartesian target and publishes a 1-point trajectory.

**What works:** the arm spawns, controllers run, the FT sensor reads real contact, and the
arm physically does APPROACH→DESCEND→CONTACT→raster. Brief in-band wiping is visible.

**What fails — the core wall:** DART's contact is effectively **rigid (~390 N/mm, measured)**.
- Rigid pad mount → 10 N needs 0.026 mm penetration (finer than command resolution) →
  force **slams to ~860 N** or quantizes; back-off thrashes.
- Added a **compliant tool**: `tool_joint` made a *passive prismatic spring-damper*
  (`<springStiffness>` in the xacro). At **500 N/m** it held ~**9 N (42% in band)** but
  **spiked when the spring bottomed out** its 30 mm travel.
- Tried your idea of a **softer spring (200 N/m, 80 mm travel)**: the slam/back-off
  **disappeared** (good!) but it then **under-pressed to ~0.4 N** — geometry pinch: for a
  *downward* counter press the wrist can only travel ~20 mm before it would dive into the
  slab, and 200 N/m needs ~50 mm to make 10 N, so the spring never compresses. Net:
  usable spring is pinched to ~500 N/m for this geometry; no clean hold.
- DART **ignores ODE `<kp>/<kd>`**, so the *surface* can't be softened in Harmonic.
- Also: the streaming diff-IK **stick-slips at contact** and occasionally glitches (saw a
  7 m/s velocity spike), so the motion is jerky and eventually loses contact.

**Current xacro state:** compliant `tool_joint` spring **500 N/m**, 30 mm travel.

---

## 5. Approach 3 — Gazebo Classic + soft ODE contact  ❌ abandoned

**Files (DEPRECATED):** `description/piper_wiping.xacro`, `worlds/wiping.world`,
`config/wiping_controllers.yaml`, `launch/gazebo_wiping.launch.py` (run in `gr_humble`).

**Idea (from the reference impl):** Gazebo **Classic/ODE honors soft contact** via
`<surface><contact><ode><kp>`. Set the surfaces soft (`kp=5000, kd=50, mu=0.6` in
`wiping.world`) and the pad soft (`kp=5000`), so force = kp·penetration is gentle.

**What we confirmed:** the soft contact **does remove the slam** (no back-off thrash) —
ODE honors it, unlike DART. So softening the *surface* is the right lever in Classic.

**Why it was abandoned:**
- Our `controller.py` (streaming diff-IK, written for the MoveIt/Harmonic pipeline) does
  **not press/sense cleanly** against the Classic position-JTC: force stayed **~0**, depth
  ran to its cap, and the diff-IK glitched. The arm doesn't build force.
- FT wiring: a *fixed* `tool_joint` lumps the pad into link6 and the wrist FT (on joint6)
  doesn't see the contact. Switched to a **locked revolute** `tool_joint` with
  `<provideFeedback>` + the `libgazebo_ros_ft_sensor.so` plugin reading **that** joint
  (the reference design) — FT still read ~0 in our runs.
- Gazebo Classic is **EOL (Jan 2025)** and not packaged for Jazzy.
- That reference recipe (slow 0.1 mm/step approach, asymmetric admittance — retract 3×
  faster than advance — force low-pass, MoveIt only for the approach) is now **implemented**
  as `controller.py`'s `contact_model:="asym"`. But the Classic **sim itself is too flaky**:
  its `controller_manager` repeatedly fails to come up — it binds a *stale* `mock_components`
  robot_description (a leftover RSP) instead of `gazebo_ros2_control/GazeboSystem` and hangs
  (~8 min, no controllers). Gazebo Classic is also EOL (Jan 2025).
- **DECISION: Classic is DITCHED.** Don't try to revive it. If you want a physics force-hold,
  do it on Harmonic with a Bullet engine + the `asym` controller (see §6 / the top prompt).

---

## 6. THE OPEN PROBLEM (for Codex)

> **Achieve a clean, sustained contact force-hold in a Gazebo *physics* simulation** — i.e.
> a real contact + a real wrist F/T sensor reading, holding **10 N ±2 N on the counter and
> 6 N ±1.5 N on the mirror** while sweeping at the spec speeds, with the 2 N switch, 15 N
> back-off, and faucet skip — *without* falling back to the software contact model.

What's been ruled out / learned (don't re-tread):
- **Harmonic/DART**: rigid contact (~390 N/mm); ignores ODE kp/kd; a compliant tool spring
  is pinched by counter-press geometry (~500 N/m only, still spikes on bottom-out).
- **Classic/ODE**: soft contact (`kp`) works and removes slam, but our streaming diff-IK
  controller can't press/sense cleanly against the position-JTC; force ~0.

Promising directions a fresh attempt could take (pick one and make it hold force):
1. **Classic + a purpose-built controller** (most likely to work): keep `kp=5000` soft
   surfaces; replace the streaming diff-IK with the reference recipe — MoveIt (or one IK
   call) only to *approach* above the start, then a slow direct descent (~0.1 mm/step) until
   |F|>2 N, then an **asymmetric admittance** (retract gain ≫ advance gain) with a force
   **low-pass filter**, publishing position via the JTC action. Tune K and the gains so it
   settles within the band in ~1 s. (This is essentially porting the reference controller.)
2. **Harmonic + softer *contact*, not a tool spring**: switch the gz-sim physics engine to
   **bullet-featherstone** (which may honor contact softness) or find a DART contact-softness
   knob, so penetration is mm-scale and a simple force-on-penetration loop works. Risky;
   needs verifying the engine honors it.
3. **Effort/torque control**: command joint torques via `τ = Jᵀ·F_desired` (+ gravity comp)
   so the arm renders a force directly against the rigid wall — true impedance, no
   penetration needed. Needs an effort command interface + dynamics; biggest change.

Definition of done: a force-vs-time plot from a **physics** run (real FT topic, not the
software model) that sits inside 10±2 N (counter) and 6±1.5 N (mirror) for the sweep, with
visible 2 N switch / 15 N back-off / faucet skip, and reasonably smooth motion.

---

## 7. Bonus that already works: smooth Gazebo *visualization*

`traj_player.py` (entry `traj_player`) replays a saved MoveIt trajectory YAML on Gazebo's
`/arm_controller/follow_joint_trajectory` **action** (the bare topic races the pub/sub
connection and gets dropped; and the transit point must be at t=0 with the wipe starting at
t=transit, else `time_from_start` isn't strictly increasing → goal rejected). This gives a
**smooth** counter→mirror motion in the Gazebo window (no stick-slip) because the JTC
follows one continuous time-parameterized trajectory. It's the *planned* path replayed in
physics, so it's a visualization, not a force-control result.
Run: `scripts/gui/start_gazebo.sh` then `scripts/gui/wipe_gz_both.sh`.

## 8. Key files

```
gr_wiping_control/
  gr_wiping_control/moveit_wiping.py     # PRIMARY: software spring-damper + MoveIt IK
  gr_wiping_control/controller.py        # gz Harmonic admittance (streaming diff-IK)
  gr_wiping_control/kdl_chain.py         # KDL FK/Jacobian/IK for the streaming controller
  gr_wiping_control/traj_player.py       # replay a trajectory YAML on a JTC (Gazebo viz)
  description/piper_wiping_gz.xacro      # Harmonic robot + compliant tool spring (500 N/m)
  description/piper_wiping.xacro         # Classic robot (DEPRECATED): FT plugin, locked-revolute pad
  worlds/wiping_gz.sdf                   # Harmonic world (DART)
  worlds/wiping.world                    # Classic world (ODE soft contact kp=5000) DEPRECATED
  config/wiping.yaml                     # all Section-3 params (forces, K, noise, regions, gains)
  config/wiping_controllers_gz.yaml      # Harmonic ros2_control
  config/wiping_controllers.yaml         # Classic ros2_control (DEPRECATED)
  launch/gz_wiping.launch.py             # Harmonic bring-up
  launch/gazebo_wiping.launch.py         # Classic bring-up (DEPRECATED)
  WIPING_NOTES.md                        # design notes / trade-offs
scripts/gui/                             # wipe_moveit{,_mirror,_both}.sh, wipe{,_mirror}.sh,
                                         #   wipe_gz_both.sh, start_moveit.sh, start_gazebo.sh, _lib.sh
```
