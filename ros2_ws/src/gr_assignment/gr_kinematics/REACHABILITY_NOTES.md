# Section 1 — Where can / can't the arm reach, and why?

**Arm:** AgileX Piper (6-DOF), base at `base_link` origin, mounted **on the
countertop** (slab top surface at z = 0 in `base_link` = the base level, so the
base sits flush on the slab and the arm reaches out/down onto it).
**Patch:** 60 × 60 cm centred at (x, y) = (0.22, 0.00) m, sampled at 2 cm →
31 × 31 = **961 cells**. Shifted IN toward the base so it spans the near-base
field where the arm folds and passes through shoulder/elbow singularities.
**Probe pose:** `link6` placed at the cell (x, y) at z = 0.02 m (2 cm above the
slab top at z = 0), tool **surface-aligned** — i.e. the tool approach axis
(`link6` +z) pointing straight down (−z_world = the horizontal counter's normal).

## Headline result

**680 / 961 cells reachable (70.8 %)** — collision-aware IK against the loaded
planning scene (countertop, faucet, mirror). But reachable ≠ usable: at each
reachable cell we also score the **Yoshikawa manipulability w = √det(J Jᵀ)**
(→ 0 at a singularity), and ~44 % of reachable cells are near-singular
(w < 15 % of the peak). The heatmap colours reachable cells by w:

- **Grey hole at the base** — the arm cannot reach the counter directly at/under
  its own shoulder (the links can't fold there); a real workspace hole.
- **Dark-purple crescent** — reachable but low-manipulability: only achievable
  near shoulder/elbow singularities, poor dexterity (can't push in all directions).
- **Yellow band (x ≈ 0.35–0.40)** — the dexterous sweet spot, well-conditioned,
  where force/velocity control during wiping actually works well.
- **Grey far/edge** — beyond the Piper's reach.

See `data/reachability.png` (manipulability heatmap) and `data/reachability.csv`
(`x, y, reachable, error_code, feasible_yaw_deg, manipulability`).

## Where it CAN reach
- The **near-to-mid field in front of the base**, roughly x ∈ [0.10, 0.50] m,
  spanning the full ±0.30 m in y on the near side.
- This matches the Piper's ~0.5 m horizontal reach: the arm comfortably strikes
  the part of the counter directly in front of and beside itself.

## Where it CAN'T reach, and why
1. **Far edge (x ≳ 0.52 m):** beyond the arm's kinematic reach. Holding the tool
   vertical at surface height that far out would require near-full extension
   *and* a downward wrist — outside the workspace. (`error_code = -31`,
   `NO_IK_SOLUTION`.)
2. **Faucet keep-out:** a pocket on the −y side around (0.20, −0.20) m is rejected
   by collision-aware IK — the faucet collision object (a 15 cm tall column)
   blocks the approach. This is the asymmetry visible in the heatmap.
3. **Directly under / behind the base:** the arm is mounted *on* the counter, so
   poses at its own base plane pointing straight down are geometrically awkward
   and largely infeasible.

## The subtle part: "surface-aligned" ≠ one fixed orientation

A first version of the sweep pinned the **full** end-effector orientation to a
single quaternion `[w,x,y,z] = [0,1,0,0]` (tool-down at yaw = 0°). That reported
only **34 / 961 = 3.5 %** reachable — two tiny lobes at the near corners — which
looks like a crippled arm.

That number is an **artifact of over-constraining a free DOF**, not a real
workspace limit:

- "Surface-aligned" fixes only the tool's **approach axis** to the surface
  normal. The **yaw about that axis** (how a square wiping pad is spun) is a
  *free* degree of freedom — the pad wipes identically at 0° or 90°.
- Verified directly: the patch centre (0.30, 0.00) — flagged *unreachable* at
  yaw = 0° — actually solves at multiple yaw angles with the tool still pointing
  straight down.
- The 34 "reachable" cells of the broken sweep are exactly the cells whose
  feasible yaw happens to include 0°.

**Fix:** keep the tool pointing down (surface normal, unchanged) but search a
configurable set of yaw angles per cell (`reachability.yaml: yaw_samples: 12`)
and mark a cell reachable if *any* yaw has a collision-free IK solution; the
feasible yaw is recorded in the CSV. None of the assignment-given inputs change
(patch 60 × 60 cm, 2 cm resolution, surface-height probe, tool-down). Only the
unspecified yaw is freed. Result: a single pinned yaw reports only a few percent
(two corner lobes); freeing the yaw gives the honest surface-aligned workspace,
**70.8 %** reachable on this patch — of which only ~22 % is *usable* (non-singular).

## Reproduce
```bash
bash scripts/section1.sh          # move_group + scene + IK + sweep
# outputs: data/reachability.{csv,png}
```
