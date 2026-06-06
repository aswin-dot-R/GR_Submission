# Section 2 — Surface coverage: raster vs spiral, and reachability

**Tool:** rectangular pad 100 × 50 mm. **Overlap:** 15 %. **Keep-out margin:**
15 mm. **Surfaces:** countertop 120 × 60 cm (raster) and mirror 90 × 60 cm
(spiral, bonus). All values parameterized in `config/coverage.yaml`.

Outputs: `data/coverage_path.png`, `data/coverage_path_{counter,mirror}.csv`,
`data/coverage_trajectory_{counter,mirror}.yaml`.

## Metrics (measured)

Measured on the finalized scene (robot seated on the slab; counter top z = 0 ·
faucet on the slab at (0.40, 0) centred under the mirror · mirror face x = 0.44,
standing on the slab, centre z = 0.30):

| Surface | Strategy | Geometric coverage | Reachable area | **USABLE area** | Path length | Exec time |
|---|---|---|---|---|---|---|
| Countertop | Raster (boustrophedon) | 98.7 % | 32.4 % | **14.0 %** | 15.50 m | 1.69 s |
| Mirror | Spiral (Archimedean) | 39.0 % | 60.3 % | **50.3 %** | 2.74 m | 2.89 s |

- **Geometric coverage** = fraction of the inset surface the pad sweeps, ignoring the arm.
- **Reachable area** = collision-free IK solution exists.
- **USABLE area** = reachable AND well-conditioned (Yoshikawa manipulability
  `w = √det(JJᵀ) > 30 %` of the peak) — where the arm can actually move/wipe, not
  just touch in a near-singular pose. This is the honest number: the countertop is
  geometrically 98.7 % coverable but only **~14 % usable** (most of the slab is
  near-base / near-singular), while the mirror sits in the dexterous mid-reach and
  is **~50 % usable**. The heatmap shades by `w` with a white "usable" contour.
  (Cartesian fraction = 100 % for both executed strokes.)

- **Geometric coverage** = fraction of the (margin-inset) surface area swept by
  the pad footprint along the planned path, ignoring the arm.
- **Executable coverage** = the same, counting only waypoints the arm can
  actually strike (collision-aware IK, any yaw about the surface normal). This
  is the honest "what can it really wipe" number.

## Why executable << geometric

The countertop (120 × 60 cm) is far larger than the Piper's ~0.5 m reach (see
Section 1). The raster covers the whole slab, but the arm can only execute the
**near band** — 269 of 788 densified waypoints. The plot colours reachable
(green) vs unreachable (red) and draws the executed stroke (navy).

The **mirror sits at x = 0.44 m (face), centre z = 0.25 m, and is now well
reachable** — 145 / 212 (68 %) of its spiral waypoints solve, giving 36.3 %
executable coverage and a 3.96 s trajectory. Lowering the pane centre from
z = 0.30 to z = 0.25 lifted the reachable fraction from ~37 % to 68 % (the arm
can't reach the upper third of a 90 cm pane — see Section 1). The unreachable
36 % is the top band of the spiral, exactly as the reachability map predicts.

## Raster vs spiral — comparison

| | Raster (boustrophedon) | Spiral (Archimedean) |
|---|---|---|
| Best for | Rectangular / bounded planar surfaces | Round / centro-symmetric surfaces |
| Overlap control | Exact, via row pitch = `tool_v · (1 − overlap)` | Approximate, via turn pitch |
| Turns / reversals | Many 180° reversals at row ends (jerk, dwell) | One continuous inward curve (smooth) |
| Edge/corner coverage | Good — strokes reach the inset rectangle corners | Weaker — square corners left under-covered (see 39 % geometric) |
| This task | Countertop ✓ (rectangular) | Mirror (bonus) ✓ — smooth, 68 % of pane reachable |

**Takeaway:** raster is the right default for these rectangular surfaces and
gives predictable overlap; the spiral is a smoother, lower-jerk motion that
shines on round/centro-symmetric surfaces but under-covers rectangular corners
(39 % geometric on the square pane). On the mirror the spiral executes cleanly
over the reachable lower 68 % of the pane; a raster would cover the corners
better but with more reversals. For the countertop the raster wins outright.

## Pipeline (Cartesian → joint trajectory)

1. Generate raster/spiral waypoints in surface-local (u, v), lift to 3D with a
   2 cm standoff along the surface normal.
2. Orientation: tool z aligned to the surface normal; **yaw is a free pad DOF**.
3. **Reachability mask = ANY-YAW**: a cell counts as reachable if a collision-free IK
   solution exists at *any* yaw (with IK retried, since KDL random-restarts each call).
   This is the same definition as Section 1, and it matches what the arm actually reaches
   in the Section 3 demo — so the three sections agree (an earlier best-yaw-only mask
   wrongly marked near-base cells unreachable that the arm demonstrably wipes).
4. Build the executable joint trajectory over the **longest contiguous stroke reachable
   at a SINGLE best yaw** (a continuous wipe keeps one pad orientation — counter best yaw
   ≈ 150°), seeded from a start IK state so `compute_cartesian_path` begins on the surface
   (retry handles IK non-determinism). Cartesian fraction achieved: **100 %**. The navy
   "executed stroke" on the plot is this single-yaw run; the green dots are the any-yaw mask.
5. **Time parameterization:** assign each segment a duration so no joint exceeds
   `timeparam.joint_speed_limit · max_vel_scale` (rad/s), then fill
   `time_from_start` and per-joint velocities by finite difference.

### Trade-offs / notes
- Time parameterization is a dependency-free constant-joint-speed retime, not
  MoveIt's TOTG/IPTP — adequate for an executable, smoothly-timed stroke; a
  production system would use TOTG for jerk/accel-limited profiles.
- Reachability mask has minor run-to-run noise from IK's randomized seed at the
  0.2 s solve timeout; raise the IK timeout for a cleaner mask if needed.

## Reproduce
```bash
bash scripts/section2.sh
# outputs: data/coverage_path.png, coverage_path_*.csv, coverage_trajectory_*.yaml
```
