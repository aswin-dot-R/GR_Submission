"""Section 2 planner node.

Generates raster (countertop) and spiral (mirror) coverage paths, projects
each Cartesian waypoint into MoveIt's compute_cartesian_path to get a joint
trajectory, time-parameterizes it, and reports coverage/length/exec-time
metrics. Outputs CSV + PNG of the path and a YAML of the trajectory.
"""
import os
import csv
import math
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK
from moveit_msgs.msg import RobotState, RobotTrajectory, MoveItErrorCodes, PositionIKRequest
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .planners import (
    Surface,
    raster_path,
    spiral_path,
    path_length,
    coverage_fraction,
)


def _q_align_z(target_z: np.ndarray) -> tuple[float, float, float, float]:
    """Quaternion (x,y,z,w) that rotates +z onto `target_z` (the desired tool
    z-axis). For tool pointing into the surface, pass -surface_normal.
    """
    src = np.array([0.0, 0.0, 1.0])
    tgt = target_z / np.linalg.norm(target_z)
    d = float(np.dot(src, tgt))
    if d > 0.9999:
        return (0.0, 0.0, 0.0, 1.0)
    if d < -0.9999:
        # 180° about x
        return (1.0, 0.0, 0.0, 0.0)
    axis = np.cross(src, tgt)
    axis /= np.linalg.norm(axis)
    angle = math.acos(d)
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


class CoverageNode(Node):
    def __init__(self):
        super().__init__("coverage_planner")
        # tool
        self.declare_parameter("tool.size_x", 0.100)
        self.declare_parameter("tool.size_y", 0.050)
        self.declare_parameter("tool.overlap", 0.15)
        self.declare_parameter("keepout_margin", 0.015)
        # counter
        self.declare_parameter("counter.frame", "base_link")
        self.declare_parameter("counter.center", [0.40, 0.00, 0.10])
        self.declare_parameter("counter.size",   [1.20, 0.60])
        self.declare_parameter("counter.normal", [0.0, 0.0, 1.0])
        self.declare_parameter("counter.standoff_z", 0.02)
        # mirror
        self.declare_parameter("mirror.frame", "base_link")
        self.declare_parameter("mirror.center", [0.45, 0.00, 0.30])
        self.declare_parameter("mirror.size",   [0.90, 0.60])
        self.declare_parameter("mirror.normal", [-1.0, 0.0, 0.0])
        self.declare_parameter("mirror.standoff_n", 0.02)
        # trajectory
        self.declare_parameter("cartesian.max_step", 0.005)
        self.declare_parameter("cartesian.jump_threshold", 0.0)
        self.declare_parameter("cartesian.avoid_collisions", True)
        self.declare_parameter("timeparam.max_vel_scale", 0.30)
        self.declare_parameter("timeparam.max_acc_scale", 0.30)
        # nominal per-joint speed limit (rad/s); scaled by max_vel_scale for retiming
        self.declare_parameter("timeparam.joint_speed_limit", 3.0)
        # yaw about the surface normal is a FREE DOF of a flat pad (see Section 1):
        # we search this many yaw angles and use the one that maximises reachable
        # waypoints, instead of pinning a single (often-infeasible) orientation.
        self.declare_parameter("orientation.yaw_samples", 12)
        # output
        self.declare_parameter("output.csv_path", "/home/dev/data/coverage_path.csv")
        self.declare_parameter("output.png_path", "/home/dev/data/coverage_path.png")
        self.declare_parameter("output.trajectory_path", "/home/dev/data/coverage_trajectory.yaml")
        # translucent IK-workspace shading behind the path (best-yaw reachability grid)
        self.declare_parameter("reach_field.enable", True)
        self.declare_parameter("reach_field.resolution", 0.04)

        self.tool_u = float(self.get_parameter("tool.size_x").value)
        self.tool_v = float(self.get_parameter("tool.size_y").value)
        self.overlap = float(self.get_parameter("tool.overlap").value)
        self.margin = float(self.get_parameter("keepout_margin").value)
        self.csv_path = self.get_parameter("output.csv_path").value
        self.png_path = self.get_parameter("output.png_path").value
        self.traj_path = self.get_parameter("output.trajectory_path").value
        self.yaw_samples = int(self.get_parameter("orientation.yaw_samples").value)
        self.field_enable = bool(self.get_parameter("reach_field.enable").value)
        self.field_res = float(self.get_parameter("reach_field.resolution").value)
        self.joint_speed = float(self.get_parameter("timeparam.joint_speed_limit").value) \
            * float(self.get_parameter("timeparam.max_vel_scale").value)

        cb = ReentrantCallbackGroup()
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path", callback_group=cb)
        if not self.cart_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_cartesian_path not available — start move_group first")
            raise SystemExit(1)
        # IK client used to test per-waypoint reachability (executable coverage)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb)
        self.ik_cli.wait_for_service(timeout_sec=10.0)

        # KDL chain for Yoshikawa manipulability w=sqrt(det(JJ^T)) at each IK solution:
        # separates IK-reachable cells from USABLE ones (well-conditioned, where the
        # arm can actually move/wipe), same lens as Section 1.
        import subprocess
        from gr_wiping_control.kdl_chain import Kinematics
        _urdf = subprocess.check_output(
            ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
        ).decode()
        self.kin = Kinematics(_urdf, "base_link", "link6")
        self.jnames = list(self.kin.joint_names)
        # 'usable' threshold as a fraction of the peak manipulability seen in the run.
        self.declare_parameter("manip.usable_frac", 0.30)
        self.usable_frac = float(self.get_parameter("manip.usable_frac").value)

    def _surface_counter(self) -> Surface:
        return Surface.from_horizontal(
            self.get_parameter("counter.center").value,
            self.get_parameter("counter.size").value,
            self.get_parameter("counter.normal").value,
        )

    def _surface_mirror(self) -> Surface:
        return Surface.from_vertical(
            self.get_parameter("mirror.center").value,
            self.get_parameter("mirror.size").value,
            self.get_parameter("mirror.normal").value,
        )

    def _orientation_xyzw(self, surface: Surface, yaw: float) -> tuple[float, float, float, float]:
        """Tool orientation as quaternion (x,y,z,w): tool z-axis aligned with the
        surface normal (pointing into the surface), then spun by `yaw` about that
        axis. The spin is the pad's free DOF — searched, not fixed (see Section 1).
        """
        r_align = Rot.from_quat(_q_align_z(-surface.normal))   # +z -> -normal (xyzw)
        r_yaw = Rot.from_rotvec([0.0, 0.0, yaw])               # spin about tool-local z
        q = (r_align * r_yaw).as_quat()                         # xyzw
        return float(q[0]), float(q[1]), float(q[2]), float(q[3])

    def _waypoints_to_poses(self, points: np.ndarray, surface: Surface, yaw: float) -> list[Pose]:
        qx, qy, qz, qw = self._orientation_xyzw(surface, yaw)
        poses = []
        for p in points:
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = map(float, p)
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = qx, qy, qz, qw
            poses.append(ps)
        return poses

    def _ik_solve(self, pose: Pose, frame: str):
        """Return (ok, joint_names, joint_positions) for a single IK query."""
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose = pose
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "link6"
        req.ik_request.pose_stamped = ps
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.nanosec = 500_000_000  # 0.5 s (KDL random-restarts within this)
        ev = threading.Event()
        fut = self.ik_cli.call_async(req)
        fut.add_done_callback(lambda _f, e=ev: e.set())
        if not ev.wait(timeout=2.0):
            return False, [], []
        res = fut.result()
        if not res or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, [], []
        js = res.solution.joint_state
        return True, list(js.name), list(js.position)

    def _ik_ok(self, pose: Pose, frame: str) -> bool:
        # Retry: MoveIt's KDL IK random-restarts each call, so a cell that is genuinely
        # reachable may miss on one shot. Retrying matches the success rate of Section 3's
        # seeded per-waypoint IK, so the reachability mask reflects what the arm can really
        # reach (no more "Section 2 denies a cell Section 3 wipes").
        for _ in range(4):
            if self._ik_solve(pose, frame)[0]:
                return True
        return False

    def _manip(self, pose: Pose, frame: str) -> float:
        """Manipulability at the IK solution for `pose` (0 if unreachable / singular)."""
        ok, names, pos = self._ik_solve(pose, frame)
        if not ok:
            return 0.0
        try:
            q = [pos[names.index(n)] for n in self.jnames]
            return self.kin.manipulability(q)
        except Exception:
            return 0.0

    @staticmethod
    def _largest_run(mask: list) -> tuple[int, int]:
        """Return [start, end) index range of the longest contiguous True run in
        `mask` (empty range if none). The Cartesian wipe stroke is built over this
        run so there are no jumps across unreachable gaps."""
        best_s = best_e = 0
        s = None
        for i, m in enumerate(mask + [False]):
            if m and s is None:
                s = i
            elif not m and s is not None:
                if i - s > best_e - best_s:
                    best_s, best_e = s, i
                s = None
        return best_s, best_e

    @staticmethod
    def _densify(pts: np.ndarray, uv: np.ndarray, spacing: float):
        """Interpolate a polyline so neighbouring samples are <= `spacing` apart.
        The raster/spiral generators only store stroke endpoints; densifying gives
        intermediate points that can individually pass/fail IK (the reachable
        middle of an otherwise-unreachable stroke)."""
        if len(pts) < 2:
            return pts, uv
        dpts, duv = [pts[0]], [uv[0]]
        for a, b, ua, ub in zip(pts[:-1], pts[1:], uv[:-1], uv[1:]):
            seg = float(np.linalg.norm(b - a))
            n = max(1, int(math.ceil(seg / spacing)))
            for k in range(1, n + 1):
                f = k / n
                dpts.append(a + (b - a) * f)
                duv.append(ua + (ub - ua) * f)
        return np.asarray(dpts), np.asarray(duv)

    def _best_yaw_and_reachable(self, points: np.ndarray, surface: Surface, frame: str):
        """Return (best_yaw, reach_mask, stroke_mask).

        - `best_yaw` maximises IK-reachable waypoints at a SINGLE orientation (a continuous
          wipe stroke keeps one pad yaw).
        - `stroke_mask` = reachable AT best_yaw — drives the executable Cartesian stroke.
        - `reach_mask` = reachable at ANY yaw (the pad's free DOF). This is the HONEST "can
          the arm place the pad on this cell" — the same definition as Section 1, and what
          the Section 3 demo actually achieves with per-waypoint IK. Using only the single
          stroke yaw UNDER-reports reachability (a cell can fail at best_yaw yet be reachable
          at another yaw), which previously made Section 2 mark cells unreachable that the
          arm demonstrably wipes in Section 3. The mask/CSV/plot now use `reach_mask` so the
          three sections agree; the stroke still uses `stroke_mask` (one valid orientation).
        """
        if len(points) == 0:
            return 0.0, [], []
        stride = max(1, len(points) // 60)
        probe = points[::stride]
        best_yaw, best_n = 0.0, -1
        for k in range(self.yaw_samples):
            yaw = 2.0 * math.pi * k / self.yaw_samples
            poses = self._waypoints_to_poses(probe, surface, yaw)
            n = sum(self._ik_ok(p, frame) for p in poses)
            if n > best_n:
                best_yaw, best_n = yaw, n
        stroke_mask = [self._ik_ok(p, frame)
                       for p in self._waypoints_to_poses(points, surface, best_yaw)]
        # any-yaw reachability: only re-test the cells that failed at best_yaw (cheap).
        reach_mask = list(stroke_mask)
        for i, ok in enumerate(stroke_mask):
            if ok:
                continue
            for k in range(self.yaw_samples):
                yaw = 2.0 * math.pi * k / self.yaw_samples
                p = self._waypoints_to_poses(points[i:i + 1], surface, yaw)[0]
                if self._ik_ok(p, frame):
                    reach_mask[i] = True
                    break
        return best_yaw, reach_mask, stroke_mask

    def _retime(self, traj: RobotTrajectory) -> float:
        """Simple time parameterization: assign each segment a duration so no joint
        exceeds `self.joint_speed` (rad/s), then fill time_from_start and per-joint
        velocities by finite difference. GetCartesianPath returns an un-timed path;
        this turns it into an executable, time-parameterized joint trajectory.
        (Trades the smoothness of MoveIt's TOTG/IPTP for a dependency-free retime;
        noted as a trade-off in the write-up.)
        """
        pts = traj.joint_trajectory.points
        if len(pts) < 2:
            return 0.0
        t = 0.0
        pts[0].time_from_start.sec, pts[0].time_from_start.nanosec = 0, 0
        for i in range(1, len(pts)):
            prev = np.asarray(pts[i - 1].positions)
            cur = np.asarray(pts[i].positions)
            dq = float(np.max(np.abs(cur - prev))) if len(cur) else 0.0
            dt = max(dq / max(self.joint_speed, 1e-6), 1e-3)
            t += dt
            pts[i].time_from_start.sec = int(t)
            pts[i].time_from_start.nanosec = int((t - int(t)) * 1e9)
            vel = (cur - prev) / dt
            pts[i].velocities = list(map(float, vel))
        pts[0].velocities = [0.0] * len(pts[0].positions)
        if pts[-1].positions:
            pts[-1].velocities = [0.0] * len(pts[-1].positions)
        return t

    def _compute_cartesian(self, poses: list[Pose], frame: str,
                           start_state: RobotState = None) -> tuple[RobotTrajectory, float, float]:
        req = GetCartesianPath.Request()
        req.header.frame_id = frame
        req.group_name = "arm"
        req.link_name = "link6"
        if start_state is not None:
            req.start_state = start_state  # begin AT the first reachable waypoint
        req.waypoints = poses
        req.max_step = float(self.get_parameter("cartesian.max_step").value)
        req.jump_threshold = float(self.get_parameter("cartesian.jump_threshold").value)
        req.avoid_collisions = bool(self.get_parameter("cartesian.avoid_collisions").value)
        ev = threading.Event()
        fut = self.cart_cli.call_async(req)
        fut.add_done_callback(lambda _f, e=ev: e.set())
        if not ev.wait(timeout=30.0):
            self.get_logger().error("compute_cartesian_path timed out")
            return RobotTrajectory(), 0.0, 0.0
        res = fut.result()
        traj = res.solution
        exec_t = self._retime(traj)  # GetCartesianPath is un-timed; parameterize it
        return traj, float(res.fraction), exec_t

    def _trajectory_exec_time(self, traj: RobotTrajectory) -> float:
        pts = traj.joint_trajectory.points
        if not pts:
            return 0.0
        last = pts[-1].time_from_start
        return last.sec + last.nanosec * 1e-9

    def _save_path(self, name: str, points: np.ndarray, uv: np.ndarray, surface: Surface,
                   reachable: list[bool]):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path.replace(".csv", f"_{name}.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["idx", "x", "y", "z", "u", "v", "reachable"])
            for i, (p, suv) in enumerate(zip(points, uv)):
                wr.writerow([i, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}",
                             f"{suv[0]:.4f}", f"{suv[1]:.4f}", int(reachable[i])])

    def _save_traj_yaml(self, name: str, traj: RobotTrajectory):
        path = self.traj_path.replace(".yaml", f"_{name}.yaml")
        with open(path, "w") as f:
            f.write(f"joint_names: {list(traj.joint_trajectory.joint_names)}\n")
            f.write("points:\n")
            for p in traj.joint_trajectory.points:
                t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                f.write(f"  - t: {t:.4f}\n")
                f.write(f"    positions: {list(p.positions)}\n")
                if p.velocities:
                    f.write(f"    velocities: {list(p.velocities)}\n")

    def _reachability_field(self, surf: Surface, frame: str, yaw: float, standoff: float):
        """Sample MANIPULABILITY w=sqrt(det(JJ^T)) on a regular (u,v) grid over the
        surface's inset region. grid[j,i] = w (0 if unreachable), so the shading shows
        the USABLE (well-conditioned) workspace — not just where IK didn't fail."""
        iu = surf.size_u / 2 - self.margin
        iv = surf.size_v / 2 - self.margin
        us = np.arange(-iu, iu + 1e-9, self.field_res)
        vs = np.arange(-iv, iv + 1e-9, self.field_res)
        pts = np.array([surf.to_world(u, v, standoff) for v in vs for u in us])
        poses = self._waypoints_to_poses(pts, surf, yaw)
        w = [self._manip(po, frame) for po in poses]
        grid = np.asarray(w, dtype=float).reshape(len(vs), len(us))
        return grid, [-iu, iu, -iv, iv]

    def _plot_both(self, plot_data: dict):
        """plot_data[name] = (uv_densified, mask, surf, run_start, run_end, title, field)."""
        # Clean binary reachability: GREEN = reachable / RED = unreachable (any-yaw — the
        # honest "can the arm place the pad here", same as Section 1 and consistent with the
        # Section 3 demo). The navy executed stroke is the time-parameterized joint-trajectory
        # segment (a continuous wipe at one fixed yaw). (Manipulability/usable-area is still
        # reported numerically in the metrics + write-up; it's just not shaded on the plot.)
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        for ax, name in [(axes[0], "counter"), (axes[1], "mirror")]:
            uv, mask, surf, s, e, title, field = plot_data[name]
            if len(uv):
                m = np.asarray(mask, dtype=bool)
                ax.scatter(uv[~m, 0], uv[~m, 1], s=12, c="tab:red", zorder=2,
                           label="unreachable")
                ax.scatter(uv[m, 0], uv[m, 1], s=12, c="tab:green", zorder=2,
                           label="reachable")
                if e > s:  # the time-parameterized wipe stroke (joint-trajectory segment)
                    ax.plot(uv[s:e, 0], uv[s:e, 1], "-", color="navy", lw=2.0, zorder=3,
                            label="executed stroke")
            inset = (surf.size_u / 2 - self.margin, surf.size_v / 2 - self.margin)
            ax.add_patch(plt.Rectangle(
                (-inset[0], -inset[1]), 2 * inset[0], 2 * inset[1],
                fill=False, ec="0.5", lw=0.8, ls="--"))
            ax.set_aspect("equal")
            ax.set_xlabel("u (m)"); ax.set_ylabel("v (m)")
            ax.set_title(title)
            ax.legend(loc="upper right", fontsize=7)
        plt.tight_layout()
        plt.savefig(self.png_path, dpi=120)

    def run(self):
        counter = self._surface_counter()
        mirror = self._surface_mirror()
        c_pts, c_uv = raster_path(counter, self.tool_u, self.tool_v, self.overlap, self.margin,
                                  float(self.get_parameter("counter.standoff_z").value))
        m_pts, m_uv = spiral_path(mirror, self.tool_u, self.tool_v, self.overlap, self.margin,
                                  float(self.get_parameter("mirror.standoff_n").value))

        results = {}
        plot_data = {}
        for name, pts, uv, surf, title in [
            ("counter", c_pts, c_uv, counter, "Countertop — raster"),
            ("mirror",  m_pts, m_uv, mirror,  "Mirror — spiral"),
        ]:
            frame = self.get_parameter(f"{name}.frame").value
            # geometric coverage of the full path over the (given) surface
            geom_cov = coverage_fraction(uv, surf, self.tool_u, self.tool_v, self.margin)
            length = path_length(pts)

            # densify so reachability is judged per-segment, not just at stroke ends
            pts, uv = self._densify(pts, uv, spacing=0.02)

            # reach_mask = any-yaw reachability (honest "can the arm reach it", matches
            # Sections 1 & 3); stroke_mask = reachable at the single stroke yaw.
            best_yaw, mask, stroke_mask = self._best_yaw_and_reachable(pts, surf, frame)
            reach_frac = (sum(mask) / len(mask)) if len(mask) else 0.0
            # executable coverage: geometric coverage counting only reachable waypoints
            reach_uv = uv[np.array(mask)] if len(uv) and any(mask) else np.empty((0, 2))
            exec_cov = coverage_fraction(reach_uv, surf, self.tool_u, self.tool_v, self.margin)

            self._save_path(name, pts, uv, surf, mask)   # CSV/plot use the any-yaw mask

            # joint trajectory over the LONGEST CONTIGUOUS stroke reachable at the SINGLE
            # stroke yaw (so compute_cartesian_path follows one valid orientation), started
            # from the IK solution of its first point.
            s, e = self._largest_run(stroke_mask)
            run_pts = pts[s:e] if e > s else np.empty((0, 3))
            poses = self._waypoints_to_poses(run_pts, surf, best_yaw)
            traj, frac, exec_t = RobotTrajectory(), 0.0, 0.0
            if len(poses) >= 2:
                # IK is non-deterministic: the start config picked for the stroke can
                # make the straight-line Cartesian follow infeasible. Retry a few
                # times (fresh start IK each time) and keep the best fraction.
                for _attempt in range(6):
                    ok, jn, jp = self._ik_solve(poses[0], frame)
                    start_state = RobotState(joint_state=JointState(name=jn, position=jp)) if ok else None
                    t_i, f_i, et_i = self._compute_cartesian(poses[1:], frame, start_state)
                    if f_i > frac:
                        traj, frac, exec_t = t_i, f_i, et_i
                    if frac > 0.95:
                        break
            n_joint_pts = len(traj.joint_trajectory.points)
            self._save_traj_yaml(name, traj)

            standoff = float(self.get_parameter(
                "counter.standoff_z" if name == "counter" else "mirror.standoff_n").value)
            field = (self._reachability_field(surf, frame, best_yaw, standoff)
                     if self.field_enable else None)
            # usable coverage: of the inset surface, what fraction is reachable, and
            # what fraction is USABLE (manipulability > usable_frac of the peak)?
            reach_area = usable_area = 0.0
            if field is not None:
                g = field[0]
                wmax = float(g.max()) if g.max() > 0 else 1.0
                reach_area = float((g > 0).sum()) / g.size
                usable_area = float((g > self.usable_frac * wmax).sum()) / g.size
            plot_data[name] = (uv, mask, surf, s, e, title, field)
            results[name] = (geom_cov, exec_cov, reach_frac, length, frac, exec_t,
                             n_joint_pts, reach_area, usable_area)
            self.get_logger().info(
                f"{name}: geom_coverage={geom_cov*100:.1f}%, exec_coverage={exec_cov*100:.1f}%, "
                f"reachable_area={reach_area*100:.1f}%, USABLE_area={usable_area*100:.1f}% "
                f"(well-conditioned), reachable_waypoints={sum(mask)}/{len(mask)}, "
                f"length={length:.3f} m, cartesian_fraction={frac*100:.1f}%, "
                f"exec_t={exec_t:.2f} s, joint_pts={n_joint_pts}"
            )

        self._plot_both(plot_data)
        self.get_logger().info(f"Wrote plot: {self.png_path}")
        return results


def main():
    rclpy.init()
    node = CoverageNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
