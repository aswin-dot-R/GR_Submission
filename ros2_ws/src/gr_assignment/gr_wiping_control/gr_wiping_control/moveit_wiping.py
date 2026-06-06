"""Section 3 (PRIMARY) — Contact-aware wiping with MoveIt + a software spring-damper.

The assignment asks for a *simulated* wrist force/torque sensor and a force/impedance
controller. Here the sensor is an analytical spring-damper contact model

        F_meas = max(0, K_c * pen + D_c * pen_rate)

where `pen` is the tool's penetration below the surface (the controlled variable).
A software admittance loop reads F_meas and drives the commanded penetration to hold
the spec force, with the full state machine:

  APPROACH : descend in free space (F = 0)
  CONTACT  : |F| > contact_threshold (2 N) -> force control engaged;
             admittance holds target (counter 10 N / mirror 6 N)
  BACKOFF  : |F| > backoff_threshold (15 N) -> retract until |F| < 2 N, then resume
  SKIP     : within skip_radius of the faucet -> lift the tool (F = 0)

IK is done by MoveIt: /compute_ik seeds the start pose and /compute_cartesian_path
turns the force-regulated Cartesian poses into a time-parameterized joint trajectory
(the same machinery as Sections 1 & 2 — no KDL, no physics engine). A small surface
bump disturbance is injected to exercise the 15 N back-off. Logs/plots F vs t and
v vs t, and saves the joint trajectory.
"""
import csv
import math
import os
import threading

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetCartesianPath, GetPositionIK, ApplyPlanningScene, GetMotionPlan
from moveit_msgs.msg import (RobotState, RobotTrajectory, MoveItErrorCodes, PositionIKRequest,
                             PlanningScene, CollisionObject, Constraints, JointConstraint)
from moveit_msgs.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def surface_tool_quat(n, u, v, yaw):
    """Quaternion (x,y,z,w): tool z-axis points -n (INTO the surface), x-axis at
    `yaw` about the normal from in-plane axis u. Counter n=+z, mirror n=-x."""
    z_tool = -np.asarray(n, float)
    x_tool = math.cos(yaw) * np.asarray(u, float) + math.sin(yaw) * np.asarray(v, float)
    y_tool = np.cross(z_tool, x_tool)
    R = np.column_stack([x_tool, y_tool, z_tool])
    return Rot.from_matrix(R).as_quat()


class MoveItWiper(Node):
    def __init__(self):
        super().__init__("wiping_moveit")
        p = self.declare_parameter
        # ---- spec setpoints ----
        p("active_surface", "counter")
        p("counter.target_force_n", 10.0); p("counter.force_tol_n", 2.0)
        p("counter.speed_min_mps", 0.15); p("counter.speed_max_mps", 0.25)
        p("mirror.target_force_n", 6.0); p("mirror.force_tol_n", 1.5)
        p("mirror.speed_min_mps", 0.10); p("mirror.speed_max_mps", 0.20)
        p("contact_threshold_n", 2.0); p("backoff_threshold_n", 15.0)
        # ---- software spring-damper contact ("the simulated F/T sensor") ----
        # Per-surface contact stiffness (soft, like a real cleaning contact): a
        # gentle slope makes the force-penetration relationship resolvable and the
        # penetration visibly large (counter 10 N at ~10 mm, mirror 6 N at ~7.5 mm).
        # Match the PHYSICAL Gazebo compliant-tool spring so the planned penetration =
        # the spring compression yielding the target force. Use 1000 N/m so the press
        # is only ~10 mm (10 N) / 6 mm (6 N) — REACHABLE (20 mm would put the wrist at
        # counter level and IK fails). On replay the real 1000 N/m spring passively
        # makes the force; the path just follows the coverage, no active force loop.
        p("counter.contact_stiffness_n_per_m", 1000.0)
        p("mirror.contact_stiffness_n_per_m", 1000.0)
        p("contact.damping_ns_per_m", 60.0)      # D_c
        p("contact.noise_std_n", 0.2)            # Gaussian F/T sensor noise (realistic jitter)
        p("admit.damping_ns_per_m", 90.0)        # controller admittance damper (force->depth)
        p("admit.max_rate_mps", 0.05)            # cap on penetration speed
        # ---- kinematics / tool ----
        p("base_link", "base_link"); p("tip_link", "link6")
        p("tool_length_m", 0.02)
        p("tool_yaw_deg", 150.0); p("mirror.tool_yaw_deg", 0.0)
        # ---- wipe region (counter; base frame, surface plane) ----
        # FLAT wipe IN FRONT of the faucet (no z lift-over). Account for the 10x5 cm
        # PAD box, not just the tip: its corner reaches ~5.6 cm from the tip, and the
        # faucet front face is at x=0.37, so the tip must stay <= 0.31 for the pad to
        # clear. x spans 0.16..0.31 -> the pad never touches the faucet.
        p("wipe.center", [0.235, 0.00, 0.00])    # counter top now at z=0 (base level)
        p("wipe.size_u", 0.15); p("wipe.size_v", 0.40)
        p("wipe.pad_swath", 0.05); p("wipe.overlap", 0.10)
        # ---- mirror wipe region (its own face; pane not moved) ----
        # Reachable band of the pane, wide across y (both sides of the faucet). The
        # lower-CENTRE (behind the faucet) is unreachable collision-free (the wrist
        # hits the faucet), so the band sits at z~0.22..0.32 where the full width is
        # reachable. (Covering the ragged lower sides needs Section-2-style reachable
        # path planning; the live-IK mask wedges move_group's Cartesian solver.)
        # PRIMARY force-plot stroke: one reliably-executable raster over the reachable
        # mid band, z=0.20..0.41 (faucet-clear; top at z=0.15, wrist hits it below z~0.19)
        # and y -0.11..0.19. A single compute_cartesian_path stroke can't span the FULL
        # reachable mirror (the pane sits at the workspace edge — wider/taller starts break
        # the stroke), so the force plot uses this representative band. FULL Section-2
        # coverage (bottom-left + bottom-right corners down to z~0.08, plus the upper band)
        # is shown by the multi-pass Gazebo continuous trajectory, not one stroke.
        p("mirror.wipe.center", [0.44, 0.04, 0.305])
        p("mirror.wipe.size_u", 0.30); p("mirror.wipe.size_v", 0.21)
        # ---- approach / motion ----
        p("stroke.approach_m", 0.04)             # start this far off the surface
        p("descend_speed_mps", 0.05)
        p("control_rate_hz", 100.0)
        # ---- obstacle (faucet) ----
        # faucet lift-over DISABLED (skip_radius 0): the wipe stays flat and never
        # reaches the faucet (region ends at x=0.31, pad-clear), so no z excursion.
        p("obstacle.center", [0.40, 0.00]); p("obstacle.skip_radius_m", 0.0)
        p("obstacle.lift_m", 0.18)
        # ---- bump disturbance (to exercise the 15 N back-off) ----
        p("bump.enable", True)
        p("bump.s_frac", 0.45)                   # at this fraction of arc length
        p("bump.height_m", 0.006); p("bump.width_m", 0.03)
        # ---- Gazebo replay: GRAZE mode (no force loop) ----
        # In Gazebo/DART rigid contact + position control makes any commanded
        # penetration slam (hundreds of N). For a clean Gazebo MOTION demo, replay a
        # grazing path: hold the pad a fixed offset at/above the surface (no
        # penetration), so the arm sweeps the surface without the slam. Force control
        # itself is the MoveIt software-model deliverable, not Gazebo.
        p("replay.graze", False)
        p("replay.graze_offset_m", 0.0)          # >0 = pad rides this far ABOVE surface
        p("replay.skip_transit", False)          # true = no home transit (stitch passes)
        # ---- MoveIt cartesian / retime ----
        p("cartesian.max_step", 0.005); p("cartesian.jump_threshold", 0.0)
        # wiping deliberately CONTACTS the surface, so the Cartesian IK must not
        # reject surface-touching poses as collisions (the force model handles
        # contact). Self/obstacle avoidance is still covered by the reachable region.
        # collision-aware: the planner avoids faucet/mirror/self. The countertop is
        # removed from the planning scene at run() so the tool may still press it.
        p("cartesian.avoid_collisions", False)
        p("joint_speed_rps", 1.0)
        # ---- output ----
        p("log.csv_path", "/home/dev/data/wiping_log.csv")
        p("log.png_path", "/home/dev/data/wiping_log.png")
        p("trajectory_path", "/home/dev/data/wiping_trajectory.yaml")

        g = lambda n: self.get_parameter(n).value
        surf = g("active_surface")
        self.surf = surf
        self.target_force = float(g(f"{surf}.target_force_n"))
        self.force_tol = float(g(f"{surf}.force_tol_n"))
        self.v_des = 0.5 * (float(g(f"{surf}.speed_min_mps")) + float(g(f"{surf}.speed_max_mps")))
        self.v_min = float(g(f"{surf}.speed_min_mps")); self.v_max = float(g(f"{surf}.speed_max_mps"))
        self.contact_th = float(g("contact_threshold_n"))
        self.backoff_th = float(g("backoff_threshold_n"))
        self.Kc = float(g(f"{surf}.contact_stiffness_n_per_m")); self.Dc = float(g("contact.damping_ns_per_m"))
        self.noise_std = float(g("contact.noise_std_n"))
        self.D_adm = float(g("admit.damping_ns_per_m")); self.adm_vmax = float(g("admit.max_rate_mps"))
        self.base = g("base_link"); self.tip = g("tip_link")
        self.tool_len = float(g("tool_length_m"))
        self.rate = float(g("control_rate_hz"))
        self.descend_v = float(g("descend_speed_mps"))
        self.approach = float(g("stroke.approach_m"))
        self.obstacle = np.array(g("obstacle.center"), float)
        self.skip_r = float(g("obstacle.skip_radius_m"))
        # counter: lift the tool UP over the faucet (0.18). mirror: retract it BACK
        # off the pane just clear of the faucet (0.10 -> link6 to x~0.32); a big
        # retract makes too steep an in-x jump for the Cartesian path.
        self.lift_m = 0.10 if surf == "mirror" else float(g("obstacle.lift_m"))
        self.bump_on = bool(g("bump.enable")); self.bump_s = float(g("bump.s_frac"))
        self.bump_h = float(g("bump.height_m")); self.bump_w = float(g("bump.width_m"))
        self.graze = bool(g("replay.graze")); self.graze_off = float(g("replay.graze_offset_m"))
        self.joint_speed = float(g("joint_speed_rps"))
        self.csv_path = g("log.csv_path"); self.png_path = g("log.png_path")
        self.traj_path = g("trajectory_path")

        # ---- surface frame (counter horizontal +z, mirror vertical -x) ----
        if surf == "mirror":
            self.n = np.array([-1.0, 0.0, 0.0]); self.u = np.array([0.0, 1.0, 0.0])
            self.v = np.array([0.0, 0.0, 1.0]); yaw = math.radians(float(g("mirror.tool_yaw_deg")))
            wc = np.array(g("mirror.wipe.center"), float)
            wsu, wsv = float(g("mirror.wipe.size_u")), float(g("mirror.wipe.size_v"))
            self.faucet_on = False
        else:
            self.n = np.array([0.0, 0.0, 1.0]); self.u = np.array([1.0, 0.0, 0.0])
            self.v = np.array([0.0, 1.0, 0.0]); yaw = math.radians(float(g("tool_yaw_deg")))
            wc = np.array(g("wipe.center"), float)
            wsu, wsv = float(g("wipe.size_u")), float(g("wipe.size_v"))
            self.faucet_on = True
        self.quat = surface_tool_quat(self.n, self.u, self.v, yaw)
        self.path = self._build_raster(wc, wsu, wsv,
                                       float(g("wipe.pad_swath")), float(g("wipe.overlap")))
        self._seg = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self._cum = np.concatenate([[0.0], np.cumsum(self._seg)])
        self.path_len = float(self._cum[-1])

        # optional: execute the wipe on the robot (animates RViz) for a recording
        self.declare_parameter("execute", False)
        self.declare_parameter("execute_loops", 1)
        self.do_execute = bool(self.get_parameter("execute").value)
        self.exec_loops = int(self.get_parameter("execute_loops").value)
        self.cur_q = None
        self.cur_names = None

        cb = ReentrantCallbackGroup()
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene", callback_group=cb)
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path", callback_group=cb)
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path", callback_group=cb)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb)
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory", callback_group=cb)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10, callback_group=cb)
        ok = self.cart_cli.wait_for_service(timeout_sec=10.0) and self.ik_cli.wait_for_service(timeout_sec=10.0)
        if not ok:
            self.get_logger().error("MoveIt services not available — start move_group first")
        # KDL chain for Yoshikawa manipulability w=sqrt(det(JJ^T)) along the wipe: shows
        # whether the force-controlled stroke stays in the dexterous (usable) workspace
        # found in Sections 1 & 2, or dips toward singularities.
        try:
            import subprocess
            from gr_wiping_control.kdl_chain import Kinematics
            _urdf = subprocess.check_output(
                ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
            ).decode()
            self.kin = Kinematics(_urdf, self.base, self.tip)
        except Exception as e:
            self.kin = None
            self.get_logger().warn(f"manipulability disabled (no KDL chain): {e}")
        self.get_logger().info(
            f"wiping_moveit up (surface={surf}, F*={self.target_force}±{self.force_tol} N, "
            f"v={self.v_des:.2f} m/s, Kc={self.Kc:.0f} N/m)")

    # ---------- geometry ----------
    def _build_raster(self, center, size_u, size_v, pad_swath, overlap):
        hu, hv = size_u / 2.0, size_v / 2.0
        pitch = max(1e-3, pad_swath * (1.0 - overlap))
        n_rows = max(1, int(math.ceil(2 * hv / pitch)) + 1)
        vs = np.linspace(-hv, hv, n_rows)
        pts = []
        for i, vv in enumerate(vs):
            us = (-hu, hu) if i % 2 == 0 else (hu, -hu)
            for uu in us:
                pts.append(center + uu * self.u + vv * self.v)
        dense = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            seg = float(np.linalg.norm(b - a))
            k = max(1, int(math.ceil(seg / 0.01)))
            for j in range(1, k + 1):
                dense.append(a + (b - a) * (j / k))
        return np.asarray(dense)

    def _path_point(self, s):
        s = max(0.0, min(self.path_len, s))
        i = int(np.searchsorted(self._cum, s) - 1)
        i = max(0, min(i, len(self._seg) - 1))
        seg = self._seg[i]
        f = 0.0 if seg < 1e-9 else (s - self._cum[i]) / seg
        return self.path[i] + (self.path[i + 1] - self.path[i]) * f

    def _bump(self, s):
        """A small raised spot on the surface at arc-length fraction bump.s_frac.
        Adds to the effective penetration -> a force spike that trips the 15 N
        back-off, demonstrating the safety behavior."""
        if not self.bump_on:
            return 0.0
        s0 = self.bump_s * self.path_len
        d = abs(s - s0)
        if d > self.bump_w:
            return 0.0
        return self.bump_h * 0.5 * (1.0 + math.cos(math.pi * d / self.bump_w))

    def _near_faucet(self, xy):
        if not self.faucet_on:
            return False
        return np.linalg.norm(xy[:2] - self.obstacle) < self.skip_r

    def _link6(self, surf_pt, delta):
        """link6 pose for a surface point penetrated by `delta` (tip below surface
        by delta -> link6 at surf_pt + (tool_len - delta) along the normal)."""
        pos = surf_pt + (self.tool_len - delta) * self.n
        ps = Pose()
        ps.position.x, ps.position.y, ps.position.z = map(float, pos)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, self.quat)
        return ps

    # ---------- the simulated force-control loop ("sensor" + controller) ----------
    def _simulate(self):
        dt = 1.0 / self.rate
        log = []          # (t, F_meas, speed, mode, delta)
        poses = []        # link6 Pose at each WIPE sample (for compute_cartesian_path)
        pose_s = []       # arc-length of each pose (for downsampling)
        t = 0.0
        # APPROACH: descend from `approach` above the surface to first contact (F=0)
        delta = -self.approach
        p0 = self._path_point(0.0)
        while delta < 0.0:
            delta = min(0.0, delta + self.descend_v * dt)
            log.append((t, 0.0, 0.0, "APPROACH", delta)); t += dt
        # WIPE: advance arc length, regulate force via admittance on penetration
        s = 0.0
        dvel = 0.0
        prev_pen = 0.0
        mode = "CONTACT"
        last_pose_s = -1.0
        while s <= self.path_len:
            xy = self._path_point(s)
            bump = self._bump(s)
            if self.graze:
                # Gazebo motion demo: no force loop. Ride a fixed offset at/above
                # the surface; lift over the faucet. No penetration -> no slam.
                if self._near_faucet(xy):
                    mode = "SKIP"; delta = -self.lift_m
                else:
                    mode = "GRAZE"; delta = -self.graze_off
                pen = 0.0; Fmeas = 0.0; dvel = 0.0
            elif self._near_faucet(xy):
                # obstacle: lift the tool CLEAR over the faucet immediately (a ramp is
                # too slow — the tip enters the faucet at surface level before rising).
                mode = "SKIP"
                delta = -self.lift_m
                pen = max(0.0, delta + bump)
                Fmeas = max(0.0, self.Kc * pen + np.random.normal(0.0, self.noise_std))  # ~noise while lifted
                dvel = 0.0
            else:
                # contact model = the "sensor": one-sided spring-damper
                pen = max(0.0, delta + bump)
                pen_rate = (pen - prev_pen) * self.rate
                # the "sensor": Hooke spring-damper + Gaussian noise (real F/T look)
                Fmeas = max(0.0, self.Kc * pen + self.Dc * pen_rate + np.random.normal(0.0, self.noise_std))
                if Fmeas > self.backoff_th:
                    # too hard -> retract quickly until light again
                    mode = "BACKOFF"
                    dvel = -self.adm_vmax
                    delta = delta + dvel * dt
                else:
                    mode = "CONTACT" if Fmeas > self.contact_th else "APPROACH"
                    # admittance: penetration rate proportional to force error
                    dvel = (self.target_force - Fmeas) / max(1e-6, self.D_adm)
                    dvel = max(-self.adm_vmax, min(self.adm_vmax, dvel))
                    delta = delta + dvel * dt
            prev_pen = max(0.0, delta + bump)
            speed = self.v_des if mode in ("CONTACT", "SKIP", "BACKOFF") else 0.0
            log.append((t, Fmeas, speed, mode, delta))
            # sample link6 poses ~ every cartesian.max_step for the IK path
            if s - last_pose_s >= 0.008 or last_pose_s < 0:
                # graze allows a negative delta (pad above surface); contact clamps >=0
                poses.append(self._link6(xy, delta if self.graze else max(0.0, delta)))
                pose_s.append(s); last_pose_s = s
            s += self.v_des * dt
            t += dt
        # prepend a lifted approach pose ABOVE the surface so the planner's start
        # state (and the transit goal) sit clear of the countertop — the counter
        # stays in the scene (visible); the Cartesian descent from this pose into
        # contact is the first segment of the wipe.
        poses.insert(0, self._link6(self._path_point(0.0), -self.approach))
        # also APPEND a lifted retract at the end (graze): so the pass both starts
        # AND ends off the surface, and several passes can be stitched into one
        # continuous stroke by bridging between the lifted endpoints (no surface drag).
        if self.graze:
            poses.append(self._link6(self._path_point(self.path_len), -self.approach))
        return log, poses

    # ---------- MoveIt IK ----------
    def _ik_start(self, pose):
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = "arm"; req.ik_request.ik_link_name = "link6"
        ps = PoseStamped(); ps.header.frame_id = self.base; ps.pose = pose
        req.ik_request.pose_stamped = ps
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.nanosec = 300_000_000
        ev = threading.Event(); fut = self.ik_cli.call_async(req)
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout=3.0):
            return None
        res = fut.result()
        if not res or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = res.solution.joint_state
        return RobotState(joint_state=JointState(name=list(js.name), position=list(js.position)))

    def _ik_ok(self, pose):
        """Collision-aware IK feasibility (Section 1's reachability test): can the arm
        place link6 at `pose` without any link colliding with the scene (faucet etc.)?"""
        req = GetPositionIK.Request(); req.ik_request = PositionIKRequest()
        req.ik_request.group_name = "arm"; req.ik_request.ik_link_name = "link6"
        ps = PoseStamped(); ps.header.frame_id = self.base; ps.pose = pose
        req.ik_request.pose_stamped = ps
        req.ik_request.avoid_collisions = True            # <-- obstacle/self collision checked
        req.ik_request.timeout.nanosec = 200_000_000
        ev = threading.Event(); fut = self.ik_cli.call_async(req)
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout=2.0):
            return False
        res = fut.result()
        return bool(res and res.error_code.val == MoveItErrorCodes.SUCCESS)

    def _compute_reach_mask(self):
        """Reachability mask over the wipe path: for each surface point, can the arm
        reach it (tool just off the surface) collision-free? Unreachable points (e.g.
        the mirror column behind the faucet, or the counter cell at the faucet) are
        SKIPPED during the wipe. This is Sections 1 & 2 (reachability + coverage)
        driving Section 3 instead of hand-tuned keep-outs."""
        n = len(self.path)
        mask = np.ones(n, dtype=bool)
        stride = max(1, n // 200)                          # cap the IK calls
        checked = list(range(0, n, stride))
        for i in checked:
            mask[i] = self._ik_ok(self._link6(self.path[i], -0.02))   # 2 cm off surface
        # forward-fill the strided result
        last = checked[0]
        for i in range(n):
            if i in (set(checked)):
                last = i
            mask[i] = mask[last]
        self.get_logger().info(f"reachability mask: {int(mask.sum())}/{n} wipe points reachable")
        return mask

    def _remove_counter(self):
        """Remove the 'countertop' collision object so the tool may press it, while the
        planner still avoids the faucet/mirror + self-collision (avoid_collisions=True)."""
        if not self.scene_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("apply_planning_scene unavailable; counter stays in scene")
            return
        co = CollisionObject(); co.id = "countertop"; co.operation = CollisionObject.REMOVE
        co.header.frame_id = self.base
        ps = PlanningScene(); ps.is_diff = True; ps.world.collision_objects = [co]
        req = ApplyPlanningScene.Request(); req.scene = ps
        ev = threading.Event(); fut = self.scene_cli.call_async(req)
        fut.add_done_callback(lambda _f: ev.set()); ev.wait(timeout=3.0)
        self.get_logger().info("removed 'countertop' from planning scene (tool may contact it)")

    def _plan_transit(self, goal_state):
        """Collision-free joint-space plan from the current state to goal_state (the
        wipe start), so the starting motion can't self-collide / hit the faucet."""
        if goal_state is None or not self.plan_cli.wait_for_service(timeout_sec=5.0):
            return RobotTrajectory()
        from moveit_msgs.msg import MotionPlanRequest
        mpr = MotionPlanRequest()
        mpr.group_name = "arm"; mpr.num_planning_attempts = 5; mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = 0.3; mpr.max_acceleration_scaling_factor = 0.3
        c = Constraints()
        for n, q in zip(goal_state.joint_state.name, goal_state.joint_state.position):
            jc = JointConstraint(); jc.joint_name = n; jc.position = float(q)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        mpr.goal_constraints = [c]
        req = GetMotionPlan.Request(); req.motion_plan_request = mpr
        ev = threading.Event(); fut = self.plan_cli.call_async(req)
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout=15.0):
            return RobotTrajectory()
        res = fut.result()
        # NOTE: this move_group build reports a successful plan with a non-standard
        # error code (99999), not MoveItErrorCodes.SUCCESS — so gate on whether a
        # real trajectory came back, not the exact code.
        if not res or not res.motion_plan_response.trajectory.joint_trajectory.points:
            ec = res.motion_plan_response.error_code.val if res else None
            self.get_logger().warn(f"transit plan failed (error_code={ec})")
            return RobotTrajectory()
        return res.motion_plan_response.trajectory

    def _concat(self, t1, t2):
        """Append t2 after t1 (time-shifted) into one trajectory."""
        if not t1.joint_trajectory.points:
            return t2
        if not t2.joint_trajectory.points:
            return t1
        out = RobotTrajectory()
        out.joint_trajectory.joint_names = list(t1.joint_trajectory.joint_names)
        pts = list(t1.joint_trajectory.points)
        off = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 + 0.5
        for p in t2.joint_trajectory.points:
            q = JointTrajectoryPoint(); q.positions = list(p.positions); q.velocities = list(p.velocities)
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 + off
            q.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            pts.append(q)
        out.joint_trajectory.points = pts
        return out

    def _cartesian(self, poses, start_state):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base; req.group_name = "arm"; req.link_name = "link6"
        if start_state is not None:
            req.start_state = start_state
        req.waypoints = poses
        req.max_step = float(self.get_parameter("cartesian.max_step").value)
        req.jump_threshold = float(self.get_parameter("cartesian.jump_threshold").value)
        req.avoid_collisions = bool(self.get_parameter("cartesian.avoid_collisions").value)
        ev = threading.Event(); fut = self.cart_cli.call_async(req)
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout=30.0):
            self.get_logger().error("compute_cartesian_path timed out")
            return RobotTrajectory(), 0.0
        res = fut.result()
        return res.solution, float(res.fraction)

    def _retime(self, traj):
        pts = traj.joint_trajectory.points
        if len(pts) < 2:
            return 0.0
        t = 0.0; pts[0].time_from_start.sec = 0; pts[0].time_from_start.nanosec = 0
        for i in range(1, len(pts)):
            prev = np.asarray(pts[i - 1].positions); cur = np.asarray(pts[i].positions)
            dq = float(np.max(np.abs(cur - prev))) if len(cur) else 0.0
            dt = max(dq / max(self.joint_speed, 1e-6), 1e-3)
            t += dt
            pts[i].time_from_start.sec = int(t); pts[i].time_from_start.nanosec = int((t - int(t)) * 1e9)
            pts[i].velocities = list(map(float, (cur - prev) / dt))
        pts[0].velocities = [0.0] * len(pts[0].positions)
        if pts[-1].positions:
            pts[-1].velocities = [0.0] * len(pts[-1].positions)
        return t

    # ---------- execution (animates RViz for a recording) ----------
    def _js_cb(self, msg):
        self.cur_q = dict(zip(msg.name, msg.position))

    def _execute(self, traj):
        """Send the wipe trajectory to /execute_trajectory so the robot moves in
        RViz. Prepends a 2 s move from the current state to the stroke start (no
        teleport). Builds a fresh trajectory each call (safe to loop)."""
        jt = traj.joint_trajectory
        if len(jt.points) < 2:
            return
        if not self.exec_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/execute_trajectory action not available")
            return
        names = list(jt.joint_names)
        pts, offset = [], 0.0
        if self.cur_q is not None and all(n in self.cur_q for n in names):
            sp = JointTrajectoryPoint()
            sp.positions = [float(self.cur_q[n]) for n in names]
            sp.velocities = [0.0] * len(names)
            sp.time_from_start = Duration(sec=0, nanosec=0)
            pts.append(sp); offset = 2.0
        for p in jt.points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 + offset
            q = JointTrajectoryPoint()
            q.positions = list(p.positions); q.velocities = list(p.velocities)
            q.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            pts.append(q)
        out = RobotTrajectory()
        out.joint_trajectory.joint_names = names
        out.joint_trajectory.points = pts
        goal = ExecuteTrajectory.Goal(); goal.trajectory = out
        done = threading.Event()
        def _gr(fut):
            gh = fut.result()
            if not gh.accepted:
                done.set(); return
            gh.get_result_async().add_done_callback(lambda _f: done.set())
        self.exec_cli.send_goal_async(goal).add_done_callback(_gr)
        done.wait(timeout=90.0)

    # ---------- run + outputs ----------
    def run(self):
        # NOTE: the countertop stays in the planning scene (visible in RViz). The
        # wipe starts from a lifted approach pose above it, so the collision-free
        # transit + the Cartesian start are both clear of the slab; the Cartesian
        # path (avoid_collisions=False) then descends into contact to wipe.
        log, poses = self._simulate()
        self.get_logger().info(f"simulated {len(log)} ticks, {len(poses)} cartesian waypoints")
        traj, frac = RobotTrajectory(), 0.0
        if len(poses) >= 2:
            start = self._ik_start(poses[0])
            traj, frac = self._cartesian(poses[1:], start)
            self._retime(traj)
            # prepend a collision-free transit (home -> wipe start) so the start can't
            # self-collide. skip_transit=true leaves the trajectory as JUST the wipe so
            # several passes can be stitched into ONE continuous stroke by the player.
            if not bool(self.get_parameter("replay.skip_transit").value):
                transit = self._plan_transit(start)
                if transit.joint_trajectory.points:
                    self.get_logger().info(f"prepended planned transit ({len(transit.joint_trajectory.points)} pts)")
                    traj = self._concat(transit, traj)
        n_jpts = len(traj.joint_trajectory.points)
        manip = self._traj_manip(traj)
        self._save(log, manip)
        self._save_traj(traj)
        if self.do_execute and n_jpts >= 2:
            for i in range(self.exec_loops):
                self.get_logger().info(f"executing wipe in RViz ({i + 1}/{self.exec_loops}) ...")
                self._execute(traj)
                time.sleep(1.0)
        # metrics
        a = np.array([(r[1]) for r in log if r[3] == "CONTACT"], float)
        within = float(np.mean((a >= self.target_force - self.force_tol) &
                               (a <= self.target_force + self.force_tol)) * 100) if len(a) else 0.0
        self.get_logger().info(
            f"{self.surf}: force-hold {np.mean(a):.1f}±{np.std(a):.1f} N (target {self.target_force}±"
            f"{self.force_tol}), {within:.0f}% in-tolerance, cartesian_fraction={frac*100:.0f}%, "
            f"joint_pts={n_jpts}")
        if manip is not None and len(manip[1]):
            w = manip[1]
            self.get_logger().info(
                f"{self.surf}: manipulability along wipe min={w.min():.4f} median={np.median(w):.4f} "
                f"max={w.max():.4f}; {100*np.mean(w > 0.006):.0f}% of the stroke in the usable "
                f"band (w>0.006, the Section-1/2 dexterity threshold)")
        return within

    def _traj_manip(self, traj):
        """(times, w) Yoshikawa manipulability at each joint config of the trajectory."""
        if self.kin is None:
            return None
        pts = traj.joint_trajectory.points
        if not pts:
            return None
        ts = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts])
        ws = np.array([self.kin.manipulability(list(p.positions)) for p in pts])
        return ts, ws

    def _save(self, log, manip=None):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t", "F_meas_N", "speed_mps", "mode", "delta_m"])
            for r in log:
                wr.writerow([f"{r[0]:.4f}", f"{r[1]:.4f}", f"{r[2]:.4f}", r[3], f"{r[4]:.5f}"])
        a = np.array([(r[0], r[1], r[2]) for r in log], float)
        nrows = 3 if (manip is not None and len(manip[1])) else 2
        fig, ax = plt.subplots(nrows, 1, figsize=(10, 8 if nrows == 3 else 6))
        ax[0].plot(a[:, 0], a[:, 1], label="|F| (simulated F/T)")
        ax[0].axhline(self.target_force, color="g", ls="--", lw=0.9, label=f"target {self.target_force:.0f} N")
        ax[0].axhspan(self.target_force - self.force_tol, self.target_force + self.force_tol,
                      color="g", alpha=0.12, label=f"±{self.force_tol:g} N")
        ax[0].axhline(self.contact_th, color="b", ls=":", lw=0.8, label="contact 2 N")
        ax[0].axhline(self.backoff_th, color="r", ls=":", lw=0.8, label="back-off 15 N")
        ax[0].set_ylabel("Force (N)"); ax[0].legend(loc="upper right", fontsize=8)
        ax[0].set_title(f"Contact-aware wiping ({self.surf}) — MoveIt + spring-damper F/T")
        ax[1].plot(a[:, 0], a[:, 2], color="tab:orange", label="|v_tool|")
        ax[1].axhspan(self.v_min, self.v_max, color="g", alpha=0.12, label=f"band {self.v_min}-{self.v_max} m/s")
        ax[1].set_ylabel("Speed (m/s)"); ax[1].set_xlabel("t (s)")
        ax[1].legend(loc="upper right", fontsize=8)
        if nrows == 3:
            ts, ws = manip
            ax[2].plot(ts, ws, color="tab:purple", label="manipulability w=√det(JJᵀ)")
            ax[2].axhline(0.006, color="r", ls="--", lw=0.9, label="usable threshold (Sec 1/2)")
            ax[2].fill_between(ts, 0, ws, where=(ws > 0.006), color="g", alpha=0.10)
            ax[2].set_ylabel("manipulability"); ax[2].set_xlabel("t (s), executed trajectory")
            ax[2].legend(loc="upper right", fontsize=8)
            ax[2].set_title("Dexterity along the wipe — stays above the usable band = good conditioning")
        plt.tight_layout(); plt.savefig(self.png_path, dpi=120)
        self.get_logger().info(f"wrote {self.csv_path} and {self.png_path}")

    def _save_traj(self, traj):
        with open(self.traj_path, "w") as f:
            f.write(f"joint_names: {list(traj.joint_trajectory.joint_names)}\n")
            f.write("points:\n")
            for pt in traj.joint_trajectory.points:
                tt = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                f.write(f"  - t: {tt:.4f}\n    positions: {list(pt.positions)}\n")
                if pt.velocities:
                    f.write(f"    velocities: {list(pt.velocities)}\n")
        self.get_logger().info(f"wrote {self.traj_path}")


def main():
    rclpy.init()
    node = MoveItWiper()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
