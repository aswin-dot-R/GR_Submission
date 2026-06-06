"""Section 3 — Contact-aware wiping controller (Gazebo).

Closed loop:
  Gazebo (gazebo_ros_ft_sensor on joint6) --/wrist_ft--> controller
  controller --/arm_controller/joint_trajectory--> JointTrajectoryController --> Gazebo

Admittance via IK streaming (KDL): the tool follows a Cartesian wipe stroke on
the countertop; the surface-normal depth is regulated from the F/T error; each
tick we solve IK and stream a joint trajectory point to the arm_controller.

State machine (assignment spec):
  APPROACH : joint-space move above the stroke start, then descend
  CONTACT  : |Fn| > contact_threshold (2 N) -> force control engaged
             admittance regulates to target (counter 10 N / mirror 6 N)
             |Fn| > backoff_threshold (15 N) -> BACKOFF
  BACKOFF  : retract along +normal until |Fn| < contact_threshold, then resume
  Obstacle : within obstacle.skip_radius of the faucet -> lift the tool (skip)

Logs force & velocity vs time -> CSV + PNG on shutdown.
"""
import csv
import math
import os
import subprocess
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as Rot

from .kdl_chain import Kinematics


class Mode(Enum):
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    CONTACT = "CONTACT"
    BACKOFF = "BACKOFF"
    SKIP = "SKIP"
    DONE = "DONE"


def down_quat_xyzw(yaw):
    """Tool-down (z -> -world z) spun by yaw about vertical; quaternion (x,y,z,w)."""
    return (math.cos(yaw / 2.0), math.sin(yaw / 2.0), 0.0, 0.0)


def surface_tool_quat(n, u, v, yaw):
    """Quaternion (x,y,z,w) orienting the tool so its z-axis points -n (INTO the
    surface) with the x-axis at `yaw` about the normal, measured from in-plane
    axis u. Generalises down_quat_xyzw to any surface normal: it reproduces the
    counter orientation exactly for n=+z,u=+x,v=+y, and gives the mirror a
    tool-into-pane (+x) orientation for n=-x."""
    z_tool = -np.asarray(n, float)
    x_tool = math.cos(yaw) * np.asarray(u, float) + math.sin(yaw) * np.asarray(v, float)
    y_tool = np.cross(z_tool, x_tool)
    R = np.column_stack([x_tool, y_tool, z_tool])
    return Rot.from_matrix(R).as_quat()


class WipingController(Node):
    def __init__(self):
        super().__init__("wiping_controller")
        p = self.declare_parameter
        # ---- spec setpoints ----
        p("active_surface", "counter")
        p("counter.target_force_n", 10.0); p("counter.force_tol_n", 2.0)
        p("counter.speed_min_mps", 0.15); p("counter.speed_max_mps", 0.25)
        p("mirror.target_force_n", 6.0); p("mirror.force_tol_n", 1.5)
        p("mirror.speed_min_mps", 0.10); p("mirror.speed_max_mps", 0.20)
        p("contact_threshold_n", 2.0); p("backoff_threshold_n", 15.0)
        # ---- topics ----
        p("topics.ft_wrench", "/wrist_ft")
        p("topics.joint_states", "/joint_states")
        p("topics.joint_cmd", "/arm_controller/joint_trajectory")
        # ---- kinematics ----
        p("base_link", "base_link"); p("tip_link", "link6")
        p("tool_length_m", 0.04)         # link6 origin -> pad contact face (along link6 z)
        p("tool_yaw_deg", 150.0)          # yaw about normal (Section 1: 150 deg reaches well)
        # ---- area raster wipe (one boustrophedon pass over a reachable patch) ----
        p("wipe.center", [0.25, 0.00, 0.0])
        p("wipe.size_u", 0.20); p("wipe.size_v", 0.28)
        p("wipe.pad_swath", 0.05); p("wipe.overlap", 0.10)
        p("stroke.approach_z", 0.12)      # clearance above surface for approach
        # ---- mirror wipe region: its OWN params. The mirror is left exactly where
        # the scene/world place it (pane face x=0.44, z=0.30); we only choose a
        # reachable patch on that existing face (base frame; u=y, v=z). ----
        p("mirror.tool_yaw_deg", 0.0)
        p("mirror.wipe.center", [0.44, 0.00, 0.34])
        p("mirror.wipe.size_u", 0.16)     # along y (horizontal across the pane)
        p("mirror.wipe.size_v", 0.12)     # along z (vertical)
        # ---- gains / limits ----
        p("force.kp", 0.0005); p("force.ki", 0.00005); p("force.max_step_m", 0.004)
        p("descend_speed_mps", 0.03)
        p("control_rate_hz", 50.0)
        # ---- contact model ----
        # "graze"      : freeze penetration depth (kp=0) — pad rides the surface.
        # "admittance" : a virtual spring-mass-damper between force error and the
        #                commanded penetration. The wrist FT measures the contact
        #                force; the loop adjusts penetration in FINE sub-0.1mm steps
        #                so force rises gradually to target instead of slamming.
        #                  M*d'' + D*d' + K*d = (F_target - F_meas)
        #                K (spring) centres the depth, D (damper) kills oscillation
        #                against the stiff DART wall, M (virtual mass) smooths it.
        p("contact_model", "graze")
        p("admit.mass", 1.0)            # virtual mass  (N / (m/s^2))
        p("admit.damping", 120.0)       # virtual damper D (N / (m/s))
        p("admit.stiffness", 60.0)      # virtual spring K (N / m), small = better force tracking
        p("admit.max_rate_mps", 0.01)   # cap on penetration speed (anti-slam)
        p("admit.depth_min", -0.02); p("admit.depth_max", 0.02)
        # reference-style ASYMMETRIC admittance (contact_model:="asym"): creep IN with a
        # tiny capped advance, retreat FAST with a large capped pull-back, so it holds
        # force on a SOFT contact without slamming. delta = ka*err (advance) or
        # ka*err*mult (retreat). Pair with a soft contact (Gazebo Classic kp~5000) and a
        # low control rate (~20 Hz) so the per-cycle steps match the reference.
        p("asym.ka", 0.00001)               # gain, m per N per cycle
        p("asym.advance_max_m", 0.000005)   # max push-in per cycle (5 um)
        p("asym.retreat_max_m", 0.0003)     # max pull-back per cycle (300 um)
        p("asym.retreat_mult", 3.0)         # retreat is 3x more aggressive
        # ---- obstacle (faucet) ----
        p("obstacle.center", [0.40, 0.00]); p("obstacle.skip_radius_m", 0.08)
        p("obstacle.lift_m", 0.06)
        # ---- logging ----
        p("log.csv_path", "/home/dev/data/wiping_log.csv")
        p("log.png_path", "/home/dev/data/wiping_log.png")

        g = lambda n: self.get_parameter(n).value
        surf = g("active_surface")
        self.target_force = float(g(f"{surf}.target_force_n"))
        self.force_tol = float(g(f"{surf}.force_tol_n"))
        self.v_des = 0.5 * (float(g(f"{surf}.speed_min_mps")) + float(g(f"{surf}.speed_max_mps")))
        self.contact_th = float(g("contact_threshold_n"))
        self.backoff_th = float(g("backoff_threshold_n"))
        self.kp = float(g("force.kp")); self.ki = float(g("force.ki"))
        self.max_step = float(g("force.max_step_m"))
        # virtual spring-mass-damper contact model
        self.contact_model = str(g("contact_model"))
        self.adm_M = float(g("admit.mass")); self.adm_D = float(g("admit.damping"))
        self.adm_K = float(g("admit.stiffness"))
        self.adm_vmax = float(g("admit.max_rate_mps"))
        self.adm_dmin = float(g("admit.depth_min")); self.adm_dmax = float(g("admit.depth_max"))
        self.asym_ka = float(g("asym.ka")); self.asym_adv = float(g("asym.advance_max_m"))
        self.asym_ret = float(g("asym.retreat_max_m")); self.asym_mult = float(g("asym.retreat_mult"))
        self.adm_dvel = 0.0           # penetration rate state (m/s)
        self.descend_v = float(g("descend_speed_mps"))
        self.rate = float(g("control_rate_hz"))
        self.tool_len = float(g("tool_length_m"))
        # ---- surface frame: outward normal n (points toward the tool) + in-plane
        # axes (u, v). Counter is horizontal (n=+z, plane x-y); the mirror is the
        # vertical pane whose face normal points back at the arm (n=-x, plane y-z).
        # The mirror is NOT relocated — only a reachable patch on its face is wiped.
        if surf == "mirror":
            self.surf_n = np.array([-1.0, 0.0, 0.0])   # toward the arm
            self.surf_u = np.array([0.0, 1.0, 0.0])    # horizontal across pane (y)
            self.surf_v = np.array([0.0, 0.0, 1.0])    # vertical (z)
            self.yaw = math.radians(float(g("mirror.tool_yaw_deg")))
            wc = np.array(g("mirror.wipe.center"), float)
            wsu, wsv = float(g("mirror.wipe.size_u")), float(g("mirror.wipe.size_v"))
        else:  # counter
            self.surf_n = np.array([0.0, 0.0, 1.0])
            self.surf_u = np.array([1.0, 0.0, 0.0])
            self.surf_v = np.array([0.0, 1.0, 0.0])
            self.yaw = math.radians(float(g("tool_yaw_deg")))
            wc = np.array(g("wipe.center"), float)
            wsu, wsv = float(g("wipe.size_u")), float(g("wipe.size_v"))
        self.faucet_skip = (surf == "counter")   # faucet obstacle lives on the counter
        # build the raster polyline (in the surface plane) + arc-length table
        self.path = self._build_raster(
            wc, wsu, wsv, float(g("wipe.pad_swath")), float(g("wipe.overlap")))
        self._seg = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self._cum = np.concatenate([[0.0], np.cumsum(self._seg)])
        self.path_len = float(self._cum[-1])
        self.p_start = self.path[0]
        self.approach_z = float(g("stroke.approach_z"))
        self.obstacle = np.array(g("obstacle.center"), float)
        self.skip_r = float(g("obstacle.skip_radius_m"))
        self.lift_m = float(g("obstacle.lift_m"))
        self.csv_path = g("log.csv_path"); self.png_path = g("log.png_path")
        self.base = g("base_link"); self.tip = g("tip_link")

        # kinematics from the live URDF
        urdf = subprocess.check_output(
            ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
        ).decode()
        self.kin = Kinematics(urdf, self.base, self.tip)
        self.quat = surface_tool_quat(self.surf_n, self.surf_u, self.surf_v, self.yaw)

        self.q = None                     # current joint positions (measured)
        self.q_cmd = None                 # commanded joint vector (continuous reference)
        self.max_lin_step = self.v_des * 1.5 / self.rate   # m per tick (caps tool speed)
        self.max_ang_step = 0.15          # rad per tick
        self.max_dq = 0.05                # rad per joint per tick
        self.f_world = np.zeros(3)        # contact force in world frame (EMA)
        self.f_bias = None                # tare offset captured before contact
        self.mode = Mode.APPROACH
        self.depth = 0.0                  # commanded push depth below nominal (m)
        self.f_int = 0.0
        self.s = 0.0                      # stroke parameter [0,1]
        self.s_dir = 1.0
        self.pass_count = 0
        self.contact_z = self.tool_len    # link6 z at which the pad just touches z=0
        self.last_tip = None
        self.tip_vel = np.zeros(3)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.q_approach = None
        self.approach_sent = False
        self.approach_tick = 0
        self.ticks = 0
        self.max_ticks = int(120.0 * self.rate)   # 120 s hard cap
        self.log = []

        # admittance mode: stream the motion to the admittance_controller's joint
        # reference + set a target contact wrench (it does the compliant force
        # holding). Otherwise stream a JointTrajectory to the JTC (graze demo).
        self.declare_parameter("use_admittance", True)
        self.use_adm = bool(self.get_parameter("use_admittance").value)
        # admittance translational stiffness (must match wiping_controllers_gz.yaml).
        # Force-holding = command the reference to penetrate by target_force/stiffness;
        # the soft admittance yields, giving a controlled contact force without slam.
        self.declare_parameter("admittance.stiffness", 200.0)
        self.adm_stiffness = float(self.get_parameter("admittance.stiffness").value)
        self.declare_parameter("topics.joint_ref", "/admittance_controller/joint_references")
        self.declare_parameter("topics.wrench_ref", "/admittance_controller/wrench_reference")

        self.sub_ft = self.create_subscription(WrenchStamped, g("topics.ft_wrench"), self._ft_cb, 50)
        self.sub_js = self.create_subscription(JointState, g("topics.joint_states"), self._js_cb, 50)
        if self.use_adm:
            self.pub = self.create_publisher(JointTrajectoryPoint, g("topics.joint_ref"), 10)
            self.pub_wrench = self.create_publisher(WrenchStamped, g("topics.wrench_ref"), 10)
        else:
            self.pub = self.create_publisher(JointTrajectory, g("topics.joint_cmd"), 10)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"wiping_controller up (surface={surf}, F*={self.target_force}±{self.force_tol} N, "
            f"v={self.v_des:.2f} m/s)"
        )

    # ---------- callbacks ----------
    def _js_cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            self.q = np.array([msg.position[idx[n]] for n in self.kin.joint_names])
        except KeyError:
            pass

    def _ft_cb(self, msg: WrenchStamped):
        if self.q is None:
            return
        f_local = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        _, R = self.kin.fk_pose(self.q)         # link6 orientation in base
        f_world = R @ f_local
        a = 0.2
        self.f_world = (1 - a) * self.f_world + a * f_world

    # ---------- helpers ----------
    def _normal_force(self):
        """Contact force along the surface normal, tare-corrected (counter +z,
        mirror -x)."""
        f = self.f_world - (self.f_bias if self.f_bias is not None else np.zeros(3))
        return abs(float(np.dot(f, self.surf_n)))

    def _admittance_step(self, fn):
        """Virtual spring-mass-damper between force error and penetration depth:
            M*d'' + D*d' + K*d = (F_target - F_meas)
        Integrated each tick with a capped penetration rate, so the commanded
        depth eases toward the target force instead of slamming the (rigid DART)
        contact. The damper D suppresses oscillation against the stiff wall; the
        small spring K bounds the depth when contact is momentarily lost; the
        steady force offset is K*depth (depth is sub-mm, so ~0). Returns depth."""
        e = self.target_force - fn
        dt = 1.0 / self.rate
        a = (e - self.adm_D * self.adm_dvel - self.adm_K * self.depth) / max(1e-6, self.adm_M)
        self.adm_dvel = max(-self.adm_vmax, min(self.adm_vmax, self.adm_dvel + a * dt))
        d = self.depth + self.adm_dvel * dt
        return max(self.adm_dmin, min(self.adm_dmax, d))

    def _asym_step(self, fn):
        """Reference asymmetric admittance: advance (push in) capped tiny & slow,
        retreat (pull back) capped large & fast. Holds force on a soft contact
        without slamming. depth = penetration (higher = press harder); floored at 0
        so the pad can't be commanded above the surface during the hold."""
        e = self.target_force - fn
        if e < 0.0:    # too much force -> retreat (depth decreases), aggressive
            d = max(-self.asym_ret, self.asym_ka * e * self.asym_mult)
        else:          # too little force -> advance (depth increases), gentle
            d = min(self.asym_adv, self.asym_ka * e)
        return max(0.0, min(self.adm_dmax, self.depth + d))

    def _build_raster(self, center, size_u, size_v, pad_swath, overlap):
        """Boustrophedon raster over a size_u x size_v patch centred at `center`,
        laid in the surface plane (u = surf_u stroke direction, v = surf_v step
        direction), one pass. Rows are spaced by pad_swath*(1-overlap); each row
        is densified for smooth following."""
        hu, hv = size_u / 2.0, size_v / 2.0
        pitch = max(1e-3, pad_swath * (1.0 - overlap))
        n_rows = max(1, int(math.ceil(2 * hv / pitch)) + 1)
        vs = np.linspace(-hv, hv, n_rows)
        pts = []
        for i, v in enumerate(vs):
            us = (-hu, hu) if i % 2 == 0 else (hu, -hu)
            for u in us:
                pts.append(center + u * self.surf_u + v * self.surf_v)
        # densify so following is smooth
        dense = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            seg = float(np.linalg.norm(b - a))
            n = max(1, int(math.ceil(seg / 0.01)))
            for k in range(1, n + 1):
                dense.append(a + (b - a) * (k / n))
        return np.asarray(dense)

    def _path_point(self, s):
        """Point on the raster polyline at arc-length s (clamped)."""
        s = max(0.0, min(self.path_len, s))
        i = int(np.searchsorted(self._cum, s) - 1)
        i = max(0, min(i, len(self._seg) - 1))
        seg = self._seg[i]
        f = 0.0 if seg < 1e-9 else (s - self._cum[i]) / seg
        return self.path[i] + (self.path[i + 1] - self.path[i]) * f

    def _near_faucet(self, xy):
        if not self.faucet_skip:        # faucet obstacle only applies on the counter
            return False
        return np.linalg.norm(xy[:2] - self.obstacle) < self.skip_r

    def _send(self, q, dt):
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start = Duration(sec=int(dt), nanosec=int((dt - int(dt)) * 1e9))
        if self.use_adm:
            self.pub.publish(pt)            # admittance reference (motion to follow)
        else:
            jt = JointTrajectory()
            jt.joint_names = list(self.kin.joint_names)
            jt.points = [pt]
            self.pub.publish(jt)

    def _send_wrench(self, fmag):
        """Target contact force along the surface normal (base frame). For the
        counter this presses down (+z); for the mirror it presses into the pane
        (-x). The admittance_controller drives the arm to it."""
        if not self.use_adm:
            return
        w = WrenchStamped()
        w.header.frame_id = self.base
        fvec = float(fmag) * self.surf_n
        w.wrench.force.x = float(fvec[0])
        w.wrench.force.y = float(fvec[1])
        w.wrench.force.z = float(fvec[2])
        self.pub_wrench.publish(w)

    def _ready_seed(self, pos):
        """A forward, elbow-down posture seed so IK doesn't pick a backward branch
        (joint1 ~ atan2(y, x) points the base toward the target)."""
        return np.array([math.atan2(pos[1], pos[0]), 0.9, -0.9, 0.0, 0.9, 0.0])

    def _diff_track(self, target_pos):
        """Differential IK: nudge the *commanded* joint vector toward the target
        pose (tool-down) with a clamped Cartesian step, so the streamed command is
        continuous (no per-tick IK branch jumps -> no velocity spikes). Caps tool
        speed via the linear-twist clamp. Returns the updated command and sends it."""
        if self.q_cmd is None:
            self.q_cmd = self.q.copy()
        tw = self.kin.cart_diff(self.q_cmd, target_pos, self.quat)
        lin = tw[:3]; ang = tw[3:]
        ln = float(np.linalg.norm(lin)); an = float(np.linalg.norm(ang))
        if ln > self.max_lin_step:
            lin = lin * (self.max_lin_step / ln)
        if an > self.max_ang_step:
            ang = ang * (self.max_ang_step / an)
        dq = self.kin.vel_ik(self.q_cmd, np.concatenate([lin, ang]))
        dq = np.clip(dq, -self.max_dq, self.max_dq)
        self.q_cmd = self.q_cmd + dq
        self._send(self.q_cmd, 2.0 / self.rate)
        return self.q_cmd

    def _ik(self, pos, seed):
        """Solve IK trying both the continuity seed and a forward ready seed; keep
        the reachable solution that actually hits the target and keeps the base
        pointing forward (rejects the flipped branch that swings the arm away)."""
        j1ref = math.atan2(pos[1], pos[0])
        best, best_score = None, 1e9
        for s in (seed, self._ready_seed(pos)):
            ok, q = self.kin.ik_pose(s, pos, self.quat)
            if not ok:
                continue
            fk, _ = self.kin.fk_pose(q)
            err = float(np.linalg.norm(fk - pos))
            branch = abs(((q[0] - j1ref + math.pi) % (2 * math.pi)) - math.pi)
            score = err + 0.05 * branch
            if err < 0.02 and score < best_score:
                best, best_score = q, score
        return (best is not None), (best if best is not None else seed)

    # ---------- main loop ----------
    def _tick(self):
        if self.q is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        tip, _ = self.kin.fk_pose(self.q)
        if self.last_tip is not None:
            self.tip_vel = (tip - self.last_tip) * self.rate
        self.last_tip = tip
        fn = self._normal_force()
        prev = self.mode

        if self.mode == Mode.APPROACH:
            # one-shot: go above the stroke start, tool down
            if not self.approach_sent:
                above = self.p_start + (self.approach_z + self.tool_len) * self.surf_n
                ok, qa = self._ik(above, self.q)
                if ok:
                    self.q_approach = qa
                    self._send(qa, 3.0)
                    self.approach_sent = True
                    self.approach_tick = self.ticks
            elif self.q_approach is not None:
                # effort+PID won't track exactly; proceed on relaxed tolerance OR timeout
                reached = np.max(np.abs(self.q - self.q_approach)) < 0.12
                timed_out = (self.ticks - self.approach_tick) > int(5.0 * self.rate)
                if reached or timed_out:
                    self.f_bias = self.f_world.copy()   # tare before contact
                    self.q_cmd = self.q.copy()          # seed differential tracking
                    self.mode = Mode.DESCEND

        elif self.mode == Mode.DESCEND:
            # lower the tool gradually, but only until the pad just meets the
            # surface (link6 at z = tool_len). Descending into a penetrating floor
            # punches the pad deep into the 4 cm slab where ODE fails to form a
            # clean contact — so we stop at the surface and let CONTACT's admittance
            # establish force from there. Switch on force OR on reaching the surface.
            self.approach_z = max(0.0, self.approach_z - self.descend_v / self.rate)
            target = self.p_start + (self.approach_z + self.tool_len) * self.surf_n
            self._diff_track(target)
            if fn > self.contact_th or self.approach_z <= 0.001:
                self.contact_z = self.tool_len   # known geometric contact height
                self.depth = 0.0; self.f_int = 0.0; self.adm_dvel = 0.0
                self.mode = Mode.CONTACT

        elif self.mode in (Mode.CONTACT, Mode.SKIP, Mode.BACKOFF):
            # advance arc-length along the raster polyline (single pass)
            self.s += self.v_des / self.rate
            if self.s >= self.path_len:
                self.mode = Mode.DONE
            xy = self._path_point(self.s)

            if self.use_adm:
                # Force-holding via the admittance controller: command the reference
                # to PENETRATE by target_force/stiffness. The soft admittance yields
                # to a controlled contact force (no slam). Lift over the faucet.
                if self._near_faucet(xy):
                    self.mode = Mode.SKIP; self.depth = -self.lift_m
                else:
                    self.mode = Mode.CONTACT
                    self.depth = self.target_force / self.adm_stiffness
            elif self._near_faucet(xy):
                # obstacle: lift the tool over the faucet, suspend force control
                self.mode = Mode.SKIP
                self.depth = -self.lift_m
                self.f_int = 0.0
            elif fn > self.backoff_th:
                # too hard: ease off one step but keep moving (transient back-off)
                self.mode = Mode.BACKOFF
                self.depth = max(-0.02, self.depth - self.max_step)
                self.f_int = 0.0; self.adm_dvel = 0.0
            elif self.contact_model == "asym":
                # reference asymmetric admittance (creep in, retreat fast) on soft contact
                self.mode = Mode.CONTACT
                self.depth = self._asym_step(fn)
            elif self.contact_model == "admittance":
                # spring-mass-damper force regulation (measures FT, holds target)
                self.mode = Mode.CONTACT
                self.depth = self._admittance_step(fn)
            else:
                self.mode = Mode.CONTACT
                # graze / PI on depth (kp=0 -> frozen depth, pad rides the surface)
                err = self.target_force - fn
                self.f_int += err / self.rate
                d = self.kp * err + self.ki * self.f_int
                d = max(-self.max_step, min(self.max_step, d))
                self.depth = max(-self.lift_m, min(0.01, self.depth + d))

            pos = xy + (self.contact_z - self.depth) * self.surf_n
            self._diff_track(pos)

        # In admittance mode the press force is handled by the admittance_controller
        # via the wrench reference (not the manual depth above): target force while
        # wiping, zero while approaching.
        if self.mode == Mode.CONTACT:
            self._send_wrench(self.target_force)    # desired contact force (FT reads +z up)
        else:
            self._send_wrench(0.0)                  # no press while approaching/lifting

        if prev != self.mode:
            self.get_logger().info(f"{prev.value} -> {self.mode.value} (|Fn|={fn:.2f} N)")

        t = now - self.t0
        self.log.append((t, self.f_world[0], self.f_world[1], self.f_world[2], fn,
                         self.tip_vel[0], self.tip_vel[1], self.tip_vel[2],
                         float(np.linalg.norm(self.tip_vel)), self.mode.value, self.depth))

        self.ticks += 1
        if self.ticks > self.max_ticks and self.mode != Mode.DONE:
            self.get_logger().warn("hit time cap — finishing")
            self.mode = Mode.DONE

        if self.mode == Mode.DONE:
            self.get_logger().info("wipe complete — writing log")
            self.shutdown()
            self.timer.cancel()
            rclpy.shutdown()

    # ---------- output ----------
    def shutdown(self):
        if not self.log:
            return
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t", "fx", "fy", "fz", "fn", "vx", "vy", "vz", "speed", "mode", "depth"])
            for r in self.log:
                wr.writerow([f"{r[0]:.4f}"] + [f"{v:.4f}" for v in r[1:9]] + [r[9], f"{r[10]:.5f}"])
        a = np.array([(r[0], r[4], r[8]) for r in self.log], float)
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        ax[0].plot(a[:, 0], a[:, 1], label="|Fn| (normal force)")
        ax[0].axhline(self.target_force, color="g", ls="--", lw=0.8, label="target")
        ax[0].axhline(self.target_force + self.force_tol, color="g", ls=":", lw=0.6)
        ax[0].axhline(self.target_force - self.force_tol, color="g", ls=":", lw=0.6)
        ax[0].axhline(self.contact_th, color="b", ls=":", lw=0.8, label="contact 2N")
        ax[0].axhline(self.backoff_th, color="r", ls=":", lw=0.8, label="back-off 15N")
        ax[0].set_ylabel("Force (N)"); ax[0].legend(loc="upper right", fontsize=8)
        ax[0].set_title("Contact-aware wiping — force & velocity tracking")
        ax[1].plot(a[:, 0], a[:, 2], label="|v_tool|")
        ax[1].axhspan(self.v_des * 0.0, self.v_des, color="g", alpha=0.0)
        ax[1].set_ylabel("Speed (m/s)"); ax[1].set_xlabel("t (s)")
        ax[1].legend(loc="upper right", fontsize=8)
        plt.tight_layout(); plt.savefig(self.png_path, dpi=120)
        self.get_logger().info(f"wrote {self.csv_path} and {self.png_path}")


def main():
    rclpy.init()
    node = WipingController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.shutdown()
        except Exception:
            pass
        node.destroy_node()


if __name__ == "__main__":
    main()
