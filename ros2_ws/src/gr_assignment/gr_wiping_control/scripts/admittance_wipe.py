#!/usr/bin/env python3
"""Section 3 — LIVE admittance force control on the Gazebo Classic sim.

Runs the Pilz-planned coverage path (one continuous trajectory) and, on each
control step, reads the REAL wrist F/T sensor (/wrist_ft) and regulates the
penetration along the tool approach axis to hold the per-surface target force:

  switch to force control when |F| > contact_threshold (2 N)
  admittance:  press_depth += k_a * (F_target - F_meas)      (asymmetric:
               retract faster than advance, like the reference impl)
  back off when |F| > backoff_threshold (15 N)

The force sensor reading is gravity-tared (the free-space reading is tracked
during the lifted transitions and subtracted) and low-pass filtered. Commands
go to the JointTrajectoryController via streamed IK (KDL). Logs + plots force
vs time and velocity vs time.

  ros2 run is not needed; run in the humble container with the Classic sim up:
    python3 admittance_wipe.py
"""
import subprocess
import time
import csv

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gr_wiping_control.kdl_chain import Kinematics

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TL = 0.02                       # link6 -> pad contact face
TRAJ = "/home/dev/data/wiping_traj_continuous.yaml"

# spec setpoints
F_CONTACT = 2.0                 # |F| > 2 N -> force control
F_BACKOFF = 15.0                # |F| > 15 N -> back off
COUNTER_F, MIRROR_F = 10.0, 6.0
# spec velocity bands (m/s): pace each wipe step so |v_tool| stays in-band
COUNTER_V = 0.20                # counter 0.15-0.25 m/s -> aim mid-band
MIRROR_V = 0.15                 # mirror 0.10-0.20 m/s
# contact model: the soft ODE surface is ~kp=5000 N/m, so target 10 N needs only
# ~2 mm penetration. The admittance is a damped law on penetration:
#   pen += (1/D)*(F_target - F_meas)*dt,  clamped so |F| can't exceed the back-off.
# measured on this sim: pad touches ~4 mm below the nominal pose, then ~5000 N/m
# (10 N at ~6 mm press). So pen ranges from a small lift to ~8 mm (≈ the 15 N cap).
D_ADM = 1500.0                  # admittance damping (N*s/m): pen rate ~ F_err / D_ADM
# The nominal Pilz path digs the pad in at some configs (slam). So LIFT the whole
# path by LIFT_BASE and let the admittance press from there: commanded press along
# the tool axis = pen - LIFT_BASE. pen in [0, PEN_MAX]; pen=0 -> pad lifted clear,
# pen=LIFT_BASE -> pad at the nominal surface, pen>LIFT_BASE -> pressing in.
LIFT_BASE = 0.008
PEN_MAX = LIFT_BASE + 0.008     # press cap (~15 N at the measured stiffness)
PEN_MIN = 0.0                   # fully lifted


class AdmittanceWipe(Node):
    def __init__(self):
        super().__init__("admittance_wipe")
        urdf = subprocess.check_output(
            ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
        ).decode()
        self.kin = Kinematics(urdf, "base_link", "link6")
        self.cur = {}
        self.fz_raw = 0.0
        self.fz_filt = 0.0
        self.bias = 0.0
        self.free_buf = []          # recent free-space raw readings (rolling gravity tare)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.create_subscription(WrenchStamped, "/wrist_ft", self._ft, 50)
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.log = []
        self.start = time.time()

    def _js(self, m):
        self.cur.update(zip(m.name, m.position))

    def _ft(self, m):
        # force along the tool approach axis = link6 z component
        self.fz_raw = m.wrench.force.z
        # low-pass (gravity-tared magnitude)
        f = abs(self.fz_raw - self.bias)
        self.fz_filt = 0.8 * self.fz_filt + 0.2 * f

    def _spin(self, dt):
        end = time.time() + max(dt, 0.0)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def _send(self, q, dur):
        jt = JointTrajectory(); jt.joint_names = NAMES
        p = JointTrajectoryPoint(); p.positions = [float(x) for x in q]
        p.time_from_start = Duration(sec=int(dur), nanosec=int((dur - int(dur)) * 1e9))
        jt.points = [p]; self.pub.publish(jt)

    def _press(self, qn, tool_z, pen, dur=0.08):
        """Command the nominal joint config qn pressed `pen` along the tool approach
        axis, via the Jacobian (reliable local correction): dq = J^+ * [pen*tool_z; 0]."""
        Jac = self.kin.jacobian(qn)
        twist = np.concatenate([pen * tool_z, [0.0, 0.0, 0.0]])
        dq = np.linalg.pinv(Jac) @ twist
        self._send(np.asarray(qn) + dq, dur)

    def _surface(self, pos, R):
        """Classify a nominal pose: ('counter'|'mirror'|None, target_force).
        Contact only when the pad is within 1 cm of that surface; else free space."""
        tool_z = R[:, 2]
        pad = pos + TL * tool_z
        if tool_z[2] < -0.7 and abs(pad[2] - 0.0) < 0.012:      # tool down, near counter top
            return "counter", COUNTER_F
        if tool_z[0] > 0.7 and abs(pad[0] - 0.44) < 0.012:          # tool +x, near mirror face
            return "mirror", MIRROR_F
        return None, 0.0

    def run(self):
        d = yaml.safe_load(open(TRAJ))
        J = [p["positions"] for p in d["points"]]
        # nominal link6 poses
        nominal = [self.kin.fk_pose(q) for q in J]
        # wait for state + sensor
        t0 = time.time()
        while not all(j in self.cur for j in NAMES) and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        seed = [self.cur[j] for j in NAMES]
        # The Pilz nominal path already places the pad ON the surface, so it makes
        # contact as it wipes. `pen` MODULATES that contact about the nominal:
        #   pen > 0 -> press deeper;  pen < 0 -> lift off.
        # The admittance is a damped law that drives pen so |F| holds the target;
        # if |F| > back-off it lifts fully clear. pen is clamped so the press can't
        # exceed ~the back-off force (PEN_MAX) and can fully retract (PEN_MIN).
        pen = 0.0                    # regulation var; press = pen - LIFT_BASE
        dt = 0.05
        prev_pad = None
        prev_surf = None
        contacts = 0
        backoff_until = -1
        self.get_logger().info(f"live admittance wipe: {len(J)} nominal pts; "
                               f"targets counter {COUNTER_F} N / mirror {MIRROR_F} N")
        for i, (pos, R) in enumerate(nominal):
            tool_z = R[:, 2]
            surf, target = self._surface(pos, R)
            F = self.fz_filt
            # ---- GUARDED SLOW APPROACH on ENTERING a contact region ----
            # The path enters contact too fast, so the pad would slam (30-45 N spike).
            # Instead, lift the pad clear at the entry point and descend SLOWLY (0.4 mm
            # per step) until it just touches (|F| > 2 N), THEN start the wipe.
            if surf is not None and prev_surf is None:
                pen = 0.0                               # pad lifted clear at the entry
                for _ in range(60):
                    self._press(J[i], tool_z, pen - LIFT_BASE)
                    self._spin(0.05)
                    fa = self.fz_filt
                    self.log.append((time.time() - self.start, fa, 0.0, "APPROACH", pen))
                    if fa > F_CONTACT:                  # contact -> stop descending
                        break
                    pen = min(PEN_MAX, pen + 0.0004)    # slow descent
                F = self.fz_filt
            # ---- per-step control ----
            if surf is None:
                # FREE SPACE (lifted transitions): track gravity-tare baseline, lift clear
                self.free_buf.append(self.fz_raw)
                self.free_buf = self.free_buf[-15:]
                self.bias = float(np.median(self.free_buf))
                pen = max(PEN_MIN, pen - 0.001)
                mode = "APPROACH"
            elif F > F_BACKOFF or i < backoff_until:
                # too hard -> retract fast until light again
                if F > F_BACKOFF:
                    backoff_until = i + 12
                mode = "BACKOFF"; pen = max(PEN_MIN, pen - 0.002)
            else:
                # force control: damped admittance modulating the path's contact
                mode = "CONTACT" if F > F_CONTACT else "APPROACH"
                if F > F_CONTACT:
                    contacts += 1
                pen = float(np.clip(pen + (target - F) / D_ADM * dt, PEN_MIN, PEN_MAX))
            # VELOCITY CONTROL: pace this step by the spec target speed for the surface,
            # so |v_tool| = lateral_dist / dur stays inside the band (counter 0.15-0.25,
            # mirror 0.10-0.20 m/s).
            pad = pos + TL * tool_z
            v_target = MIRROR_V if surf == "mirror" else COUNTER_V
            lat = float(np.linalg.norm(pad - prev_pad)) if prev_pad is not None else 0.0
            dur = float(np.clip(lat / v_target, 0.04, 0.5)) if lat > 1e-6 else 0.05
            self._press(J[i], tool_z, pen - LIFT_BASE, dur + 0.02)
            v = lat / dur if lat > 1e-6 else 0.0
            prev_pad = pad
            prev_surf = surf
            self.log.append((time.time() - self.start, F, v, mode, pen))
            self._spin(dur)
        self.get_logger().info(f"done: {contacts} contact steps")
        self._save()

    def _save(self):
        a = np.array([(r[0], r[1], r[2]) for r in self.log], float)
        with open("/home/dev/data/admittance_log.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t", "F_meas_N", "speed_mps", "mode", "depth_m"])
            for r in self.log:
                w.writerow([f"{r[0]:.3f}", f"{r[1]:.3f}", f"{r[2]:.3f}", r[3], f"{r[4]:.5f}"])
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        ax[0].plot(a[:, 0], a[:, 1], lw=0.8, label="|F| measured (live /wrist_ft, gravity-tared)")
        ax[0].axhspan(COUNTER_F - 2, COUNTER_F + 2, color="g", alpha=0.10, label="counter 10±2 N")
        ax[0].axhspan(MIRROR_F - 1.5, MIRROR_F + 1.5, color="b", alpha=0.10, label="mirror 6±1.5 N")
        ax[0].axhline(F_CONTACT, color="orange", ls=":", lw=0.8, label="contact 2 N")
        ax[0].axhline(F_BACKOFF, color="r", ls=":", lw=0.8, label="back-off 15 N")
        ax[0].set_ylabel("Force (N)"); ax[0].legend(fontsize=7, loc="upper right")
        ax[0].set_title("Section 3 — LIVE admittance force control (Gazebo Classic, real F/T)")
        ax[1].plot(a[:, 0], a[:, 2], color="tab:orange", lw=0.8, label="|v_tool|")
        ax[1].axhspan(0.15, 0.25, color="g", alpha=0.10, label="counter band 0.15-0.25 m/s")
        ax[1].axhspan(0.10, 0.20, color="b", alpha=0.10, label="mirror band 0.10-0.20 m/s")
        ax[1].set_ylim(0, 0.4)
        ax[1].set_ylabel("Speed (m/s)"); ax[1].set_xlabel("t (s)"); ax[1].legend(fontsize=7, loc="upper right")
        plt.tight_layout(); plt.savefig("/home/dev/data/admittance_log.png", dpi=120)
        self.get_logger().info("wrote admittance_log.csv and admittance_log.png")


def main():
    rclpy.init()
    n = AdmittanceWipe()
    try:
        n.run()
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
