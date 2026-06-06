#!/usr/bin/env python3
"""Section 3 — force control via the ros2_control ADMITTANCE CONTROLLER (Harmonic).

The PROPER tool (vs a hand-rolled loop): `admittance_controller/AdmittanceController`
on gz_ros2_control. It reads a real gz force/torque sensor with built-in
gravity compensation, and makes the arm COMPLIANT (mass-damper-spring), so contact
is bounded — no slam, and no config-varying gravity-bias spikes.

The compliant contact force is set by how far the joint REFERENCE is pressed into the
surface (force = stiffness * penetration). Calibrated on this sim: ~660 N/m, so
10 N at ~15.5 mm and 6 N at ~10 mm of reference penetration.

This runner streams the Pilz coverage path's joint references, each pressed along the
tool approach axis (via the Jacobian) by the per-surface penetration, paced to the spec
velocity band. Logs force + velocity vs time.

  Run in the jazzy container with the Harmonic sim up (admittance_controller active):
    python3 admittance_controller_wipe.py
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
from trajectory_msgs.msg import JointTrajectoryPoint
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gr_wiping_control.kdl_chain import Kinematics

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TL = 0.02
TRAJ = "/home/dev/data/wiping_traj_continuous.yaml"
# calibrated reference penetration (m) for the per-surface target force
PEN_COUNTER = 0.0155     # ~10 N
PEN_MIRROR = 0.0100      # ~6 N
V_COUNTER, V_MIRROR = 0.20, 0.15
COUNTER_F, MIRROR_F = 10.0, 6.0


class AdmittanceCtrlWipe(Node):
    def __init__(self):
        super().__init__("admittance_controller_wipe")
        urdf = subprocess.check_output(
            ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
        ).decode()
        self.kin = Kinematics(urdf, "base_link", "link6")
        self.cur = {}
        self.F = np.zeros(3)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.create_subscription(WrenchStamped, "/tcp_fts_broadcaster/wrench", self._ft, 50)
        self.ref = self.create_publisher(JointTrajectoryPoint, "/admittance_controller/joint_references", 10)
        self.log = []
        self.start = time.time()

    def _js(self, m):
        self.cur.update(zip(m.name, m.position))

    def _ft(self, m):
        self.F = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])

    def _spin(self, dt):
        end = time.time() + max(dt, 0.0)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def _press(self, qn, tool_z, pen):
        """Joint reference pressed `pen` along the tool approach axis (Jacobian)."""
        Jac = self.kin.jacobian(qn)
        dq = np.linalg.pinv(Jac) @ np.concatenate([pen * tool_z, [0, 0, 0]])
        jp = JointTrajectoryPoint()
        jp.positions = [float(x) for x in (np.asarray(qn) + dq)]
        jp.velocities = [0.0] * 6
        self.ref.publish(jp)

    def _surface(self, pos, R):
        tool_z = R[:, 2]
        pad = pos + TL * tool_z
        if tool_z[2] < -0.7 and abs(pad[2] - 0.0) < 0.015:
            return "counter", PEN_COUNTER, V_COUNTER, COUNTER_F
        if tool_z[0] > 0.7 and abs(pad[0] - 0.44) < 0.015:
            return "mirror", PEN_MIRROR, V_MIRROR, MIRROR_F
        return None, 0.0, V_COUNTER, 0.0

    def run(self):
        d = yaml.safe_load(open(TRAJ))
        J = [p["positions"] for p in d["points"]]
        nominal = [self.kin.fk_pose(q) for q in J]
        t0 = time.time()
        while not all(j in self.cur for j in NAMES) and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"admittance_controller wipe: {len(J)} pts (10/6 N via compliant contact)")
        prev_pad = None
        for i, (pos, R) in enumerate(nominal):
            tool_z = R[:, 2]
            surf, pen, v_target, target = self._surface(pos, R)
            self._press(J[i], tool_z, pen)               # reference pressed for the target force
            pad = pos + TL * tool_z
            lat = float(np.linalg.norm(pad - prev_pad)) if prev_pad is not None else 0.0
            dur = float(np.clip(lat / v_target, 0.04, 0.5)) if lat > 1e-6 else 0.05
            self._spin(dur)
            Fn = float(abs(self.F[2]) if surf != "mirror" else abs(self.F[0]))
            v = lat / dur if lat > 1e-6 else 0.0
            mode = "CONTACT" if (surf and Fn > 2.0) else "APPROACH"
            self.log.append((time.time() - self.start, Fn, v, mode if surf else "TRANSIT", target))
            prev_pad = pad
        self.get_logger().info("done")
        self._save()

    def _save(self):
        a = np.array([(r[0], r[1], r[2]) for r in self.log], float)
        with open("/home/dev/data/admittance_ctrl_log.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t", "F_meas_N", "speed_mps", "mode", "target_N"])
            for r in self.log:
                w.writerow([f"{r[0]:.3f}", f"{r[1]:.3f}", f"{r[2]:.3f}", r[3], f"{r[4]:.1f}"])
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        ax[0].plot(a[:, 0], a[:, 1], lw=0.8, color="navy", label="|F| (ros2_control admittance, real gz F/T)")
        ax[0].axhspan(COUNTER_F - 2, COUNTER_F + 2, color="g", alpha=0.12, label="counter 10±2 N")
        ax[0].axhspan(MIRROR_F - 1.5, MIRROR_F + 1.5, color="b", alpha=0.12, label="mirror 6±1.5 N")
        ax[0].axhline(15, color="r", ls=":", lw=0.8, label="back-off 15 N")
        ax[0].set_ylabel("Force (N)"); ax[0].legend(fontsize=7, loc="upper right")
        ax[0].set_title("Section 3 — ros2_control AdmittanceController (Gazebo Harmonic, gravity-compensated)")
        ax[1].plot(a[:, 0], a[:, 2], color="tab:orange", lw=0.8, label="|v_tool|")
        ax[1].axhspan(0.15, 0.25, color="g", alpha=0.10, label="counter 0.15-0.25 m/s")
        ax[1].axhspan(0.10, 0.20, color="b", alpha=0.10, label="mirror 0.10-0.20 m/s")
        ax[1].set_ylim(0, 0.4); ax[1].set_ylabel("Speed (m/s)"); ax[1].set_xlabel("t (s)")
        ax[1].legend(fontsize=7, loc="upper right")
        plt.tight_layout(); plt.savefig("/home/dev/data/admittance_ctrl_log.png", dpi=120)
        self.get_logger().info("wrote admittance_ctrl_log.csv and admittance_ctrl_log.png")


def main():
    rclpy.init()
    n = AdmittanceCtrlWipe()
    try:
        n.run()
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
