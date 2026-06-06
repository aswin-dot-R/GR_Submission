#!/usr/bin/env python3
"""Replay the planned full-coverage wipe trajectory on the Gazebo Classic arm,
as smooth continuous motion (FollowJointTrajectory). Each cycle first eases from
the arm's CURRENT pose to the trajectory start (no 100-deg jump), then runs the wipe.
Used to visualize the primary (software-model) coverage in Gazebo with the paint trail.

  python3 replay_wipe.py [cycles]   (default 1; pass 0 for an endless loop)
"""
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
DEFAULT_TRAJ = "/home/dev/data/wiping_traj_continuous.yaml"


class Replay(Node):
    def __init__(self, traj=DEFAULT_TRAJ):
        super().__init__("replay_wipe")
        self.cur = {}
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.ac.wait_for_server(timeout_sec=15)
        self.pts = yaml.safe_load(open(traj))["points"]

    def _js(self, m):
        self.cur.update(zip(m.name, m.position))

    def _goal(self, pts, scale):
        jt = JointTrajectory(); jt.joint_names = NAMES; pv = -1.0
        for p in pts:
            t = float(p["t"]) * scale
            if t <= pv:
                t = pv + 0.05
            pv = t
            q = JointTrajectoryPoint(); q.positions = [float(x) for x in p["positions"]]
            q.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            jt.points.append(q)
        g = FollowJointTrajectory.Goal(); g.trajectory = jt
        return g, pv

    def _run(self, g, dur):
        gf = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, gf, timeout_sec=5)
        rf = gf.result().get_result_async()
        t0 = time.time()
        while time.time() - t0 < dur + 1:
            rclpy.spin_once(self, timeout_sec=0.05)
            if rf.done():
                break

    def play(self, cycles):
        i = 0
        while cycles == 0 or i < cycles:
            t0 = time.time()
            while not all(j in self.cur for j in NAMES) and time.time() - t0 < 3:
                rclpy.spin_once(self, timeout_sec=0.1)
            start = [self.cur[j] for j in NAMES]
            transit = [{"t": 0.0, "positions": start}, {"t": 3.0, "positions": self.pts[0]["positions"]}]
            g, d = self._goal(transit, 1.0); self._run(g, d)        # ease to start
            # Wipe slowly (scale 2.5): a faster sweep outruns the paint-trail's async
            # spawn service, so pad stamps get skipped and the swath looks patchy. At
            # this pace every pad-footprint marker renders as the arm passes.
            g, d = self._goal(self.pts, 2.5); self._run(g, d)       # the wipe
            i += 1


def main():
    # usage: replay_wipe.py [cycles] [trajectory_yaml]
    cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    traj = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_TRAJ
    rclpy.init()
    n = Replay(traj)
    try:
        n.play(cycles)
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
