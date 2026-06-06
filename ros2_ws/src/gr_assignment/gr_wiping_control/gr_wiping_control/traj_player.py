"""Replay a saved joint-trajectory YAML on a JointTrajectoryController by STREAMING
setpoints (publish each point as a 1-point trajectory with a short lookahead, pacing
by the trajectory's own timing). A one-shot trajectory (whole thing in a single
publish or a FollowJointTrajectory goal) is silently ignored by the gz_ros2_control
JTC here, but streaming setpoints drives it (same as the live controller). Used to
play the MoveIt-planned wipe in Gazebo for a smooth counter->mirror demo.

Usage:
  ros2 run gr_wiping_control traj_player --ros-args \
      -p file:=/home/dev/data/wiping_trajectory.yaml \
      -p topic:=/arm_controller/joint_trajectory -p transit_s:=3.0 -p time_scale:=1.5
"""
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajPlayer(Node):
    def __init__(self):
        super().__init__("traj_player")
        self.declare_parameter("file", "")
        self.declare_parameter("topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("transit_s", 3.0)
        self.declare_parameter("time_scale", 1.0)   # >1 = slower (easier to watch)
        self.declare_parameter("lookahead_s", 0.15)  # JTC setpoint lookahead per stream tick
        self.path = self.get_parameter("file").value
        self.topic = self.get_parameter("topic").value
        self.transit = float(self.get_parameter("transit_s").value)
        self.scale = float(self.get_parameter("time_scale").value)
        self.look = float(self.get_parameter("lookahead_s").value)
        self.cur = {}
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.pub = self.create_publisher(JointTrajectory, self.topic, 10)

    def _js(self, m):
        self.cur = dict(zip(m.name, m.position))

    def _spin_sleep(self, dt):
        end = time.time() + max(dt, 0.0)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def _send(self, names, positions, t_from_start):
        jt = JointTrajectory()
        jt.joint_names = list(names)
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in positions]
        p.time_from_start = Duration(sec=int(t_from_start),
                                     nanosec=int((t_from_start - int(t_from_start)) * 1e9))
        jt.points = [p]
        self.pub.publish(jt)

    def play(self):
        d = yaml.safe_load(open(self.path))
        names = d["joint_names"]
        pts = d.get("points", []) or []
        if len(pts) < 2:
            self.get_logger().error(f"{self.path}: no trajectory points")
            return 0.0
        # wait for a joint-state sample (so the transit starts from the real pose)
        t0 = time.time()
        while not all(n in self.cur for n in names) and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        total = pts[-1]["t"] * self.scale
        self.get_logger().info(f"streaming {self.path}: {len(pts)} pts over ~{total:.1f}s")
        # transit: ease from the current pose to the stroke start
        if all(n in self.cur for n in names):
            start = [self.cur[n] for n in names]
            steps = max(1, int(self.transit / 0.05))
            for k in range(1, steps + 1):
                a = k / steps
                q = [start[i] + a * (pts[0]["positions"][i] - start[i]) for i in range(len(names))]
                self._send(names, q, self.look)
                self._spin_sleep(self.transit / steps)
        # stream the trajectory points, pacing by their own (scaled) timing
        prev = 0.0
        for p in pts:
            dt = max(0.0, (p["t"] - prev)) * self.scale
            self._send(names, p["positions"], max(dt, self.look))
            self._spin_sleep(max(dt, 0.02))
            prev = p["t"]
        self._spin_sleep(0.3)
        self.get_logger().info("stream finished")
        return total


def main():
    rclpy.init()
    node = TrajPlayer()
    try:
        node.play()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
