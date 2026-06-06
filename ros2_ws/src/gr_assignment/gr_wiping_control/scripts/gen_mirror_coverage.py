#!/usr/bin/env python3
"""Generate a FULL-reachable-region mirror coverage joint trajectory for the Gazebo
demo — covering everything Section 2 marks reachable on the mirror, including the
bottom-left / bottom-right corners that a single compute_cartesian_path stroke can't
reach (the pane sits at the workspace edge, so continuous strokes break, but discrete
per-waypoint IK succeeds).

Strategy (same as a coverage mask, not a Cartesian stroke):
  - raster the reachable mirror face (y x z) at the pad pitch, boustrophedon order;
  - per-waypoint collision-aware /compute_ik, SEEDED from the previous solution so
    consecutive joint configs stay close (smooth motion, pad stays near the surface);
  - skip waypoints with no IK (faucet centre column below the faucet top, far corners);
  - when two kept waypoints are far apart in Cartesian space (the faucet gap), bridge
    them through a RETRACTED via so the arm lifts off and never scrapes the faucet;
  - write a joint trajectory the Gazebo replay can stream.

Run in the jazzy container (needs move_group up):
  python3 gen_mirror_coverage.py
Writes /home/dev/data/p_mirror_full.yaml
"""
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from gr_wiping_control.kdl_chain import Kinematics

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TL = 0.02
FACE_X = 0.414                      # link6 x so the pad tip sits ~on the pane (x=0.44 - standoff)
# mirror reachable raster (matches the mapped region / Section-2 mirror extent)
YS = np.round(np.arange(-0.19, 0.221, 0.03), 3)
ZS = np.round(np.arange(0.08, 0.481, 0.03), 3)
# tool z -> +x quaternion (pad approach axis points at the pane)
QUAT = (0.0, 0.7071, 0.0, 0.7071)   # (x, y, z, w)


class Gen(Node):
    def __init__(self):
        super().__init__("gen_mirror_cov")
        urdf = subprocess.check_output(
            ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
        ).decode()
        self.kin = Kinematics(urdf, "base_link", "link6")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.ik.wait_for_service(timeout_sec=10)

    def solve(self, y, z, seed):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = FACE_X
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = QUAT
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = "arm"
        r.pose_stamped = ps
        r.timeout.sec = 1
        r.avoid_collisions = True
        if seed is not None:
            rs = RobotState()
            js = JointState()
            js.name = NAMES
            js.position = [float(x) for x in seed]
            rs.joint_state = js
            r.robot_state = rs
        req.ik_request = r
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=4)
        res = f.result()
        if not res or res.error_code.val != 1:
            return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in NAMES]

    def pad_clear(self, q):
        """True if the wiping pad box (100x50x20 mm, 1 cm past link6) clears the FAUCET.
        MoveIt's URDF has no pad (tool_link) so it can't see pad<->faucet contact; the low
        mirror rows sit just above the faucet, so check the pad box explicitly."""
        p, R = self.kin.fk_pose(q)
        c = p + 0.01 * R[:, 2]
        hx, hy, hz = 0.05, 0.025, 0.01
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    P = c + sx * hx * R[:, 0] + sy * hy * R[:, 1] + sz * hz * R[:, 2]
                    if 0.37 <= P[0] <= 0.43 and -0.03 <= P[1] <= 0.03 and 0.0 <= P[2] <= 0.15:
                        return False
        return True

    def retract(self, q, dist=0.10):
        Jac = self.kin.jacobian(q)
        tz = self.kin.fk_pose(q)[1][:, 2]
        twist = np.concatenate([-dist * tz, [0, 0, 0]])
        return (np.array(q) + np.linalg.pinv(Jac) @ twist).tolist()

    def cart(self, q):
        p, R = self.kin.fk_pose(q)
        return p + TL * R[:, 2]

    def run(self):
        # collect reachable waypoints in boustrophedon order
        kept = []
        seed = None
        for i, z in enumerate(ZS):
            row_ys = YS if i % 2 == 0 else YS[::-1]
            for y in row_ys:
                q = self.solve(y, z, seed)
                if q is not None and self.pad_clear(q):     # skip cells where the pad hits the faucet
                    kept.append(q)
                    seed = q
        self.get_logger().info("reachable mirror waypoints: %d / %d" % (len(kept), len(YS) * len(ZS)))

        # build a streamable joint path: bridge consecutive kept points; if the Cartesian
        # gap is large (faucet gap / row jump), go through a retracted via so we lift off.
        def bridge(a, b, n):
            return [(np.array(a) + (np.array(b) - np.array(a)) * k / n).tolist() for k in range(1, n + 1)]

        path = [kept[0]]
        for a, b in zip(kept[:-1], kept[1:]):
            gap = np.linalg.norm(self.cart(a) - self.cart(b))
            if gap > 0.08:                                  # disconnected -> lift over
                via_a = self.retract(a, 0.10)
                via_b = self.retract(b, 0.10)
                path += bridge(a, via_a, 12) + bridge(via_a, via_b, 20) + bridge(via_b, b, 12)
            else:
                path += bridge(a, b, 4)

        # FK extent report
        T = np.array([self.cart(q) for q in path])
        M = T[T[:, 0] > 0.40]
        self.get_logger().info(
            "mirror coverage extent: y %.2f..%.2f  z %.2f..%.2f  (%d traj pts)"
            % (M[:, 1].min(), M[:, 1].max(), M[:, 2].min(), M[:, 2].max(), len(path))
        )
        bl = ((T[:, 1] < -0.10) & (T[:, 2] < 0.20) & (T[:, 0] > 0.40)).sum()
        br = ((T[:, 1] > 0.10) & (T[:, 2] < 0.20) & (T[:, 0] > 0.40)).sum()
        self.get_logger().info("bottom-left pts: %d   bottom-right pts: %d" % (bl, br))

        with open("/home/dev/data/p_mirror_full.yaml", "w") as f:
            f.write("joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6']\n")
            f.write("points:\n")
            t = 0.0
            for q in path:
                t += 0.05
                f.write("  - t: %.3f\n" % t)
                f.write("    positions: %s\n" % [float(x) for x in q])
        self.get_logger().info("wrote /home/dev/data/p_mirror_full.yaml")


def main():
    rclpy.init()
    n = Gen()
    try:
        n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
