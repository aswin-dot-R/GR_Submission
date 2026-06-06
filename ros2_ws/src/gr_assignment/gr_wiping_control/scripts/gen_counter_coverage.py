#!/usr/bin/env python3
"""Generate a FULL-reachable-region COUNTER coverage joint trajectory for the Gazebo
demo — covering everything Section 2 marks reachable on the countertop, not just the
small central band a single compute_cartesian_path stroke executes.

Same strategy as gen_mirror_coverage.py: raster the counter plane (z=0) at the pad
pitch in boustrophedon order; per-waypoint collision-aware /compute_ik SEEDED from the
previous solution (smooth, pad stays near the surface); skip unreachable cells (near-
base hole, faucet keep-out, far edge); bridge disconnected kept points through a
retracted via so the arm lifts off and never scrapes the faucet.

Run in the jazzy container (move_group up):
  python3 gen_counter_coverage.py
Writes /home/dev/data/p_counter_full.yaml
"""
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from gr_wiping_control.kdl_chain import Kinematics

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TL = 0.02
SURF_Z = 0.02                        # link6 z so the pad tip sits ~on the slab (z=0) + standoff
YAW = np.deg2rad(150.0)              # best counter yaw (from Section 2)
# counter reachable raster over the full front field. Every cell is ATTEMPTED; cells where
# IK self-collides, or which have no collision-free connecting motion (the near-base
# "grey hole" right under the shoulder, where the arm folds onto itself), are skipped and
# we move on to the next viable cell — so the sweep keeps whatever near-base cells ARE
# safely wipeable and drops only the truly-unusable ones (Section-1 "reachable != usable").
XS = np.round(np.arange(0.0, 0.401, 0.04), 3)
YS = np.round(np.arange(-0.24, 0.241, 0.04), 3)
# faucet base keep-out (faucet is a 6 cm post at x=0.40, y=0 standing on the slab):
# skip raster cells whose tool would put the wrist into the faucet column. These cells
# are occluded by the obstacle anyway (PDF Sec-3 Task-3: skip the obstacle).
FAUCET_XY = (0.40, 0.0)
FAUCET_KEEPOUT = 0.10
# The wiping PAD (tool_link, 100x50x20 mm) is solid in Gazebo but is NOT in the MoveIt
# planning URDF, so /check_state_validity is blind to pad<->scene contact. The pad lies
# flat on the counter and extends ~half its diagonal (~0.056 m) past the contact point;
# the mirror face is at x=0.44. So a counter contact whose pad would reach the mirror must
# be skipped (the far counter rows). Keep the pad clear of the mirror by this margin.
MIRROR_FACE_X = 0.44
PAD_HALF_DIAG = 0.056
PAD_MIRROR_MARGIN = 0.01
COUNTER_X_MAX = MIRROR_FACE_X - PAD_HALF_DIAG - PAD_MIRROR_MARGIN   # ~0.374


def tooldown_quat(yaw):
    """Rotation with tool-z (approach) = world -z, rotated by `yaw` about it. -> (x,y,z,w)."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S; x = (R[2, 1] - R[1, 2]) / S; y = (R[0, 2] - R[2, 0]) / S; z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S; x = 0.25 * S; y = (R[0, 1] + R[1, 0]) / S; z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S; x = (R[0, 1] + R[1, 0]) / S; y = 0.25 * S; z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S; x = (R[0, 2] + R[2, 0]) / S; y = (R[1, 2] + R[2, 1]) / S; z = 0.25 * S
    return (x, y, z, w)


class Gen(Node):
    def __init__(self):
        super().__init__("gen_counter_cov")
        urdf = subprocess.check_output(
            ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
        ).decode()
        self.kin = Kinematics(urdf, "base_link", "link6")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.ik.wait_for_service(timeout_sec=10)
        self.cv = self.create_client(GetStateValidity, "/check_state_validity")
        self.cv.wait_for_service(timeout_sec=10)
        self.quat = tooldown_quat(YAW)

    def valid(self, q):
        """True if joint config q is collision-free against the planning scene AND the
        wiping PAD clears the scene. MoveIt's URDF has no pad (tool_link), so its collision
        check is blind to pad<->mirror / pad<->faucet contact — we add an explicit pad-box
        geometric test to compensate."""
        rs = RobotState(); js = JointState(); js.name = NAMES
        js.position = [float(v) for v in q]; rs.joint_state = js
        req = GetStateValidity.Request(); req.robot_state = rs; req.group_name = "arm"
        f = self.cv.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3)
        r = f.result()
        return bool(r and r.valid) and self.pad_clear(q)

    def pad_clear(self, q):
        """True if the wiping pad box (100x50x20 mm, 1 cm past link6) clears the mirror
        and faucet. The 8 pad corners must avoid both obstacle AABBs."""
        p, R = self.kin.fk_pose(q)
        c = p + 0.01 * R[:, 2]                          # pad center
        hx, hy, hz = 0.05, 0.025, 0.01
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    P = c + sx * hx * R[:, 0] + sy * hy * R[:, 1] + sz * hz * R[:, 2]
                    if 0.43 <= P[0] <= 0.45 and -0.45 <= P[1] <= 0.45 and 0.0 <= P[2] <= 0.60:
                        return False                    # pad in mirror slab
                    if 0.37 <= P[0] <= 0.43 and -0.03 <= P[1] <= 0.03 and 0.0 <= P[2] <= 0.15:
                        return False                    # pad in faucet column
        return True

    def solve(self, x, y, seed):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = SURF_Z
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = self.quat
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = "arm"
        r.pose_stamped = ps
        r.timeout.sec = 1
        r.avoid_collisions = True
        if seed is not None:
            rs = RobotState(); js = JointState(); js.name = NAMES
            js.position = [float(v) for v in seed]; rs.joint_state = js
            r.robot_state = rs
        req.ik_request = r
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=4)
        res = f.result()
        if not res or res.error_code.val != 1:
            return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in NAMES]

    def retract(self, q, dist=0.10):
        Jac = self.kin.jacobian(q); tz = self.kin.fk_pose(q)[1][:, 2]
        return (np.array(q) + np.linalg.pinv(Jac) @ np.concatenate([-dist * tz, [0, 0, 0]])).tolist()

    def cart(self, q):
        p, R = self.kin.fk_pose(q); return p + TL * R[:, 2]

    def run(self):
        # 1) COLLECT viable cells. Attempt every raster cell; if IK fails OR the returned
        #    config self-collides, SKIP that cell and move on to the next viable one.
        kept = []; seed = None; skipped_faucet = 0; skipped_collide = 0; skipped_mirror = 0
        for i, x in enumerate(XS):
            row_ys = YS if i % 2 == 0 else YS[::-1]
            for y in row_ys:
                if x > COUNTER_X_MAX:
                    skipped_mirror += 1; continue           # pad would hit the mirror -> skip
                if np.hypot(x - FAUCET_XY[0], y - FAUCET_XY[1]) < FAUCET_KEEPOUT:
                    skipped_faucet += 1; continue           # occluded by the faucet -> skip
                q = self.solve(x, y, seed)
                if q is None:
                    continue                                # no IK -> skip cell
                if not self.valid(q):
                    skipped_collide += 1; continue          # IK self-collides -> skip cell, move on
                kept.append(q); seed = q
        self.get_logger().info(
            "viable counter cells: %d / %d  (skipped %d mirror-pad keep-out, %d faucet keep-out, %d self-collision)"
            % (len(kept), len(XS) * len(YS), skipped_mirror, skipped_faucet, skipped_collide))

        def bridge(a, b, n):
            return [(np.array(a) + (np.array(b) - np.array(a)) * k / n).tolist() for k in range(1, n + 1)]

        def collide_free(seg):
            return all(self.valid(q) for q in seg)

        # 2) CONNECT viable cells with only collision-free motion. Try direct, then lift
        #    over at increasing heights. If NO safe connection exists, SKIP that cell (drop
        #    the link) and move on to the next — never emit a colliding bridge.
        path = [kept[0]]
        connected = 1; skipped_link = 0
        for b in kept[1:]:
            a = path[-1]
            direct = bridge(a, b, 6)
            if np.linalg.norm(self.cart(a) - self.cart(b)) <= 0.08 and collide_free(direct):
                path += direct; connected += 1; continue
            placed = False
            for lift in (0.08, 0.12, 0.16, 0.22, 0.30):     # lift-and-traverse over obstacles
                va, vb = self.retract(a, lift), self.retract(b, lift)
                seg = bridge(a, va, 10) + bridge(va, vb, 18) + bridge(vb, b, 10)
                if collide_free(seg):
                    path += seg; placed = True; connected += 1; break
            if not placed:
                skipped_link += 1                            # unreachable-as-a-stroke -> skip cell
        self.get_logger().info(
            "connected %d / %d viable cells (%d skipped: no collision-free path to them)"
            % (connected, len(kept), skipped_link))

        T = np.array([self.cart(q) for q in path]); C = T[T[:, 2] < 0.05]
        self.get_logger().info("counter coverage extent: x %.2f..%.2f  y %.2f..%.2f  (%d traj pts)"
                               % (C[:, 0].min(), C[:, 0].max(), C[:, 1].min(), C[:, 1].max(), len(path)))

        with open("/home/dev/data/p_counter_full.yaml", "w") as f:
            f.write("joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6']\n")
            f.write("points:\n")
            t = 0.0
            for q in path:
                t += 0.05
                f.write("  - t: %.3f\n" % t)
                f.write("    positions: %s\n" % [float(v) for v in q])
        self.get_logger().info("wrote /home/dev/data/p_counter_full.yaml")


def main():
    rclpy.init(); n = Gen()
    try:
        n.run()
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
