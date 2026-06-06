#!/usr/bin/env python3
"""Coverage paint-trail for the Gazebo Classic wiping demo.

Tracks the pad's contact pose (FK of /joint_states) and stamps a coloured tile the
size and orientation of the WHOLE wiping pad (100 x 50 mm footprint) for each new
3 cm grid cell it touches — GREEN on the countertop, BLUE on the mirror. Because the
tiles are pad-sized and overlap as the arm travels, the swept region builds up as a
continuous pad-width swath rather than a trail of dots. Clears any previous paint
tiles at startup (no leftover clutter).

  Run in the humble container alongside the Classic sim:
    python3 paint_trail.py
"""
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList

from gr_wiping_control.kdl_chain import Kinematics

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TL = 0.02
COUNTER_Z = 0.0
MIRROR_X = 0.44
GRID = 0.03
# wiping pad footprint (matches gr_coverage tool.size_x/​y) + a thin paint depth.
PAD_X = 0.100
PAD_Y = 0.050
PAD_T = 0.004


def tile_sdf(name, rgba, sx, sy, sz):
    return ('<sdf version="1.6"><model name="' + name + '"><static>true</static><link name="l">'
            '<visual name="v"><geometry><box><size>%g %g %g</size></box></geometry>' % (sx, sy, sz) +
            '<material><ambient>' + rgba + '</ambient><diffuse>' + rgba + '</diffuse></material>'
            '</visual></link></model></sdf>')


def rot2quat(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion."""
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
    return x, y, z, w


def _fetch_urdf(retries=20, delay=1.0):
    """Fetch robot_description via the param service, retrying — the service is flaky
    right after the sim comes up (returns non-zero before the param is served)."""
    for i in range(retries):
        try:
            out = subprocess.check_output(
                ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"],
                stderr=subprocess.DEVNULL,
            ).decode()
            if "<robot" in out:
                return out
        except subprocess.CalledProcessError:
            pass
        time.sleep(delay)
    raise RuntimeError("could not fetch robot_description after %d tries" % retries)


class PaintTrail(Node):
    def __init__(self):
        super().__init__("paint_trail")
        urdf = _fetch_urdf()
        self.kin = Kinematics(urdf, "base_link", "link6")
        self.cur = {}
        self.seen = set()
        self.cnt = 0
        self.sp = self.create_client(SpawnEntity, "/spawn_entity")
        self.dl = self.create_client(DeleteEntity, "/delete_entity")
        self.gl = self.create_client(GetModelList, "/get_model_list")
        self.sp.wait_for_service(timeout_sec=10)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self._clear_old()

    def _js(self, m):
        self.cur.update(zip(m.name, m.position))

    def _clear_old(self):
        if not self.gl.wait_for_service(timeout_sec=3):
            return
        fut = self.gl.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=4)
        res = fut.result()
        if not res:
            return
        for name in res.model_names:
            if name.startswith("paint_"):
                self.dl.call_async(DeleteEntity.Request(name=name))
        self.get_logger().info("cleared old paint tiles")

    def _spawn(self, px, py, pz, surf, quat):
        key = (surf, round(px / GRID), round(py / GRID), round(pz / GRID))
        if key in self.seen:
            return
        self.seen.add(key)
        self.cnt += 1
        self.gc = getattr(self, "gc", 0) + (1 if surf == "c" else 0)
        self.bc = getattr(self, "bc", 0) + (1 if surf == "m" else 0)
        if self.cnt % 20 == 0:
            self.get_logger().info("spawned green(counter)=%d blue(mirror)=%d; last %s @ (%.2f,%.2f,%.3f)"
                                   % (self.gc, self.bc, surf, px, py, pz))
        # Stamp the WHOLE pad footprint (100 x 50 mm). The pad's long axis (PAD_X) lies
        # along the tool x-axis, short axis (PAD_Y) along tool y; the thin paint depth
        # (PAD_T) along the tool approach axis (+z). The tile is rotated to the pad pose
        # via `quat`, so on the counter it lies flat and on the mirror it stands upright.
        rgba = "0.1 0.9 0.2 0.7" if surf == "c" else "0.2 0.6 1 0.7"
        sdf = tile_sdf("paint%d" % self.cnt, rgba, PAD_X, PAD_Y, PAD_T)
        r = SpawnEntity.Request(); r.name = "paint_%d" % self.cnt; r.xml = sdf
        r.initial_pose.position.x = float(px)
        r.initial_pose.position.y = float(py)
        r.initial_pose.position.z = float(pz)
        r.initial_pose.orientation.x = float(quat[0])
        r.initial_pose.orientation.y = float(quat[1])
        r.initial_pose.orientation.z = float(quat[2])
        r.initial_pose.orientation.w = float(quat[3])
        self.sp.call_async(r)

    def tick(self):
        if not all(j in self.cur for j in NAMES):
            return
        q = [self.cur[j] for j in NAMES]
        pos, R = self.kin.fk_pose(q)
        tip = pos + TL * R[:, 2]
        quat = rot2quat(R)            # pad orientation = tool orientation
        # Discriminate the two surfaces by HEIGHT, not x: the countertop pad is at z~0 for
        # ANY x (the sweep now reaches x=0.40, so the old "x<0.36" guard left a dead zone
        # at the far counter edge); the mirror pad is up the pane (z>0.06) at x~0.44.
        if abs(tip[2] - COUNTER_Z) < 0.03 and tip[0] < 0.42:
            self._spawn(tip[0], tip[1], COUNTER_Z + PAD_T, "c", quat)
        elif abs(tip[0] - MIRROR_X) < 0.04 and tip[2] > 0.06:
            self._spawn(MIRROR_X + PAD_T, tip[1], tip[2], "m", quat)


def main():
    rclpy.init()
    n = PaintTrail()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.03)
            n.tick()
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
