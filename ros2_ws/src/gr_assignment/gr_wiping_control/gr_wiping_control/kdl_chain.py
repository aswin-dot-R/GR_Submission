"""Build a PyKDL chain from a URDF string (kdl_parser_py is not installed in the
container, so we construct the chain directly). Provides FK, Jacobian, and an
LMA position-IK solver for the base_link -> tip chain. Self-contained — no
move_group needed.
"""
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF


def _rpy_to_rot(rpy):
    r, p, y = rpy
    return kdl.Rotation.RPY(r, p, y)


def _origin_to_frame(origin):
    if origin is None:
        return kdl.Frame()
    xyz = origin.xyz or [0, 0, 0]
    rpy = origin.rpy or [0, 0, 0]
    return kdl.Frame(_rpy_to_rot(rpy), kdl.Vector(*xyz))


def _joint(j):
    """URDF joint -> KDL joint (revolute/continuous about axis, else fixed)."""
    if j.type in ("revolute", "continuous") and j.axis is not None:
        axis = kdl.Vector(*j.axis)
        origin = _origin_to_frame(j.origin)
        # KDL joint axis is expressed at the joint origin
        return kdl.Joint(j.name, origin.p, origin.M * axis, kdl.Joint.RotAxis)
    return kdl.Joint(j.name, kdl.Joint.Fixed)


def build_chain(urdf_str: str, base: str, tip: str):
    robot = URDF.from_xml_string(urdf_str)
    # map child_link -> (joint, parent_link)
    child_map = {j.child: j for j in robot.joints}
    # walk from tip up to base
    joints = []
    link = tip
    while link != base:
        if link not in child_map:
            raise RuntimeError(f"no path from {tip} to {base} (stuck at {link})")
        j = child_map[link]
        joints.append(j)
        link = j.parent
    joints.reverse()

    chain = kdl.Chain()
    for j in joints:
        origin = _origin_to_frame(j.origin)
        if j.type in ("revolute", "continuous") and j.axis is not None:
            axis = origin.M * kdl.Vector(*j.axis)
            kj = kdl.Joint(j.name, origin.p, axis, kdl.Joint.RotAxis)
            seg = kdl.Segment(j.child, kj, origin)
        else:
            kj = kdl.Joint(j.name, kdl.Joint.Fixed)
            seg = kdl.Segment(j.child, kj, origin)
        chain.addSegment(seg)
    return chain, [j.name for j in joints if j.type in ("revolute", "continuous")]


class Kinematics:
    def __init__(self, urdf_str: str, base: str, tip: str):
        self.chain, self.joint_names = build_chain(urdf_str, base, tip)
        self.nj = self.chain.getNrOfJoints()
        self.fk = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ikv = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik = kdl.ChainIkSolverPos_LMA(self.chain)
        self.jac = kdl.ChainJntToJacSolver(self.chain)

    def _to_jnt(self, q):
        a = kdl.JntArray(self.nj)
        for i in range(self.nj):
            a[i] = float(q[i])
        return a

    def fk_pose(self, q):
        """Return (pos[3], rot3x3) of tip in base for joint vector q."""
        f = kdl.Frame()
        self.fk.JntToCart(self._to_jnt(q), f)
        pos = np.array([f.p[0], f.p[1], f.p[2]])
        R = np.array([[f.M[i, j] for j in range(3)] for i in range(3)])
        return pos, R

    def jacobian(self, q):
        """6 x n geometric Jacobian of the tip wrt joints, in base_link."""
        jac = kdl.Jacobian(self.nj)
        self.jac.JntToJac(self._to_jnt(q), jac)
        return np.array([[jac[r, c] for c in range(self.nj)] for r in range(6)])

    def manipulability(self, q):
        """Yoshikawa manipulability w = sqrt(det(J J^T)). -> 0 near a singularity."""
        J = self.jacobian(q)
        return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))

    def ik_pose(self, q_seed, pos, quat_xyzw):
        """Solve IK for target position + orientation. Returns (ok, q)."""
        rot = kdl.Rotation.Quaternion(*quat_xyzw)
        target = kdl.Frame(rot, kdl.Vector(*pos))
        q_out = kdl.JntArray(self.nj)
        rc = self.ik.CartToJnt(self._to_jnt(q_seed), target, q_out)
        q = np.array([q_out[i] for i in range(self.nj)])
        return rc >= 0, q

    def cart_diff(self, q, target_pos, quat_xyzw):
        """6D error twist [vx,vy,vz,wx,wy,wz] from FK(q) to the target frame
        (the displacement that, applied for unit time, moves tip onto target)."""
        cur = kdl.Frame()
        self.fk.JntToCart(self._to_jnt(q), cur)
        tgt = kdl.Frame(kdl.Rotation.Quaternion(*quat_xyzw), kdl.Vector(*target_pos))
        tw = kdl.diff(cur, tgt)
        return np.array([tw.vel[0], tw.vel[1], tw.vel[2], tw.rot[0], tw.rot[1], tw.rot[2]])

    def vel_ik(self, q, twist):
        """Joint delta dq for a desired tip twist via damped pseudo-inverse."""
        tw = kdl.Twist(kdl.Vector(twist[0], twist[1], twist[2]),
                       kdl.Vector(twist[3], twist[4], twist[5]))
        dq = kdl.JntArray(self.nj)
        self.ikv.CartToJnt(self._to_jnt(q), tw, dq)
        return np.array([dq[i] for i in range(self.nj)])
