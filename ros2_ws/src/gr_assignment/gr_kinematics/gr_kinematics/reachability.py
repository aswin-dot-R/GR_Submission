"""Section 1, Task 3: Reachability heatmap over a 60x60 cm patch of the
countertop at 2 cm resolution. For each cell, queries /gr_kinematics/solve_ik
with the tool oriented so its approach axis is aligned with the surface normal
(z_tool = -z_world, i.e. pointing straight down onto the horizontal counter).

IMPORTANT — surface-aligned != one fixed full pose:
  "Surface-aligned" constrains only the tool's APPROACH AXIS (z_tool) to the
  surface normal. The rotation ABOUT that axis (the yaw of a flat wiping pad)
  is a FREE degree of freedom — a square pad wipes the same whether it's at
  0deg or 90deg. An earlier version of this sweep pinned the full orientation
  to a single quaternion [w,x,y,z]=[0,1,0,0] (== tool-down at yaw=0deg). For the
  Piper, yaw=0deg is infeasible across most of the patch, so that version reported
  only 34/961 (3.5%) reachable — an artifact of over-constraining a free DOF,
  not a real workspace limit. (Verified: the patch centre (0.40,0) solves at
  17 of 24 yaw angles with the tool still pointing straight down.)

  Fix: for each cell we keep z_tool aligned with the surface normal but SEARCH
  over a configurable set of yaw angles, and mark the cell reachable if ANY yaw
  has a collision-free IK solution. The feasible yaw is recorded. None of the
  assignment-given inputs change (patch 60x60 cm, 2 cm resolution, surface-height
  probe, tool pointing down) — only the unspecified yaw is freed.

Writes:
  - CSV: x, y, reachable (0/1), error_code, feasible_yaw_deg (-1 if none)
  - PNG: heatmap rendered with matplotlib
"""
import csv
import math
import os
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


class ReachabilitySweep(Node):
    def __init__(self):
        super().__init__("reachability_sweep")
        self.declare_parameter("reachability.planning_group", "arm")
        self.declare_parameter("reachability.base_frame", "base_link")
        self.declare_parameter("reachability.tip_link", "link6")
        self.declare_parameter("reachability.patch.x_center", 0.40)
        self.declare_parameter("reachability.patch.y_center", 0.0)
        self.declare_parameter("reachability.patch.width", 0.60)
        self.declare_parameter("reachability.patch.height", 0.60)
        self.declare_parameter("reachability.patch.resolution", 0.02)
        self.declare_parameter("reachability.patch.z_offset", 0.02)
        # Base "tool-down" orientation: z_tool aligned with -z_world (surface normal
        # of the horizontal counter). Yaw about that axis is searched, not fixed.
        self.declare_parameter("reachability.tool_orientation_wxyz", [0.0, 1.0, 0.0, 0.0])
        # Number of yaw samples about the surface normal to try per cell (free DOF
        # of a wiping pad). A cell counts as reachable if ANY yaw solves IK.
        self.declare_parameter("reachability.yaw_samples", 12)
        self.declare_parameter("reachability.output.csv_path", "/home/dev/data/reachability.csv")
        self.declare_parameter("reachability.output.png_path", "/home/dev/data/reachability.png")

        gp = lambda n: self.get_parameter(f"reachability.{n}").value
        self.group = gp("planning_group")
        self.base = gp("base_frame")
        self.tip = gp("tip_link")
        self.xc, self.yc = gp("patch.x_center"), gp("patch.y_center")
        self.w, self.h = gp("patch.width"), gp("patch.height")
        self.res = gp("patch.resolution")
        self.dz = gp("patch.z_offset")
        self.quat = gp("tool_orientation_wxyz")  # [w, x, y, z] base tool-down (yaw=0)
        self.yaw_samples = int(gp("yaw_samples"))
        self.csv_path = gp("output.csv_path")
        self.png_path = gp("output.png_path")

        self.cli = self.create_client(GetPositionIK, "/gr_kinematics/solve_ik")
        if not self.cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("/gr_kinematics/solve_ik not up — start ik_service first")
            raise SystemExit(1)

        # KDL chain to score each IK solution by Yoshikawa manipulability w=sqrt(det(JJ^T)).
        # w -> 0 at a kinematic singularity: shown as low-dexterity bands inside the
        # reachable region, so the map isn't a flat "all reachable" blob.
        import subprocess
        from gr_wiping_control.kdl_chain import Kinematics
        urdf = subprocess.check_output(
            ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
        ).decode()
        self.kin = Kinematics(urdf, self.base, self.tip)
        self.jnames = list(self.kin.joint_names)

    def _yaw_quat(self, yaw_rad):
        """Quaternion (w,x,y,z) for Rz(yaw) * base-tool-down.

        With the base orientation [0,1,0,0] (180deg about x => z_tool down), adding
        a yaw about the world z / surface normal gives (0, cos(yaw/2), sin(yaw/2), 0):
        the tool keeps pointing straight down while the pad rotates about its normal.
        yaw=0 reproduces the original fixed quaternion exactly.
        """
        c, s = math.cos(yaw_rad / 2.0), math.sin(yaw_rad / 2.0)
        return (0.0, c, s, 0.0)

    def _solve_one(self, x, y, wxyz) -> int:
        ps = PoseStamped()
        ps.header.frame_id = self.base
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(self.dz)
        ps.pose.orientation.w = float(wxyz[0])
        ps.pose.orientation.x = float(wxyz[1])
        ps.pose.orientation.y = float(wxyz[2])
        ps.pose.orientation.z = float(wxyz[3])

        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.tip
        req.ik_request.pose_stamped = ps
        req.ik_request.avoid_collisions = True

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if not res:
            return MoveItErrorCodes.FAILURE, None
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            return res.error_code.val, res.solution.joint_state
        return res.error_code.val, None

    def _manip(self, js):
        try:
            q = [js.position[js.name.index(n)] for n in self.jnames]
            return self.kin.manipulability(q)
        except Exception:
            return 0.0

    def _ik(self, x, y):
        """Surface-aligned reachability for one cell: keep the tool pointing down
        (z_tool = surface normal) and search over yaw (the pad's free DOF). Returns
        (reachable: 0/1, last_error_code, feasible_yaw_deg or -1, manipulability). A
        cell is reachable if ANY sampled yaw has a collision-free IK solution; the
        manipulability is taken at the first feasible yaw (0 if unreachable).
        """
        last = MoveItErrorCodes.FAILURE
        for k in range(self.yaw_samples):
            yaw = 2.0 * math.pi * k / self.yaw_samples
            code, js = self._solve_one(x, y, self._yaw_quat(yaw))
            last = code
            if code == MoveItErrorCodes.SUCCESS and js is not None:
                return 1, code, round(math.degrees(yaw), 1), self._manip(js)
        return 0, last, -1.0, 0.0

    def run(self):
        xs = np.arange(self.xc - self.w / 2, self.xc + self.w / 2 + 1e-9, self.res)
        ys = np.arange(self.yc - self.h / 2, self.yc + self.h / 2 + 1e-9, self.res)
        grid = np.zeros((len(ys), len(xs)), dtype=np.int8)
        manip = np.full((len(ys), len(xs)), np.nan)

        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["x_m", "y_m", "reachable", "error_code", "feasible_yaw_deg", "manipulability"])
            total = len(xs) * len(ys)
            done = 0
            self.get_logger().info(
                f"Surface-aligned sweep: tool-down, {self.yaw_samples} yaw/cell + "
                f"Yoshikawa manipulability (singularity measure) at each IK solution"
            )
            for j, y in enumerate(ys):
                for i, x in enumerate(xs):
                    ok, code, yaw_deg, w = self._ik(x, y)
                    grid[j, i] = ok
                    if ok:
                        manip[j, i] = w
                    wr.writerow([f"{x:.4f}", f"{y:.4f}", ok, code, yaw_deg, f"{w:.5f}"])
                    done += 1
                    if done % 50 == 0:
                        self.get_logger().info(f"{done}/{total} cells ({100*done/total:.1f}%)")

        pct = 100.0 * grid.sum() / grid.size
        rw = manip[np.isfinite(manip)]
        wmax = float(np.nanmax(manip)) if len(rw) else 0.0
        nsing = int((rw < 0.15 * wmax).sum()) if len(rw) else 0
        self.get_logger().info(
            f"Reachable: {grid.sum()}/{grid.size} ({pct:.1f}%); manipulability "
            f"min={np.nanmin(manip) if len(rw) else 0:.4f} max={wmax:.4f}; "
            f"near-singular reachable cells (<15% of max): {nsing}"
        )

        # PRIMARY plot: binary reachability — GREEN = reachable, RED = unreachable.
        # A cell is reachable if ANY yaw (the pad's free DOF) has a collision-free IK
        # solution. (Manipulability is kept in the CSV and a secondary heatmap below.)
        from matplotlib.colors import ListedColormap
        from matplotlib.patches import Patch
        fig, ax = plt.subplots(figsize=(6.6, 6))
        rg = ListedColormap(["#d62728", "#2ca02c"])      # 0 -> red, 1 -> green
        ax.imshow(grid, origin="lower",
                  extent=[xs.min(), xs.max(), ys.min(), ys.max()],
                  cmap=rg, vmin=0, vmax=1, aspect="equal")
        ax.scatter([0], [0], c="white", marker="s", s=55, edgecolors="black", zorder=5)
        ax.set_xlabel("x (m, base_link frame)")
        ax.set_ylabel("y (m, base_link frame)")
        ax.set_title(f"Reachability — {grid.shape[1]}x{grid.shape[0]} @ "
                     f"{self.res*100:.0f} cm  ({pct:.0f}% reachable)")
        ax.legend(handles=[Patch(color="#2ca02c", label="reachable"),
                           Patch(color="#d62728", label="unreachable"),
                           Patch(facecolor="white", edgecolor="black", label="base_link")],
                  loc="upper right", fontsize=8)
        fig.tight_layout()
        fig.savefig(self.png_path, dpi=120)

        # SECONDARY plot: manipulability heatmap (reachable cells shaded by w, grey =
        # unreachable) — the "reachable != usable" view, kept alongside the binary map.
        manip_png = self.png_path.replace(".png", "_manip.png")
        fig2, ax2 = plt.subplots(figsize=(6.6, 6))
        ax2.set_facecolor("0.6")
        im = ax2.imshow(manip, origin="lower",
                        extent=[xs.min(), xs.max(), ys.min(), ys.max()],
                        cmap="viridis", aspect="equal")
        ax2.scatter([0], [0], c="red", marker="s", s=45, label="base_link")
        ax2.set_xlabel("x (m, base_link frame)")
        ax2.set_ylabel("y (m, base_link frame)")
        ax2.set_title(f"Manipulability — {grid.shape[1]}x{grid.shape[0]} @ "
                      f"{self.res*100:.0f} cm  ({pct:.0f}% reachable)")
        ax2.legend(loc="upper right", fontsize=8)
        fig2.colorbar(im, ax=ax2, label="manipulability  w=sqrt(det(J Jᵀ))   (grey = unreachable)")
        fig2.tight_layout()
        fig2.savefig(manip_png, dpi=120)
        self.get_logger().info(f"Wrote {self.csv_path}, {self.png_path} (binary), {manip_png} (manip)")


def main():
    rclpy.init()
    n = ReachabilitySweep()
    n.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
