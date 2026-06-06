#!/usr/bin/env python3
"""Headless Section-3 demo GIF.

Renders the saved wiping joint trajectories (counter then mirror) as an animated
arm skeleton (per-segment FK) next to the synchronized contact-force trace. No
RViz / screen capture needed — pure matplotlib + Pillow, so it always works.

  ros2 run is not needed; run inside the jazzy container with move_group up
  (for robot_description):  python3 make_demo_gif.py

Output: /home/dev/data/wiping_demo.gif
"""
import subprocess
import csv
import numpy as np
import yaml
import PyKDL as kdl
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from gr_wiping_control.kdl_chain import Kinematics

TOOL_LEN = 0.02
OUT = "/home/dev/data/wiping_demo.gif"

urdf = subprocess.check_output(
    ["ros2", "param", "get", "--hide-type", "/robot_state_publisher", "robot_description"]
).decode()
kin = Kinematics(urdf, "base_link", "link6")
nseg = kin.chain.getNrOfSegments()


def link_points(q):
    """Origins of each link in base_link (base + 6 segment endpoints)."""
    jnt = kin._to_jnt(q)
    pts = [np.zeros(3)]
    f = kdl.Frame()
    for s in range(1, nseg + 1):
        kin.fk.JntToCart(jnt, f, s)
        pts.append(np.array([f.p[0], f.p[1], f.p[2]]))
    return np.asarray(pts)


def tip(q):
    pos, R = kin.fk_pose(q)
    return pos + TOOL_LEN * R[:, 2]   # pad face = link6 + tool_len along tool z


def load_traj(path):
    d = yaml.safe_load(open(path))
    return np.array([p["positions"] for p in d["points"]])


def load_force(path):
    F = []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            F.append(float(row["F_meas_N"]))
    return np.asarray(F)


# ---- per-surface config: (traj, log, target, tol, label, surface_z/x, normal) ----
SEGMENTS = [
    dict(traj="/home/dev/data/wiping_trajectory.yaml",
         log="/home/dev/data/wiping_log.csv",
         target=10.0, tol=2.0, label="counter", axis="z", level=-0.05),
    dict(traj="/home/dev/data/wiping_trajectory_mirror.yaml",
         log="/home/dev/data/wiping_log_mirror.csv",
         target=6.0, tol=1.5, label="mirror", axis="x", level=0.44),
]

frames = []   # list of dicts per animation frame
for seg in SEGMENTS:
    Q = load_traj(seg["traj"])
    F = load_force(seg["log"])
    # detect first contact (tool penetrates the surface) -> earlier pts are transit
    pen = []
    for q in Q:
        t = tip(q)
        p = (seg["level"] - t[2]) if seg["axis"] == "z" else (t[0] - seg["level"])
        pen.append(p)
    pen = np.asarray(pen)
    contact = np.where(pen > 0.002)[0]
    first = int(contact[0]) if len(contact) else len(Q)
    step = max(1, len(Q) // 95)
    for i in range(0, len(Q), step):
        if i < first:
            fval = 0.0
        else:
            frac = (i - first) / max(1, (len(Q) - 1 - first))
            fval = float(F[int(frac * (len(F) - 1))])
        frames.append(dict(q=Q[i], force=fval, seg=seg, new=(i == 0)))

print(f"{len(frames)} frames")

fig = plt.figure(figsize=(12, 5.4))
ax3 = fig.add_subplot(1, 2, 1, projection="3d")
axf = fig.add_subplot(1, 2, 2)
gt = np.arange(len(frames)) * 0.1   # pseudo global time (s) for the force x-axis
forces = np.array([fr["force"] for fr in frames])
path_pts = []


def draw_surface(ax, seg):
    if seg["axis"] == "z":
        z = seg["level"]
        quad = [[0.18, -0.18, z], [0.42, -0.18, z], [0.42, 0.18, z], [0.18, 0.18, z]]
        col = "#5b8def"
    else:
        x = seg["level"]
        quad = [[x, -0.18, 0.18], [x, 0.18, 0.18], [x, 0.18, 0.36], [x, -0.18, 0.36]]
        col = "#9b59b6"
    pc = Poly3DCollection([quad], alpha=0.18, facecolor=col, edgecolor=col)
    ax.add_collection3d(pc)
    # faucet post at (0.40, 0)
    ax.plot([0.40, 0.40], [0.0, 0.0], [-0.05, 0.10], color="0.4", lw=4, alpha=0.7)


def update(k):
    fr = frames[k]
    seg = fr["seg"]
    if fr["new"]:
        path_pts.clear()
    ax3.clear()
    P = link_points(fr["q"])
    draw_surface(ax3, seg)
    ax3.plot(P[:, 0], P[:, 1], P[:, 2], "-o", color="0.15", lw=2.5, ms=4, zorder=5)
    t = tip(fr["q"])
    path_pts.append((t, fr["force"]))
    tp = np.array([p[0] for p in path_pts])
    cols = ["tab:green" if p[1] > 2 else "0.7" for p in path_pts]
    ax3.scatter(tp[:, 0], tp[:, 1], tp[:, 2], c=cols, s=7, zorder=4)
    ax3.scatter([t[0]], [t[1]], [t[2]], c="red", s=40, zorder=6)
    ax3.set_xlim(-0.05, 0.5); ax3.set_ylim(-0.3, 0.3); ax3.set_zlim(-0.1, 0.5)
    ax3.set_xlabel("x"); ax3.set_ylabel("y"); ax3.set_zlabel("z")
    ax3.view_init(elev=22, azim=-60)
    ax3.set_title(f"Section 3 wiping — {seg['label']}   F = {fr['force']:.1f} N", fontsize=11)

    axf.clear()
    axf.axhspan(seg["target"] - seg["tol"], seg["target"] + seg["tol"],
                color="tab:green", alpha=0.15, label=f"target {seg['target']:.0f}±{seg['tol']:.0f} N")
    axf.axhline(2.0, ls="--", color="orange", lw=1, label="contact 2 N")
    axf.axhline(15.0, ls="--", color="red", lw=1, label="backoff 15 N")
    axf.plot(gt[:k + 1], forces[:k + 1], "-", color="navy", lw=1.5)
    axf.scatter([gt[k]], [forces[k]], color="red", s=30, zorder=5)
    axf.set_xlim(0, gt[-1]); axf.set_ylim(0, 18)
    axf.set_xlabel("time (s)"); axf.set_ylabel("contact force (N)")
    axf.legend(loc="upper right", fontsize=7)
    axf.set_title("Measured contact force (simulated F/T sensor)", fontsize=11)


ani = FuncAnimation(fig, update, frames=len(frames), interval=90)
ani.save(OUT, writer=PillowWriter(fps=11))
print("wrote", OUT)
