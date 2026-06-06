"""Render simple, accurate diagrams of the assignment 'world' (countertop,
faucet, mirror) straight from gr_scene/config/scene.yaml geometry, for the
explainer doc. Outputs:
  data/world_topdown.png  — looking straight down (x-y plane)
  data/world_side.png     — looking from the side (x-z plane)
All coordinates are in the base_link frame, metres.
"""
import os
import csv
import yaml
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.colors import ListedColormap

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "data")
GR = os.path.join(HERE, "..", "ros2_ws", "src", "gr_assignment")
SCENE = os.path.join(GR, "gr_scene", "config", "scene.yaml")
REACH_YAML = os.path.join(GR, "gr_kinematics", "config", "reachability.yaml")
REACH_CSV = os.path.join(OUT, "reachability.csv")

# --- geometry read live from scene.yaml (single source of truth) ---
_p = yaml.safe_load(open(SCENE))["scene_loader"]["ros__parameters"]
def _obj(name):
    return dict(size=tuple(_p[f"scene.{name}.size"]), pose=tuple(_p[f"scene.{name}.pose"]))
COUNTER = _obj("countertop")
FAUCET  = _obj("faucet")
MIRROR  = _obj("mirror")

# reachability patch (Section 1) read live from reachability.yaml
_rp = yaml.safe_load(open(REACH_YAML))["reachability_sweep"]["ros__parameters"]
PATCH = dict(center=(_rp["reachability.patch.x_center"], _rp["reachability.patch.y_center"]),
             w=_rp["reachability.patch.width"], h=_rp["reachability.patch.height"])


def _reach_overlay(ax):
    """Shade the patch by IK reachability (green=reachable, red=not) at low opacity,
    like the Section 2 coverage workspace shading. Reads data/reachability.csv
    (x_m, y_m, reachable). Silently skips if the sweep hasn't been run yet."""
    if not os.path.exists(REACH_CSV):
        return
    xs, ys, rs = [], [], []
    with open(REACH_CSV) as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"])); ys.append(float(row["y_m"]))
            rs.append(float(row["reachable"]))
    if not xs:
        return
    ux = np.array(sorted(set(xs))); uy = np.array(sorted(set(ys)))
    grid = np.full((len(uy), len(ux)), np.nan)
    xi = {v: i for i, v in enumerate(ux)}; yi = {v: i for i, v in enumerate(uy)}
    for x, y, r in zip(xs, ys, rs):
        grid[yi[y], xi[x]] = r
    dx = (ux[1] - ux[0]) / 2 if len(ux) > 1 else 0.01
    dy = (uy[1] - uy[0]) / 2 if len(uy) > 1 else 0.01
    ax.imshow(grid, origin="lower", extent=[ux.min() - dx, ux.max() + dx, uy.min() - dy, uy.max() + dy],
              cmap=ListedColormap(["#d9534f", "#5cb85c"]), vmin=0, vmax=1, alpha=0.45,
              interpolation="nearest", zorder=1.5, aspect="auto")


def _rect(ax, cx, cy, w, h, **kw):
    ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, **kw))


def topdown():
    fig, ax = plt.subplots(figsize=(7, 6))
    # countertop (x by y)
    _rect(ax, COUNTER["pose"][0], COUNTER["pose"][1], COUNTER["size"][0],
          COUNTER["size"][1], facecolor="#d9d2c5", edgecolor="0.4", label="countertop (120x60 cm)")
    # mirror (thin in x, wide in y)
    _rect(ax, MIRROR["pose"][0], MIRROR["pose"][1], MIRROR["size"][0],
          MIRROR["size"][1], facecolor="#9ec7e8", edgecolor="0.3", label="mirror (90 cm wide)")
    # faucet (small square footprint)
    _rect(ax, FAUCET["pose"][0], FAUCET["pose"][1], FAUCET["size"][0],
          FAUCET["size"][1], facecolor="#c0392b", edgecolor="0.2", label="faucet (6x6 cm)")
    # reachability opacity overlay (Section 1) — green=reachable, red=not
    _reach_overlay(ax)
    # reachability patch outline (Section 1)
    _rect(ax, PATCH["center"][0], PATCH["center"][1], PATCH["w"], PATCH["h"],
          fill=False, edgecolor="green", lw=1.6, ls="--", label="reachability patch (60x60 cm)")
    # robot base
    ax.plot(0, 0, "ks", ms=9)
    ax.annotate("base_link\n(robot base)", (0, 0), textcoords="offset points",
                xytext=(8, 8), fontsize=9)
    # legend proxies for the reachability shading
    if os.path.exists(REACH_CSV):
        from matplotlib.patches import Patch
        ax._reach_handles = [Patch(facecolor="#5cb85c", alpha=0.45, label="reachable (IK)"),
                             Patch(facecolor="#d9534f", alpha=0.45, label="unreachable")]

    ax.set_xlabel("x (m) — forward from base")
    ax.set_ylabel("y (m) — left/right")
    ax.set_title("World — top-down view (looking down the z-axis)")
    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.5)
    h, _l = ax.get_legend_handles_labels()
    h += getattr(ax, "_reach_handles", [])
    ax.legend(handles=h, loc="upper left", fontsize=8)
    ax.set_xlim(-0.2, 1.2)
    ax.set_ylim(-0.6, 0.6)
    fig.tight_layout()
    p = os.path.join(OUT, "world_topdown.png")
    fig.savefig(p, dpi=120)
    print("wrote", p)


def side():
    fig, ax = plt.subplots(figsize=(7, 5))
    # countertop (x by z)
    _rect(ax, COUNTER["pose"][0], COUNTER["pose"][2], COUNTER["size"][0],
          COUNTER["size"][2], facecolor="#d9d2c5", edgecolor="0.4", label="countertop (top at z=0)")
    # mirror (thin in x, tall in z)
    _rect(ax, MIRROR["pose"][0], MIRROR["pose"][2], MIRROR["size"][0],
          MIRROR["size"][2], facecolor="#9ec7e8", edgecolor="0.3", label="mirror (60 cm tall)")
    # faucet (small in x, 30 cm tall)
    _rect(ax, FAUCET["pose"][0], FAUCET["pose"][2], FAUCET["size"][0],
          FAUCET["size"][2], facecolor="#c0392b", edgecolor="0.2", label="faucet (30 cm tall)")
    ax.axhline(0.0, color="0.7", lw=0.8, ls=":")
    ax.plot(0, 0, "ks", ms=9)
    ax.annotate("base_link", (0, 0), textcoords="offset points", xytext=(6, 6), fontsize=9)

    ax.set_xlabel("x (m) — forward from base")
    ax.set_ylabel("z (m) — height")
    ax.set_title("World — side view (x-z plane)")
    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(loc="upper left", fontsize=8)
    ax.set_xlim(-0.2, 1.2)
    ax.set_ylim(-0.15, 0.7)
    fig.tight_layout()
    p = os.path.join(OUT, "world_side.png")
    fig.savefig(p, dpi=120)
    print("wrote", p)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    topdown()
    side()
