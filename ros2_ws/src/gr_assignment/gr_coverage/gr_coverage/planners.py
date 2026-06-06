"""Pure-Python coverage path generators. No ROS imports here — kept testable
and reusable across the raster / spiral / metric flows.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable
import math
import numpy as np


@dataclass
class Surface:
    """A planar surface in the arm's base frame.

    `center` is the centroid of the surface, `u` and `v` are orthonormal
    in-plane axes (right-handed with `normal`), `size_u` and `size_v` are the
    extents along `u` and `v` respectively, `normal` points OUT of the surface.
    """
    center: np.ndarray
    u: np.ndarray
    v: np.ndarray
    normal: np.ndarray
    size_u: float
    size_v: float

    @classmethod
    def from_horizontal(cls, center, size_xy, normal=(0, 0, 1)) -> "Surface":
        c = np.asarray(center, dtype=float)
        n = np.asarray(normal, dtype=float)
        n = n / np.linalg.norm(n)
        # pick world-x as u if n is z-up, otherwise fall back
        u = np.array([1.0, 0.0, 0.0]) if abs(n[2]) > 0.9 else np.array([0.0, 0.0, 1.0])
        u = u - np.dot(u, n) * n
        u = u / np.linalg.norm(u)
        v = np.cross(n, u)
        return cls(c, u, v, n, float(size_xy[0]), float(size_xy[1]))

    @classmethod
    def from_vertical(cls, center, size_uv, normal=(-1, 0, 0)) -> "Surface":
        """Vertical pane (e.g. mirror). `size_uv` = (height, width)."""
        c = np.asarray(center, dtype=float)
        n = np.asarray(normal, dtype=float) / np.linalg.norm(normal)
        # v = world up, projected to surface; u completes the right-handed frame
        v = np.array([0.0, 0.0, 1.0]) - np.dot([0, 0, 1], n) * n
        v = v / np.linalg.norm(v)
        u = np.cross(v, n)  # horizontal in the pane
        return cls(c, u, v, n, float(size_uv[1]), float(size_uv[0]))  # u=width, v=height

    def to_world(self, su: float, sv: float, standoff: float) -> np.ndarray:
        """In-plane (su, sv) + standoff along +normal → 3D world point."""
        return self.center + su * self.u + sv * self.v + standoff * self.normal


def raster_path(
    surface: Surface,
    tool_size_u: float,
    tool_size_v: float,
    overlap: float,
    margin: float,
    standoff: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Boustrophedon raster covering `surface` with the tool footprint.

    Returns (points_xyz, surface_uv) where surface_uv is the same path in
    surface-local 2D coords (useful for the metrics / coverage check).
    Strokes run along `u`; row pitch along `v`.
    """
    if not (0.0 <= overlap < 1.0):
        raise ValueError(f"overlap must be in [0, 1); got {overlap}")
    eff_u = tool_size_u  # the long axis sweeps along u as the tool moves
    pitch_v = tool_size_v * (1.0 - overlap)

    half_u = surface.size_u / 2 - margin - eff_u / 2
    half_v = surface.size_v / 2 - margin - tool_size_v / 2
    if half_u <= 0 or half_v <= 0:
        return np.empty((0, 3)), np.empty((0, 2))

    n_rows = max(1, int(math.ceil(2 * half_v / pitch_v)) + 1)
    vs = np.linspace(-half_v, half_v, n_rows)

    pts, uv = [], []
    for r, v in enumerate(vs):
        u_start, u_end = (-half_u, half_u) if r % 2 == 0 else (half_u, -half_u)
        pts.append(surface.to_world(u_start, v, standoff))
        uv.append([u_start, v])
        pts.append(surface.to_world(u_end, v, standoff))
        uv.append([u_end, v])
    return np.asarray(pts), np.asarray(uv)


def spiral_path(
    surface: Surface,
    tool_size_u: float,
    tool_size_v: float,
    overlap: float,
    margin: float,
    standoff: float,
    samples_per_turn: int = 48,
) -> tuple[np.ndarray, np.ndarray]:
    """Inward Archimedean spiral. Step per turn = effective tool footprint
    (mean of u/v size) * (1 - overlap). Good for round/mirror surfaces."""
    if not (0.0 <= overlap < 1.0):
        raise ValueError(f"overlap must be in [0, 1); got {overlap}")
    half_u = surface.size_u / 2 - margin
    half_v = surface.size_v / 2 - margin
    r_max = min(half_u, half_v) - max(tool_size_u, tool_size_v) / 2
    step = (tool_size_u + tool_size_v) / 2 * (1.0 - overlap)
    if r_max <= 0 or step <= 0:
        return np.empty((0, 3)), np.empty((0, 2))

    # Archimedean: r(θ) = b * θ, with b chosen so each turn advances by `step`
    b = step / (2 * math.pi)
    theta_max = r_max / b
    n = max(8, int(theta_max / (2 * math.pi)) * samples_per_turn)
    thetas = np.linspace(theta_max, 0.0, n)  # outside-in
    rs = b * thetas

    pts, uv = [], []
    for r, t in zip(rs, thetas):
        su, sv = r * math.cos(t), r * math.sin(t)
        pts.append(surface.to_world(su, sv, standoff))
        uv.append([su, sv])
    return np.asarray(pts), np.asarray(uv)


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def coverage_fraction(
    surface_uv: np.ndarray,
    surface: Surface,
    tool_size_u: float,
    tool_size_v: float,
    margin: float,
    grid_res: float = 0.005,
) -> float:
    """Discretized estimate of fraction of the (margin-inset) surface area
    swept by the tool footprint along the path. Approximates the rectangular
    footprint as axis-aligned in (u, v); reasonable since the tool is held
    surface-aligned by the controller."""
    if len(surface_uv) == 0:
        return 0.0
    half_u = surface.size_u / 2 - margin
    half_v = surface.size_v / 2 - margin
    if half_u <= 0 or half_v <= 0:
        return 0.0
    us = np.arange(-half_u, half_u + grid_res, grid_res)
    vs = np.arange(-half_v, half_v + grid_res, grid_res)
    swept = np.zeros((len(vs), len(us)), dtype=bool)

    # densify the path so footprint stamps overlap between waypoints
    densified = [surface_uv[0]]
    for a, b in zip(surface_uv[:-1], surface_uv[1:]):
        seg = np.linalg.norm(b - a)
        n = max(1, int(math.ceil(seg / (min(tool_size_u, tool_size_v) / 4))))
        for k in range(1, n + 1):
            densified.append(a + (b - a) * (k / n))
    densified = np.asarray(densified)

    for su, sv in densified:
        u_lo, u_hi = su - tool_size_u / 2, su + tool_size_u / 2
        v_lo, v_hi = sv - tool_size_v / 2, sv + tool_size_v / 2
        ui = np.searchsorted(us, [u_lo, u_hi])
        vi = np.searchsorted(vs, [v_lo, v_hi])
        swept[vi[0]:vi[1], ui[0]:ui[1]] = True

    return float(swept.sum()) / float(swept.size)
