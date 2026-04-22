"""
sim/sim_body.py — Hexapod 3D stick-figure visualization (matplotlib).

Provides:
    render_hexapod(ax, legs, ...)   — stateless renderer; draws the robot
                                       into an existing mplot3d Axes given a
                                       dict of Leg instances. Uses each leg's
                                       currently commanded angles.
    SimBody(body)                   — thin wrapper around a Body that owns a
                                       figure/axes and re-renders each tick.
"""

from __future__ import annotations
import math
from typing import Dict, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from leg import Leg
    from body import Body


# ─── Geometry helpers ────────────────────────────────────────────────────────

def _rot_z(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _leg_chain_body_frame(leg: "Leg", pan_deg: float, hip_deg: float, knee_deg: float):
    """
    Joint chain for one leg in BODY frame.
    Returns a list of 4 np.ndarray(3) points: [pan_axis, hip_axis, knee_axis, foot].
    """
    chain_local = leg.fk_chain(pan_deg, hip_deg, knee_deg)
    R = _rot_z(math.radians(leg.mount_yaw_deg))
    mount = np.asarray(leg.mount_xyz_mm, dtype=float)
    return [mount + R @ np.asarray(p, dtype=float) for p in chain_local]


# ─── Rendering ───────────────────────────────────────────────────────────────

_THORAX_ORDER = ["LF", "LM", "LB", "RB", "RM", "RF"]


def render_hexapod(
    ax,
    legs: Dict[str, "Leg"],
    *,
    show_thorax: bool = True,
    show_ground: bool = True,
    ground_z_mm: float = -50.0,
    title: str = "Hexapod",
    axis_span_mm: float = 220.0,
) -> None:
    """Draw a hexapod into an existing Axes3D. Stateless; clears axis first."""
    ax.cla()
    s = float(axis_span_mm)
    ax.set_xlim(-s, s)
    ax.set_ylim(-s, s)
    ax.set_zlim(ground_z_mm - 30.0, 120.0)
    ax.set_xlabel("X (forward, mm)")
    ax.set_ylabel("Y (left, mm)")
    ax.set_zlabel("Z (up, mm)")
    ax.set_title(title)
    try:
        ax.set_box_aspect((1.0, 1.0, (150.0 + ground_z_mm + 30.0) / (2.0 * s)))
    except Exception:
        pass

    # Ground plane (translucent)
    if show_ground:
        gx = np.array([[-s, s], [-s, s]])
        gy = np.array([[-s, -s], [s, s]])
        gz = np.full_like(gx, ground_z_mm, dtype=float)
        ax.plot_surface(gx, gy, gz, color="tan", alpha=0.15, edgecolor="none")

    # Thorax outline (hexagon through mount points)
    if show_thorax:
        pts = [legs[n].mount_xyz_mm for n in _THORAX_ORDER if n in legs]
        if len(pts) >= 3:
            loop = pts + [pts[0]]
            xs = [p[0] for p in loop]
            ys = [p[1] for p in loop]
            zs = [p[2] for p in loop]
            ax.plot(xs, ys, zs, color="black", linewidth=1.5)
        ax.scatter([0.0], [0.0], [0.0], color="red", s=50, zorder=10, label="body center")

    # Each leg: 3-link chain
    for name, leg in legs.items():
        pan, hip, knee = leg.commands
        chain = _leg_chain_body_frame(leg, pan, hip, knee)
        xs = [p[0] for p in chain]
        ys = [p[1] for p in chain]
        zs = [p[2] for p in chain]
        ax.plot(xs, ys, zs, "-o", color="steelblue", linewidth=2, markersize=4)
        # Foot label
        ax.text(chain[-1][0], chain[-1][1], chain[-1][2] - 8.0, name, fontsize=8)


# ─── SimBody (Body-driven live viz) ──────────────────────────────────────────

class SimBody:
    """
    Visualization wrapper around a Body. Opens a matplotlib 3D figure and
    re-renders on render(). Call after each body.update(dt).
    """

    def __init__(self, body: "Body", *, ground_z_mm: float = -50.0) -> None:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

        self.body = body
        self.ground_z_mm = float(ground_z_mm)
        self._plt = plt
        self._fig = plt.figure(figsize=(9, 7))
        self._ax = self._fig.add_subplot(111, projection="3d")
        plt.ion()
        plt.show()

    def render(self) -> None:
        render_hexapod(self._ax, self.body.legs, ground_z_mm=self.ground_z_mm)
        self._plt.pause(0.001)
