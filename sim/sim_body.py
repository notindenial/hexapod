"""
sim/sim_body.py — Visualisation wrapper for Body in simulation.

Wraps a Body instance and renders the hexapod state after each update().
Currently supports a simple matplotlib stick-figure view.
Pybullet integration is planned (see urdf/ stub).

Usage:
    from body import Body
    from sim.sim_body import SimBody

    body = Body(legs, cfg)
    sim  = SimBody(body)

    while True:
        body.update(dt)
        sim.render()
"""

from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from body import Body


class SimBody:
    """
    Visualisation wrapper around a Body instance.

    render() should be called after body.update() each tick.
    """

    def __init__(self, body: "Body", backend: str = "matplotlib") -> None:
        self.body = body
        self.backend = backend
        self._fig = None
        self._ax  = None

        if backend == "matplotlib":
            self._init_matplotlib()
        elif backend == "pybullet":
            self._init_pybullet()
        else:
            raise ValueError(f"Unknown sim backend: {backend!r}. Use 'matplotlib' or 'pybullet'.")

    # ── Matplotlib backend ────────────────────────────────────────────────────

    def _init_matplotlib(self) -> None:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
            self._plt = plt
            self._fig = plt.figure(figsize=(8, 6))
            self._ax  = self._fig.add_subplot(111, projection="3d")
            plt.ion()
            plt.show()
        except ImportError:
            print("matplotlib not installed — SimBody in no-op mode.")
            self._plt = None

    def render(self) -> None:
        if self.backend == "matplotlib":
            self._render_matplotlib()

    def _render_matplotlib(self) -> None:
        if self._plt is None or self._ax is None:
            return

        ax = self._ax
        ax.cla()
        ax.set_xlim(-150, 150)
        ax.set_ylim(-150, 150)
        ax.set_zlim(-150, 50)
        ax.set_xlabel("X (forward)")
        ax.set_ylabel("Y (left)")
        ax.set_zlabel("Z (up)")
        ax.set_title("Hexapod Simulation")

        import numpy as np
        for name, leg in self.body.legs.items():
            mount = np.array(leg.mount_xyz_mm)
            pan, hip, knee = leg.commands
            fx, fy, fz = leg.fk(pan, hip, knee)
            foot_world = mount + np.array([fx, fy, fz])

            # Draw stick: mount → foot
            xs = [mount[0], foot_world[0]]
            ys = [mount[1], foot_world[1]]
            zs = [mount[2], foot_world[2]]
            ax.plot(xs, ys, zs, "b-o", markersize=4)
            ax.text(foot_world[0], foot_world[1], foot_world[2], name, fontsize=7)

        # Body center
        ax.scatter([0], [0], [0], c="r", s=60, zorder=5)

        self._plt.pause(0.001)

    # ── Pybullet backend (stub) ────────────────────────────────────────────────

    def _init_pybullet(self) -> None:
        # TODO: load URDF from sim/urdf/, connect to GUI
        raise NotImplementedError("Pybullet backend not yet implemented. See sim/urdf/.")
