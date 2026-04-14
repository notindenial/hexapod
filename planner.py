"""
planner.py — Global path planner (ARA* stub).

ARA* (Anytime Repairing A*) finds an initial solution quickly with a high
inflation factor ε, then iteratively repairs it toward optimal as time allows.

This module is a STUB — the grid map, heuristic, and ARA* search loop are
left for future implementation. The interface is defined here so body.py and
main.py can already depend on it.

Planned interface:
    planner = Planner(cfg)
    planner.set_goal(x_mm, y_mm, theta_rad)
    planner.update(robot_x, robot_y, robot_theta)  # call each cycle
    vx, vy, omega = planner.velocity_command()     # consume in body.py
"""

from __future__ import annotations
from typing import Optional, Tuple


class Planner:
    """
    ARA* global path planner.

    State space: (x, y, theta) on a 2-D grid map.
    Output: velocity commands (vx_mm_s, vy_mm_s, omega_rad_s) for the body.

    TODO: implement occupancy grid, ARA* search, and path follower.
    """

    def __init__(self, cfg: dict) -> None:
        self._cfg = cfg
        self._goal: Optional[Tuple[float, float, float]] = None
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._omega: float = 0.0

    def set_goal(self, x_mm: float, y_mm: float, theta_rad: float = 0.0) -> None:
        """Set the navigation goal in world frame."""
        self._goal = (x_mm, y_mm, theta_rad)

    def update(self, robot_x: float, robot_y: float, robot_theta: float) -> None:
        """
        Replan or track the current path given the robot's pose.
        Updates internal velocity commands.
        """
        # TODO: run ARA* iteration, update path, compute tracking commands
        if self._goal is None:
            self._vx = self._vy = self._omega = 0.0
            return

        # Stub: naive point-to-point proportional controller
        gx, gy, gtheta = self._goal
        dx = gx - robot_x
        dy = gy - robot_y
        dist = (dx**2 + dy**2) ** 0.5

        if dist < 20.0:  # within 20 mm — goal reached
            self._vx = self._vy = self._omega = 0.0
            return

        # Proportional velocity toward goal
        k_v = 0.5   # mm/s per mm of error
        k_w = 1.0   # rad/s per rad of heading error

        import math
        heading_to_goal = math.atan2(dy, dx)
        heading_error = heading_to_goal - robot_theta
        # Wrap to [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        self._vx = k_v * dx
        self._vy = k_v * dy
        self._omega = k_w * heading_error

    def velocity_command(self) -> Tuple[float, float, float]:
        """Return current (vx_mm_s, vy_mm_s, omega_rad_s) command."""
        return (self._vx, self._vy, self._omega)
