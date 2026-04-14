"""
body.py — Hexapod body: pose control and gait coordination.

The Body class owns all 6 Leg instances and is responsible for:
  1. Maintaining a body pose (translation + rotation) relative to world.
  2. Transforming desired foot positions from world frame → body frame →
     leg-local frame before passing them down to each Leg.
  3. Executing gaits by coordinating swing/stance phases across legs.

Coordinate frames:
  World  : fixed to ground, +x forward, +y left, +z up.
  Body   : origin at body center, aligned with world at neutral pose.
           Shifted by body_pos and rotated by body_rpy.
  Leg    : origin at leg mount point on the body, local x/y/z per leg.py.

Gait state machine (per leg):
  STANCE — foot on ground, body moves over it.
  SWING  — foot lifted and moved to next contact point.
"""

from __future__ import annotations
import math
import time
from enum import Enum, auto
from typing import Dict, List, Tuple

import numpy as np

from leg import Leg


# ─── Types ───────────────────────────────────────────────────────────────────

class Phase(Enum):
    STANCE = auto()
    SWING  = auto()


# ─── Coordinate helpers ──────────────────────────────────────────────────────

def _rot_z(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _rot_x(roll_rad: float) -> np.ndarray:
    c, s = math.cos(roll_rad), math.sin(roll_rad)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rot_y(pitch_rad: float) -> np.ndarray:
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def body_transform(
    pos: np.ndarray,
    rpy_rad: Tuple[float, float, float],
) -> np.ndarray:
    """4×4 homogeneous transform: world ← body."""
    R = _rot_z(rpy_rad[2]) @ _rot_y(rpy_rad[1]) @ _rot_x(rpy_rad[0])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = pos
    return T


# ─── Body class ──────────────────────────────────────────────────────────────

class Body:
    """
    Hexapod body controller.

    Typical usage:
        body = Body(legs, cfg)
        body.stand()
        while True:
            body.walk(vx=0.05, vy=0.0, omega=0.0)
            body.update(dt)
    """

    # Tripod gait groups — Group A and B alternate swing/stance
    TRIPOD_A = ["RF", "LM", "LB"]
    TRIPOD_B = ["LF", "RM", "RB"]

    def __init__(self, legs: Dict[str, Leg], cfg: dict) -> None:
        self.legs = legs
        self._cfg = cfg

        gait_cfg = cfg.get("gait", {}).get("tripod", {})
        self._lift_height = float(gait_cfg.get("lift_height_mm", 30.0))
        self._step_length = float(gait_cfg.get("step_length_mm", 40.0))
        self._swing_ticks = int(gait_cfg.get("swing_ticks", 20))

        # Body pose in world frame
        self._body_pos = np.zeros(3)          # x, y, z (mm)
        self._body_rpy = [0.0, 0.0, 0.0]     # roll, pitch, yaw (rad)

        # Default neutral foot positions in leg-local frame (set by stand())
        self._neutral_foot: Dict[str, np.ndarray] = {}

        # Gait state
        self._phase: Dict[str, Phase] = {name: Phase.STANCE for name in legs}
        self._swing_tick: Dict[str, int] = {name: 0 for name in legs}
        self._swing_start: Dict[str, np.ndarray] = {}
        self._swing_end: Dict[str, np.ndarray] = {}

        # Velocity commands (world frame, m/s → mm/tick handled in update)
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._omega: float = 0.0  # rad/s yaw rate

        # Which tripod group is currently swinging (0 = A, 1 = B)
        self._active_group: int = 0
        self._gait_tick: int = 0

    # ── Body pose ─────────────────────────────────────────────────────────────

    def set_body_pose(
        self,
        x_mm: float = 0.0,
        y_mm: float = 0.0,
        z_mm: float = 0.0,
        roll_rad: float = 0.0,
        pitch_rad: float = 0.0,
        yaw_rad: float = 0.0,
        *,
        immediate: bool = False,
    ) -> None:
        """
        Set body pose relative to default standing pose.
        Recomputes foot targets for all legs in stance.
        """
        self._body_pos = np.array([x_mm, y_mm, z_mm], dtype=float)
        self._body_rpy = [roll_rad, pitch_rad, yaw_rad]
        self._apply_body_pose(immediate=immediate)

    def _apply_body_pose(self, *, immediate: bool = False) -> None:
        """Reproject neutral foot positions through current body transform."""
        T_wb = body_transform(self._body_pos, self._body_rpy)
        T_bw = np.linalg.inv(T_wb)

        for name, leg in self.legs.items():
            if name not in self._neutral_foot:
                continue
            foot_world = self._neutral_foot[name]
            mount = np.array(leg.mount_xyz_mm, dtype=float)

            # Foot in body frame
            foot_body = T_bw[:3, :3] @ foot_world + T_bw[:3, 3]
            # Foot in leg-local frame
            foot_local = foot_body - mount

            leg.set_foot_xyz(*foot_local, immediate=immediate)

    # ── Standing / sitting ────────────────────────────────────────────────────

    def stand(self, *, dt: float = 0.02, timeout_s: float = 3.0) -> None:
        """
        Smoothly move all legs to standing pose and record neutral foot positions.
        """
        stand_cfg = self._cfg.get("poses", {}).get("stand", {})
        pan  = float(stand_cfg.get("pan",  90.0))
        hip  = float(stand_cfg.get("hip",  89.0))
        knee = float(stand_cfg.get("knee", 10.0))

        for leg in self.legs.values():
            leg.set_angles(pan, hip, knee, immediate=False)
            # Compute and cache the foot position this corresponds to
            fx, fy, fz = leg.fk(pan, hip, knee)
            mount = np.array(leg.mount_xyz_mm, dtype=float)
            # Foot in world frame = mount + local foot (at neutral body pose)
            self._neutral_foot[leg.name] = mount + np.array([fx, fy, fz])

        self._await_all(dt=dt, timeout_s=timeout_s)

    def sit(self, *, dt: float = 0.02, timeout_s: float = 3.0) -> None:
        """Move all legs to sitting pose."""
        sit_cfg = self._cfg.get("poses", {}).get("sit", {})
        pan  = float(sit_cfg.get("pan",  90.0))
        hip  = float(sit_cfg.get("hip",  165.0))
        knee = float(sit_cfg.get("knee", 0.0))

        for leg in self.legs.values():
            leg.set_angles(pan, hip, knee, immediate=False)
        self._await_all(dt=dt, timeout_s=timeout_s)

    # ── Velocity command ──────────────────────────────────────────────────────

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        """
        Set body velocity command.
        vx, vy : forward/lateral speed (mm/s in body frame)
        omega  : yaw rate (rad/s)
        """
        self._vx = vx
        self._vy = vy
        self._omega = omega

    # ── Tripod gait ───────────────────────────────────────────────────────────

    def tripod_step(self) -> None:
        """
        Initiate one tripod half-step: swing the inactive group forward.
        Call this externally or let update() handle it automatically.
        """
        swing_group = self.TRIPOD_A if self._active_group == 0 else self.TRIPOD_B
        for name in swing_group:
            if name not in self.legs:
                continue
            self._phase[name] = Phase.SWING
            self._swing_tick[name] = 0
            leg = self.legs[name]
            # Swing start = current foot position
            self._swing_start[name] = np.array(leg.foot_xyz_mm, dtype=float)
            # Swing end = neutral + forward offset from velocity
            self._swing_end[name] = np.array(self._neutral_foot.get(
                name, np.array(leg.foot_xyz_mm)
            ), dtype=float)

    # ── Main update ───────────────────────────────────────────────────────────

    def update(self, dt: float) -> None:
        """
        Advance all legs one control tick.
        - Updates swing trajectories.
        - Applies stance body motion.
        - Calls leg.update(dt) for all legs.
        """
        for name, leg in self.legs.items():
            if self._phase[name] == Phase.SWING:
                self._update_swing(name, leg, dt)
            else:
                self._update_stance(name, leg, dt)

            leg.update(dt)

    def _update_swing(self, name: str, leg: Leg, dt: float) -> None:
        tick = self._swing_tick[name]
        if tick >= self._swing_ticks:
            self._phase[name] = Phase.STANCE
            # Switch active group when all swing legs have landed
            swing_group = self.TRIPOD_A if self._active_group == 0 else self.TRIPOD_B
            if all(self._phase[n] == Phase.STANCE for n in swing_group if n in self.legs):
                self._active_group ^= 1
            return

        t = tick / self._swing_ticks
        start = self._swing_start.get(name, np.zeros(3))
        end   = self._swing_end.get(name, np.zeros(3))

        # Cubic interpolation for x/y, raised sine arc for z
        xy = start + (end - start) * (3 * t**2 - 2 * t**3)
        z  = start[2] + self._lift_height * math.sin(math.pi * t)

        mount = np.array(leg.mount_xyz_mm, dtype=float)
        local = xy - mount
        local[2] = z - mount[2]

        leg.set_foot_xyz(local[0], local[1], local[2])
        self._swing_tick[name] += 1

    def _update_stance(self, name: str, leg: Leg, dt: float) -> None:
        # During stance the foot is fixed; body motion is handled by
        # set_body_pose() from the velocity integrator (future: gait planner).
        pass

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _await_all(self, *, dt: float, timeout_s: float) -> None:
        t0 = time.time()
        while not all(leg.is_at_target() for leg in self.legs.values()):
            for leg in self.legs.values():
                leg.update(dt)
            time.sleep(dt)
            if (time.time() - t0) > timeout_s:
                print("WARNING: Body._await_all timed out")
                break
