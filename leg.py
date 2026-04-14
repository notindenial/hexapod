"""
leg.py — Single 3-DOF hexapod leg.

Joints (proximal → distal):
  pan  : yaw about vertical axis at leg mount
  hip  : pitch (upper link / coxa+femur)
  knee : pitch (lower link / tibia)

Coordinate frame (origin at leg mount point on body):
  x — forward (robot heading)
  y — outward (away from body centerline)
  z — up  (negative = down toward ground)

The Leg class is hardware-agnostic: it accepts a ServoDriver object that
implements the write_angles(pan, hip, knee) interface. This lets the same
class drive real PCA9685 servos or a simulation backend.
"""

from __future__ import annotations
import math
from typing import Protocol, Tuple


# ─── Helpers ────────────────────────────────────────────────────────────────

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _atan2d(y: float, x: float) -> float:
    return math.degrees(math.atan2(y, x))


def _acosd(x: float) -> float:
    return math.degrees(math.acos(_clamp(x, -1.0, 1.0)))


# ─── ServoDriver protocol ────────────────────────────────────────────────────

class ServoDriver(Protocol):
    """
    Minimal interface that both hardware (PCA9685) and sim drivers must satisfy.
    Angles are in degrees, in logical (unmirrrored) space — the driver handles
    any mirroring needed for left-side legs.
    """
    def write_angles(self, pan_deg: float, hip_deg: float, knee_deg: float) -> None: ...
    def set_pulse_range(self, pan: Tuple[int,int], hip: Tuple[int,int], knee: Tuple[int,int]) -> None: ...


# ─── Leg class ───────────────────────────────────────────────────────────────

class Leg:
    """
    3-DOF hexapod leg with:
      - Closed-form 2-link IK  (set_foot_xyz)
      - Direct angle interface  (set_angles)
      - Software rate limiting  (update(dt) → bool)
      - Joint-limit enforcement

    Usage (hardware tick loop):
        leg.set_foot_xyz(x, y, z)        # set target each cycle
        done = leg.update(dt)            # ramp toward target, send to servo
    """

    def __init__(
        self,
        driver: ServoDriver,
        *,
        name: str = "leg",
        # Link lengths (mm)
        L1_mm: float,           # coxa length (pan joint → hip joint)
        L2_mm: float,           # femur+tibia effective length
        # Neutral servo angles (degrees) — the "home" pose
        pan_neutral_deg: float = 90.0,
        hip_neutral_deg: float = 90.0,
        knee_neutral_deg: float = 90.0,
        # Joint limits in servo space (degrees)
        pan_limits: Tuple[float, float] = (0.0, 180.0),
        hip_limits: Tuple[float, float] = (10.0, 170.0),
        knee_limits: Tuple[float, float] = (10.0, 170.0),
        # Mount point in body frame (mm) — used by Body for world→leg transforms
        mount_xyz_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        # Rate limiting
        max_speed_deg_s: float = 120.0,
        angle_epsilon_deg: float = 0.5,
    ) -> None:
        self.name = name
        self.driver = driver

        self.L1 = float(L1_mm)
        self.L2 = float(L2_mm)

        self.pan_neutral = float(pan_neutral_deg)
        self.hip_neutral = float(hip_neutral_deg)
        self.knee_neutral = float(knee_neutral_deg)

        self.pan_limits = pan_limits
        self.hip_limits = hip_limits
        self.knee_limits = knee_limits

        self.mount_xyz_mm = tuple(mount_xyz_mm)

        self.max_speed = float(max_speed_deg_s)
        self.epsilon = float(angle_epsilon_deg)

        # Current commanded angles (what was last sent to driver)
        self._cmd = [self.pan_neutral, self.hip_neutral, self.knee_neutral]
        # Target angles
        self._tgt = [self.pan_neutral, self.hip_neutral, self.knee_neutral]

        # Last foot target for external reference
        self.foot_xyz_mm: Tuple[float, float, float] = (0.0, 0.0, -(self.L1 + self.L2))

        self.home(immediate=True)

    # ── Angle interface ───────────────────────────────────────────────────────

    def home(self, *, immediate: bool = False) -> None:
        """Move to neutral pose."""
        self.set_angles(self.pan_neutral, self.hip_neutral, self.knee_neutral, immediate=immediate)

    def set_angles(
        self,
        pan_deg: float,
        hip_deg: float,
        knee_deg: float,
        *,
        immediate: bool = False,
    ) -> None:
        """Set target joint angles (clamped to limits). immediate=True skips ramping."""
        pan  = _clamp(pan_deg,  *self.pan_limits)
        hip  = _clamp(hip_deg,  *self.hip_limits)
        knee = _clamp(knee_deg, *self.knee_limits)

        self._tgt = [pan, hip, knee]

        if immediate:
            self._cmd = [pan, hip, knee]
            self.driver.write_angles(*self._cmd)

    # ── IK interface ─────────────────────────────────────────────────────────

    def set_foot_xyz(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        *,
        immediate: bool = False,
    ) -> Tuple[float, float, float]:
        """
        Set foot target in leg-local frame and compute IK.
        Returns the resulting (pan, hip, knee) target angles in degrees.
        """
        pan, hip, knee = self.ik(x_mm, y_mm, z_mm)
        self.foot_xyz_mm = (float(x_mm), float(y_mm), float(z_mm))
        self.set_angles(pan, hip, knee, immediate=immediate)
        return (pan, hip, knee)

    def ik(self, x_mm: float, y_mm: float, z_mm: float) -> Tuple[float, float, float]:
        """
        Closed-form 2-link IK.

        Step 1: Pan — point the leg toward (x, y) in the horizontal plane.
        Step 2: Solve hip + knee in the (r, z) plane using the law of cosines.

        Returns servo-space (pan, hip, knee) angles in degrees.
        The neutral angles define the zero reference:
            pan  = pan_neutral  + pan_geometry
            hip  = hip_neutral  + hip_geometry
            knee = knee_neutral + knee_geometry
        """
        x, y, z = float(x_mm), float(y_mm), float(z_mm)

        # Pan: point toward (x, y)
        pan_geom = _atan2d(y, x)
        pan = self.pan_neutral + pan_geom

        # Reduce to planar problem
        r = math.hypot(x, y)
        D = math.hypot(r, z)
        D = _clamp(D, abs(self.L1 - self.L2) + 1e-6, (self.L1 + self.L2) - 1e-6)

        # Knee (elbow-down configuration)
        cos_knee_inner = (self.L1**2 + self.L2**2 - D**2) / (2.0 * self.L1 * self.L2)
        knee_inner = _acosd(cos_knee_inner)
        knee_geom = 180.0 - knee_inner

        # Hip
        alpha = _atan2d(z, r)
        cos_beta = (self.L1**2 + D**2 - self.L2**2) / (2.0 * self.L1 * D)
        hip_geom = alpha - _acosd(cos_beta)

        pan  = _clamp(self.pan_neutral  + pan_geom,  *self.pan_limits)
        hip  = _clamp(self.hip_neutral  + hip_geom,  *self.hip_limits)
        knee = _clamp(self.knee_neutral + knee_geom, *self.knee_limits)

        return (pan, hip, knee)

    def fk(self, pan_deg: float, hip_deg: float, knee_deg: float) -> Tuple[float, float, float]:
        """
        Forward kinematics: servo angles → foot position in leg-local frame (mm).

        Uses the same 2-link planar model as the IK solver.
        Returns (x, y, z).
        """
        pan_geom  = pan_deg  - self.pan_neutral
        hip_geom  = hip_deg  - self.hip_neutral
        knee_geom = knee_deg - self.knee_neutral

        yaw_rad = math.radians(pan_geom)

        knee_inner = 180.0 - knee_geom
        knee_inner_rad = math.radians(knee_inner)
        hip_rad = math.radians(hip_geom)

        # End-effector in (r, z) plane
        # r = L1*cos(hip) + L2*cos(hip - knee_geom) using signed angles from neutral
        hip_a = math.radians(hip_geom)
        knee_a = math.radians(knee_geom)

        r = self.L1 * math.cos(hip_a) + self.L2 * math.cos(hip_a - knee_a)
        z = self.L1 * math.sin(hip_a) + self.L2 * math.sin(hip_a - knee_a)

        x = r * math.cos(yaw_rad)
        y = r * math.sin(yaw_rad)

        return (x, y, z)

    # ── Motion update (rate limiting) ─────────────────────────────────────────

    def update(self, dt: float) -> bool:
        """
        Advance commanded angles toward targets at max_speed_deg_s.
        Writes to servo driver each call.

        Returns True when all joints are within epsilon of their targets.
        """
        if dt <= 0.0:
            return self.is_at_target()

        max_step = self.max_speed * dt

        def _step(curr: float, tgt: float) -> float:
            err = tgt - curr
            if abs(err) <= max_step:
                return tgt
            return curr + math.copysign(max_step, err)

        self._cmd = [_step(c, t) for c, t in zip(self._cmd, self._tgt)]
        self.driver.write_angles(*self._cmd)
        return self.is_at_target()

    def is_at_target(self) -> bool:
        return all(abs(c - t) <= self.epsilon for c, t in zip(self._cmd, self._tgt))

    # ── Readbacks ─────────────────────────────────────────────────────────────

    @property
    def targets(self) -> Tuple[float, float, float]:
        return tuple(self._tgt)

    @property
    def commands(self) -> Tuple[float, float, float]:
        return tuple(self._cmd)
