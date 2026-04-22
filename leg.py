"""
leg.py — Single 3-DOF hexapod leg (3-link kinematic chain).

Kinematic chain (proximal → distal):
    pan (vertical axis, at mount)
      → coxa offset (coxa_lateral outward, coxa_drop down, FIXED)
      → hip (horizontal axis ⊥ pan)
      → femur (length femur_mm)
      → knee (horizontal axis ‖ hip, translated to knee axis)
      → tibia (length tibia_mm)
      → foot tip

Leg-local frame (origin at the pan axis, at the leg mount point on the body):
    +x : radially outward (leg extends in +x at neutral pan)
    +y : tangent to pan rotation  (pan rotates +x toward +y)
    +z : up

Angle conventions (logical / servo degrees):
    pan   = pan_neutral + pan_geom
            pan_geom = azimuth of foot in leg-local xy plane.
            pan_geom = 0  →  leg extends along +x.
    hip   = hip_neutral + hip_geom
            hip_geom = femur elevation from horizontal.
            hip_geom = 0  →  femur horizontal (along +r direction).
            hip_geom > 0  →  femur tilted up (toward +z).
    knee  = interior angle at the knee vertex, in degrees.
            knee = 180  →  straight leg (tibia colinear with femur).
            knee = 90   →  L-bent leg (tibia perpendicular to femur).
            knee =  0   →  fully folded (tibia against femur).
            With standing feet below the hip, the "knee-up" IK branch
            places the knee above the hip-to-foot line, so the tibia
            bends downward from the knee.

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
    Angles are in degrees, in logical (unmirrored) space — the driver handles
    any mirroring needed for left-side legs.
    """
    def write_angles(self, pan_deg: float, hip_deg: float, knee_deg: float) -> None: ...
    def set_pulse_range(self, pan: Tuple[int,int], hip: Tuple[int,int], knee: Tuple[int,int]) -> None: ...


# ─── Leg class ───────────────────────────────────────────────────────────────

Vec3 = Tuple[float, float, float]


class Leg:
    """
    3-DOF hexapod leg with:
      - Closed-form 3-link IK  (set_foot_xyz)
      - Direct angle interface (set_angles)
      - Software rate limiting (update(dt) → bool)
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
        # Link geometry (mm)
        coxa_lateral_mm: float,          # pan axis → hip axis, horizontal
        coxa_drop_mm: float,             # hip axis below pan axis (vertical)
        femur_mm: float,                 # hip axis → knee axis
        tibia_mm: float,                 # knee axis → foot tip
        # Neutral servo angles (degrees)
        pan_neutral_deg: float = 90.0,
        hip_neutral_deg: float = 90.0,
        # Joint limits in servo space (degrees)
        pan_limits: Tuple[float, float] = (0.0, 180.0),
        hip_limits: Tuple[float, float] = (10.0, 170.0),
        knee_limits: Tuple[float, float] = (10.0, 170.0),
        # Mount point in body frame (mm) — pan-axis origin in body coordinates
        mount_xyz_mm: Vec3 = (0.0, 0.0, 0.0),
        # Rotation from body +x to leg-local +x about z (degrees)
        mount_yaw_deg: float = 0.0,
        # Rate limiting
        max_speed_deg_s: float = 120.0,
        angle_epsilon_deg: float = 0.5,
    ) -> None:
        self.name = name
        self.driver = driver

        self.coxa_lateral = float(coxa_lateral_mm)
        self.coxa_drop    = float(coxa_drop_mm)
        self.femur        = float(femur_mm)
        self.tibia        = float(tibia_mm)

        self.pan_neutral = float(pan_neutral_deg)
        self.hip_neutral = float(hip_neutral_deg)

        self.pan_limits  = pan_limits
        self.hip_limits  = hip_limits
        self.knee_limits = knee_limits

        self.mount_xyz_mm = tuple(mount_xyz_mm)
        self.mount_yaw_deg = float(mount_yaw_deg)

        self.max_speed = float(max_speed_deg_s)
        self.epsilon   = float(angle_epsilon_deg)

        # Knee servo angle is interpreted directly as the interior knee angle.
        # "Neutral" knee = 90° (L-bent) is consistent with config's neutral pose.
        self._pan_cmd  = self.pan_neutral
        self._hip_cmd  = self.hip_neutral
        self._knee_cmd = 90.0

        self._pan_tgt  = self.pan_neutral
        self._hip_tgt  = self.hip_neutral
        self._knee_tgt = 90.0

        # Last commanded foot target (leg-local frame, mm)
        self.foot_xyz_mm: Vec3 = self.fk(self._pan_cmd, self._hip_cmd, self._knee_cmd)

        self.home(immediate=True)

    # ── Angle interface ───────────────────────────────────────────────────────

    def home(self, *, immediate: bool = False) -> None:
        """Move to neutral pose (pan=pan_neutral, hip=hip_neutral, knee=90°)."""
        self.set_angles(self.pan_neutral, self.hip_neutral, 90.0, immediate=immediate)

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

        self._pan_tgt, self._hip_tgt, self._knee_tgt = pan, hip, knee

        if immediate:
            self._pan_cmd, self._hip_cmd, self._knee_cmd = pan, hip, knee
            self.driver.write_angles(pan, hip, knee)

    # ── IK / FK ──────────────────────────────────────────────────────────────

    def set_foot_xyz(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        *,
        immediate: bool = False,
    ) -> Tuple[float, float, float]:
        """Set foot target in leg-local frame; solve IK; command angles."""
        pan, hip, knee = self.ik(x_mm, y_mm, z_mm)
        self.foot_xyz_mm = (float(x_mm), float(y_mm), float(z_mm))
        self.set_angles(pan, hip, knee, immediate=immediate)
        return (pan, hip, knee)

    def ik(self, x_mm: float, y_mm: float, z_mm: float) -> Tuple[float, float, float]:
        """
        Closed-form 3-link IK. Returns (pan, hip, knee) in servo degrees.

        Knee-up branch (standard hexapod): the knee sits above the line from
        the hip axis to the foot, so the femur tilts upward from the hip and
        the tibia folds downward from the knee.
        """
        x, y, z = float(x_mm), float(y_mm), float(z_mm)

        # Step 1: pan — point the leg at the foot's azimuth
        pan_geom = _atan2d(y, x)
        pan = self.pan_neutral + pan_geom

        # Step 2: reduce to the (r, z) plane after pan rotation
        r = math.hypot(x, y)

        # Foot relative to hip axis
        dr = r - self.coxa_lateral
        dz = z - (-self.coxa_drop)
        D  = math.hypot(dr, dz)

        # Reachable annulus: |femur - tibia| < D < femur + tibia
        d_min = abs(self.femur - self.tibia) + 1e-6
        d_max = (self.femur + self.tibia) - 1e-6
        D = _clamp(D, d_min, d_max)

        # Step 3: interior knee angle via law of cosines
        cos_knee = (self.femur**2 + self.tibia**2 - D**2) / (2.0 * self.femur * self.tibia)
        knee = _acosd(cos_knee)   # interior angle at knee vertex, degrees

        # Step 4: hip elevation — knee-up branch (hip_geom = alpha + beta)
        alpha = _atan2d(dz, dr)
        cos_beta = (self.femur**2 + D**2 - self.tibia**2) / (2.0 * self.femur * D)
        beta = _acosd(cos_beta)
        hip_geom = alpha + beta
        hip = self.hip_neutral + hip_geom

        # Clamp to servo limits
        pan  = _clamp(pan,  *self.pan_limits)
        hip  = _clamp(hip,  *self.hip_limits)
        knee = _clamp(knee, *self.knee_limits)

        return (pan, hip, knee)

    def fk(self, pan_deg: float, hip_deg: float, knee_deg: float) -> Vec3:
        """Forward kinematics: servo angles → foot position in leg-local frame (mm)."""
        _, _, _, foot = self.fk_chain(pan_deg, hip_deg, knee_deg)
        return foot

    def fk_chain(
        self, pan_deg: float, hip_deg: float, knee_deg: float
    ) -> Tuple[Vec3, Vec3, Vec3, Vec3]:
        """
        Forward kinematics for every joint in the chain, in leg-local frame (mm).

        Returns:
            (pan_axis_pos, hip_axis_pos, knee_axis_pos, foot_tip_pos)
            pan_axis_pos is always (0, 0, 0) — included for convenience when
            drawing the full chain.
        """
        pan_rad  = math.radians(pan_deg - self.pan_neutral)
        hip_rad  = math.radians(hip_deg - self.hip_neutral)
        knee_rad = math.radians(knee_deg)   # interior angle

        # Pan axis at leg-local origin
        p0 = (0.0, 0.0, 0.0)

        # Hip axis: coxa offset, rotated by pan about z
        # In coxa frame (before pan): offset is (coxa_lateral, 0, -coxa_drop).
        cp, sp = math.cos(pan_rad), math.sin(pan_rad)
        h_x = cp * self.coxa_lateral
        h_y = sp * self.coxa_lateral
        h_z = -self.coxa_drop
        p1 = (h_x, h_y, h_z)

        # Femur vector in the (r, z) plane, then rotated into pan frame.
        # In the coxa-after-pan frame: femur_local = (femur*cos(hip), 0, femur*sin(hip))
        ch, sh = math.cos(hip_rad), math.sin(hip_rad)
        fr = self.femur * ch   # along +r
        fz = self.femur * sh   # along +z
        # Lift into world leg-local: rotate (fr, 0, fz) by pan about z
        k_x = h_x + cp * fr
        k_y = h_y + sp * fr
        k_z = h_z + fz
        p2 = (k_x, k_y, k_z)

        # Tibia direction in the (r, z) plane:
        #   tibia_angle_rz = hip_rad - (π - knee_rad)
        # (Knee-up convention: tibia rotates CW from femur by (π - knee_inner).)
        ta = hip_rad - (math.pi - knee_rad)
        cta, sta = math.cos(ta), math.sin(ta)
        tr = self.tibia * cta
        tz = self.tibia * sta
        f_x = k_x + cp * tr
        f_y = k_y + sp * tr
        f_z = k_z + tz
        p3 = (f_x, f_y, f_z)

        return (p0, p1, p2, p3)

    # ── Motion update (rate limiting) ─────────────────────────────────────────

    def update(self, dt: float) -> bool:
        """Advance commanded angles toward targets at max_speed_deg_s."""
        if dt <= 0.0:
            return self.is_at_target()

        max_step = self.max_speed * dt

        def _step(curr: float, tgt: float) -> float:
            err = tgt - curr
            if abs(err) <= max_step:
                return tgt
            return curr + math.copysign(max_step, err)

        self._pan_cmd  = _step(self._pan_cmd,  self._pan_tgt)
        self._hip_cmd  = _step(self._hip_cmd,  self._hip_tgt)
        self._knee_cmd = _step(self._knee_cmd, self._knee_tgt)

        self.driver.write_angles(self._pan_cmd, self._hip_cmd, self._knee_cmd)
        return self.is_at_target()

    def is_at_target(self) -> bool:
        return (
            abs(self._pan_cmd  - self._pan_tgt)  <= self.epsilon
            and abs(self._hip_cmd  - self._hip_tgt)  <= self.epsilon
            and abs(self._knee_cmd - self._knee_tgt) <= self.epsilon
        )

    # ── Readbacks ─────────────────────────────────────────────────────────────

    @property
    def targets(self) -> Tuple[float, float, float]:
        return (self._pan_tgt, self._hip_tgt, self._knee_tgt)

    @property
    def commands(self) -> Tuple[float, float, float]:
        return (self._pan_cmd, self._hip_cmd, self._knee_cmd)
