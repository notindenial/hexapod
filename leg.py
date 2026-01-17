# leg.py
# 3DOF hexapod leg: yaw (pan) + hip pitch + knee pitch
#
# Internal trig uses radians, but hidden behind *_deg helpers.
# Stores a local foot position (x,y,z) in mm relative to yaw joint frame.
#
# Coordinate convention (pick and keep consistent):
#   x: forward
#   y: outward (to the side of the body)
#   z: up (so "down" is negative z)
#

import math
from dataclasses import dataclass
from typing import Tuple, Optional

# ----------------------- Degree helpers (internal) -----------------------
def sin_deg(d: float) -> float:
    return math.sin(math.radians(d))

def cos_deg(d: float) -> float:
    return math.cos(math.radians(d))

def atan2_deg(y: float, x: float) -> float:
    return math.degrees(math.atan2(y, x))

def acos_deg(x: float) -> float:
    return math.degrees(math.acos(x))

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# ----------------------- Config dataclasses -----------------------
@dataclass(frozen=True)
class LegGeometry:
    # Link lengths (mm)
    L1: float  # hip pitch -> knee
    L2: float  # knee -> foot

    # Optional offsets from yaw axis to hip pitch axis (mm)
    # Use these if your pitch joint is not exactly on the yaw axis.
    hip_offset_y: float = 0.0
    hip_offset_z: float = 0.0


@dataclass(frozen=True)
class ServoChannels:
    yaw: int
    hip: int
    knee: int


@dataclass(frozen=True)
class PulseRange:
    min_us: int = 500
    max_us: int = 2500


@dataclass(frozen=True)
class AngleLimitsDeg:
    yaw: Tuple[float, float] = (0.0, 180.0)
    hip: Tuple[float, float] = (0.0, 180.0)
    knee: Tuple[float, float] = (0.0, 180.0)


@dataclass(frozen=True)
class AngleOffsetsDeg:
    # Add these AFTER computing the geometric joint angles.
    # Use to account for horn mounting / “90 isn’t neutral” realities.
    yaw: float = 90.0   # common: yaw neutral points forward/outward
    hip: float = 90.0   # common: hip 90 = neutral
    knee: float = 90.0  # common: knee 90 = neutral


@dataclass(frozen=True)
class MirrorConfig:
    # Useful for left/right symmetry depending on how servos are mounted.
    # If a joint moves opposite direction than expected, flip it.
    yaw: bool = False
    hip: bool = False
    knee: bool = False


# ----------------------- Leg class -----------------------
class Leg:
    """
    A single 3-DOF leg with:
      - yaw (pan) about vertical axis
      - hip pitch
      - knee pitch

    Inputs/outputs:
      - Angles in degrees
      - Positions in mm

    Main methods:
      - set_angles_deg(yaw, hip, knee)
      - set_foot_local_mm(x, y, z)  -> runs IK and moves servos
      - ik_deg(x, y, z)             -> returns (yaw, hip, knee) in degrees
      - fk_mm(yaw, hip, knee)       -> returns (x, y, z) in mm
    """

    def __init__(
        self,
        kit,  # adafruit_servokit.ServoKit
        channels: ServoChannels,
        geom: LegGeometry,
        limits: AngleLimitsDeg = AngleLimitsDeg(),
        pulse: PulseRange = PulseRange(),
        offsets: AngleOffsetsDeg = AngleOffsetsDeg(),
        mirror: MirrorConfig = MirrorConfig(),
        name: str = "leg",
        initial_angles_deg: Tuple[float, float, float] = (90.0, 90.0, 90.0),
    ):
        self.kit = kit
        self.channels = channels
        self.geom = geom
        self.limits = limits
        self.pulse = pulse
        self.offsets = offsets
        self.mirror = mirror
        self.name = name

        # Configure pulse width range
        for ch in (channels.yaw, channels.hip, channels.knee):
            self.kit.servo[ch].set_pulse_width_range(pulse.min_us, pulse.max_us)

        # State
        self.yaw_deg, self.hip_deg, self.knee_deg = initial_angles_deg
        self.foot_local_mm: Tuple[float, float, float] = (0.0, 0.0, -(geom.L1 + geom.L2))

        # Apply initial pose
        self.set_angles_deg(*initial_angles_deg)

    # ----------------------- Servo output -----------------------
    def set_angles_deg(self, yaw_deg: float, hip_deg: float, knee_deg: float) -> None:
        """Set servo angles directly (degrees), with limits + mirroring."""
        yaw = clamp(yaw_deg, *self.limits.yaw)
        hip = clamp(hip_deg, *self.limits.hip)
        knee = clamp(knee_deg, *self.limits.knee)

        # Apply mirroring at the final servo angle stage
        if self.mirror.yaw:
            yaw = 180.0 - yaw
        if self.mirror.hip:
            hip = 180.0 - hip
        if self.mirror.knee:
            knee = 180.0 - knee

        self.kit.servo[self.channels.yaw].angle = yaw
        self.kit.servo[self.channels.hip].angle = hip
        self.kit.servo[self.channels.knee].angle = knee

        self.yaw_deg, self.hip_deg, self.knee_deg = yaw, hip, knee

    def home(self, yaw: float = 90.0, hip: float = 90.0, knee: float = 90.0) -> None:
        self.set_angles_deg(yaw, hip, knee)

    # ----------------------- High-level foot control -----------------------
    def set_foot_local_mm(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        *,
        update_state: bool = True,
    ) -> Tuple[float, float, float]:
        """
        Command foot position (x,y,z) in leg-local frame (mm).
        Returns commanded (yaw, hip, knee) in degrees (servo angles).
        """
        yaw, hip, knee = self.ik_deg(x_mm, y_mm, z_mm)
        self.set_angles_deg(yaw, hip, knee)
        if update_state:
            self.foot_local_mm = (x_mm, y_mm, z_mm)
        return (yaw, hip, knee)

    # ----------------------- Inverse kinematics (degrees) -----------------------
    def ik_deg(self, x_mm: float, y_mm: float, z_mm: float) -> Tuple[float, float, float]:
        """
        IK in degrees.
        Steps:
          1) yaw points toward (x, y)
          2) reduce to planar 2-link IK in (r, z)
        """
        g = self.geom

        # Apply offsets if hip pitch axis is shifted from yaw axis
        y_eff = y_mm - g.hip_offset_y
        z_eff = z_mm - g.hip_offset_z

        # --- 1) Yaw ---
        # Geometric yaw angle (0 deg points along +x). We map into servo space via offsets later.
        yaw_geom_deg = atan2_deg(y_eff, x_mm)

        # Rotate into yaw plane: r is distance from yaw axis
        r = math.hypot(x_mm, y_eff)

        # --- 2) Planar 2-link IK (r, z_eff) ---
        L1, L2 = g.L1, g.L2
        D = math.hypot(r, z_eff)

        # Clamp to reachable workspace to avoid acos domain errors
        D = clamp(D, abs(L1 - L2) + 1e-6, (L1 + L2) - 1e-6)

        # Knee: law of cosines
        cos_knee_inner = (L1 * L1 + L2 * L2 - D * D) / (2.0 * L1 * L2)
        cos_knee_inner = clamp(cos_knee_inner, -1.0, 1.0)
        knee_inner_deg = acos_deg(cos_knee_inner)      # 0..180
        knee_geom_deg = 180.0 - knee_inner_deg         # “elbow-down” configuration

        # Hip:
        # alpha = angle to target in (r,z)
        alpha_deg = atan2_deg(z_eff, r)
        # beta from law of cosines
        cos_beta = (L1 * L1 + D * D - L2 * L2) / (2.0 * L1 * D)
        cos_beta = clamp(cos_beta, -1.0, 1.0)
        beta_deg = acos_deg(cos_beta)

        hip_geom_deg = alpha_deg - beta_deg

        # --- Map geometric angles into servo angles (degree-only offsets) ---
        yaw_servo = yaw_geom_deg + self.offsets.yaw
        hip_servo = hip_geom_deg + self.offsets.hip
        knee_servo = knee_geom_deg + self.offsets.knee

        # Clamp to limits (still servo-space degrees)
        yaw_servo = clamp(yaw_servo, *self.limits.yaw)
        hip_servo = clamp(hip_servo, *self.limits.hip)
        knee_servo = clamp(knee_servo, *self.limits.knee)

        return (yaw_servo, hip_servo, knee_servo)

    # ----------------------- Forward kinematics (degrees) -----------------------
    def fk_mm(self, yaw_deg: float, hip_deg: float, knee_deg: float) -> Tuple[float, float, float]:
        """
        Forward kinematics (for validation / debugging).
        Inputs are servo angles in degrees.
        Output: (x,y,z) in leg-local frame in mm.

        Notes:
          - This uses your offsets as inverse mapping (servo -> geometric).
          - If your offsets are wrong, FK will be wrong (which is useful feedback).
        """
        g = self.geom
        L1, L2 = g.L1, g.L2

        # Convert servo-space -> geometric-space
        yaw_geom = yaw_deg - self.offsets.yaw
        hip_geom = hip_deg - self.offsets.hip
        knee_geom = knee_deg - self.offsets.knee

        # Planar position in yaw plane (r, z)
        # In planar chain:
        #   hip angle = hip_geom
        #   knee angle = knee_geom (where 0 means fully straight depends on convention)
        #
        # With our IK convention, knee_geom is 0 when straight, increasing as it bends.
        # The second link angle relative to the first is (hip_geom + (180 - knee_inner)) style,
        # but we defined knee_geom = 180 - knee_inner.
        # So absolute angle of link2 = hip_geom + (knee_geom - 180)? depends on chosen convention.
        #
        # We can derive from triangle geometry more robustly:
        # We'll treat hip_geom as angle of link1 from +r axis, and knee_geom as "bend" where 0 is straight.
        # Then link2 angle = hip_geom + (knee_geom - 180) would be wrong.
        #
        # Instead: define knee_bend = knee_geom. When knee_bend=0 -> link2 aligned with link1.
        # So link2 absolute angle = hip_geom + knee_bend.
        knee_bend = knee_geom

        r = L1 * cos_deg(hip_geom) + L2 * cos_deg(hip_geom + knee_bend)
        z = L1 * sin_deg(hip_geom) + L2 * sin_deg(hip_geom + knee_bend)

        # Rotate r back into (x,y) using yaw
        x = r * cos_deg(yaw_geom)
        y = r * sin_deg(yaw_geom)

        # Re-apply offsets from yaw axis to hip axis
        y += g.hip_offset_y
        z += g.hip_offset_z

        return (x, y, z)

    # ----------------------- Debug helpers -----------------------
    def get_angles_deg(self) -> Tuple[float, float, float]:
        return (self.yaw_deg, self.hip_deg, self.knee_deg)

    def get_foot_local_mm(self) -> Tuple[float, float, float]:
        return self.foot_local_mm
s