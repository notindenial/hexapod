# Leg.py
#
# Barebones 3DOF hexapod leg controller with:
#   - Easy initialization
#   - Per-leg neutral yaw (front/back legs can be angled forward at rest)
#   - Optional IK (foot x,y,z -> yaw,hip,knee)
#   - Software speed limiting (deg/sec) via update(dt)
#   - update(dt) returns True when target is reached
#
# Angles: degrees
# Lengths: mm
#
# Coordinate convention (be consistent):
#   x: forward
#   y: outward
#   z: up (so "down" is negative)

import math
from typing import Tuple


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def atan2_deg(y: float, x: float) -> float:
    return math.degrees(math.atan2(y, x))


def acos_deg(x: float) -> float:
    return math.degrees(math.acos(x))


class Leg:
    """
    Barebones 3DOF leg:
      - yaw (pan)
      - hip pitch
      - knee pitch

    Key idea:
      - set_angles(...) and set_foot_xyz(...) set TARGETS
      - update(dt) rate-limits motion toward targets and sends servo commands
      - update(dt) returns True when all joints have reached targets
    """

    def __init__(
        self,
        kit,
        yaw_ch: int,
        hip_ch: int,
        knee_ch: int,
        *,
        L1_mm: float,
        L2_mm: float,
        # Neutral pose per leg (servo angles):
        yaw_neutral_deg: float = 90.0,  # front/back can be slightly forward
        hip_neutral_deg: float = 90.0,
        knee_neutral_deg: float = 90.0,
        # Servo calibration:
        pulse_min_us: int = 500,
        pulse_max_us: int = 2500,
        # Speed limiting:
        max_speed_deg_s: float = 120.0,   # tune: 60 (safe) .. 180 (fast)
        angle_epsilon_deg: float = 0.5,   # "close enough" threshold
        name: str = "leg",
        # Optional safety limits (servo space):
        yaw_limits: Tuple[float, float] = (0.0, 180.0),
        hip_limits: Tuple[float, float] = (0.0, 180.0),
        knee_limits: Tuple[float, float] = (0.0, 180.0),
    ):
        self.kit = kit
        self.name = name

        # Channels
        self.yaw_ch = yaw_ch
        self.hip_ch = hip_ch
        self.knee_ch = knee_ch

        # Geometry (mm)
        self.L1 = float(L1_mm)
        self.L2 = float(L2_mm)

        # Limits
        self.yaw_limits = yaw_limits
        self.hip_limits = hip_limits
        self.knee_limits = knee_limits

        # Neutrals (servo angles)
        self.yaw_neutral = float(yaw_neutral_deg)
        self.hip_neutral = float(hip_neutral_deg)
        self.knee_neutral = float(knee_neutral_deg)

        # Speed limiting
        self.max_speed_deg_s = float(max_speed_deg_s)
        self.angle_epsilon = float(angle_epsilon_deg)

        # Configure PWM calibration
        for ch in (yaw_ch, hip_ch, knee_ch):
            self.kit.servo[ch].set_pulse_width_range(pulse_min_us, pulse_max_us)

        # Internal commanded angles (what we last sent)
        self._yaw_cmd = self.yaw_neutral
        self._hip_cmd = self.hip_neutral
        self._knee_cmd = self.knee_neutral

        # Targets (what we want to reach)
        self._yaw_target = self.yaw_neutral
        self._hip_target = self.hip_neutral
        self._knee_target = self.knee_neutral

        # Optional: store last foot target (for debugging/gaits)
        self.foot_target_xyz_mm = (0.0, 0.0, -(self.L1 + self.L2))

        # Move to home (targets set + one immediate write)
        self.home(immediate=True)

    # -------------------- Basic target setters --------------------
    def home(self, *, immediate: bool = False) -> None:
        """Set targets to neutral pose. If immediate=True, write instantly (no ramp)."""
        self.set_angles(self.yaw_neutral, self.hip_neutral, self.knee_neutral, immediate=immediate)

    def set_angles(self, yaw_deg: float, hip_deg: float, knee_deg: float, *, immediate: bool = False) -> None:
        """
        Set TARGET servo angles in degrees.
        If immediate=True, jump to target (no speed limiting).
        """
        yaw = clamp(float(yaw_deg), *self.yaw_limits)
        hip = clamp(float(hip_deg), *self.hip_limits)
        knee = clamp(float(knee_deg), *self.knee_limits)

        self._yaw_target = yaw
        self._hip_target = hip
        self._knee_target = knee

        if immediate:
            self._yaw_cmd, self._hip_cmd, self._knee_cmd = yaw, hip, knee
            self._write_servos(self._yaw_cmd, self._hip_cmd, self._knee_cmd)

    # -------------------- Optional IK interface --------------------
    def set_foot_xyz(self, x_mm: float, y_mm: float, z_mm: float, *, immediate: bool = False) -> Tuple[float, float, float]:
        """
        Set a foot target (x,y,z) in mm and compute joint targets via IK.
        Returns (yaw_target, hip_target, knee_target).
        """
        yaw, hip, knee = self.ik(x_mm, y_mm, z_mm)
        self.foot_target_xyz_mm = (float(x_mm), float(y_mm), float(z_mm))
        self.set_angles(yaw, hip, knee, immediate=immediate)
        return (yaw, hip, knee)

    def ik(self, x_mm: float, y_mm: float, z_mm: float) -> Tuple[float, float, float]:
        """
        Simple IK:
          - yaw points toward (x,y)
          - hip/knee solve 2-link planar IK in (r,z)

        Returns servo-space angles (yaw, hip, knee) in degrees.
        Uses neutrals as the zero reference:
          yaw = yaw_neutral + yaw_geom
          hip = hip_neutral + hip_geom
          knee = knee_neutral + knee_geom
        """
        x = float(x_mm)
        y = float(y_mm)
        z = float(z_mm)

        # Yaw geometry: 0 deg points along +x
        yaw_geom = atan2_deg(y, x)  # -180..180
        yaw = self.yaw_neutral + yaw_geom

        # Reduce to planar problem
        r = math.hypot(x, y)
        D = math.hypot(r, z)

        # Clamp D to reachable range to avoid acos domain errors
        D = clamp(D, abs(self.L1 - self.L2) + 1e-6, (self.L1 + self.L2) - 1e-6)

        # Knee (elbow-down)
        cos_knee_inner = (self.L1**2 + self.L2**2 - D**2) / (2.0 * self.L1 * self.L2)
        cos_knee_inner = clamp(cos_knee_inner, -1.0, 1.0)
        knee_inner = acos_deg(cos_knee_inner)     # 0..180
        knee_geom = 180.0 - knee_inner

        # Hip
        alpha = atan2_deg(z, r)
        cos_beta = (self.L1**2 + D**2 - self.L2**2) / (2.0 * self.L1 * D)
        cos_beta = clamp(cos_beta, -1.0, 1.0)
        beta = acos_deg(cos_beta)
        hip_geom = alpha - beta

        hip = self.hip_neutral + hip_geom
        knee = self.knee_neutral + knee_geom

        # Clamp to servo limits
        yaw = clamp(yaw, *self.yaw_limits)
        hip = clamp(hip, *self.hip_limits)
        knee = clamp(knee, *self.knee_limits)

        return (yaw, hip, knee)

    # -------------------- Motion update (rate limiting) --------------------
    def update(self, dt: float) -> bool:
        """
        Move commanded angles toward targets, respecting max_speed_deg_s.
        Sends servo commands each call.

        Returns True if all joints are within angle_epsilon of their targets.
        """
        dt = float(dt)
        if dt <= 0.0:
            # Nothing can progress; report whether already done
            return self.is_at_target()

        max_step = self.max_speed_deg_s * dt

        def step(curr: float, target: float) -> float:
            err = target - curr
            if abs(err) <= max_step:
                return target
            return curr + max_step if err > 0 else curr - max_step

        self._yaw_cmd = step(self._yaw_cmd, self._yaw_target)
        self._hip_cmd = step(self._hip_cmd, self._hip_target)
        self._knee_cmd = step(self._knee_cmd, self._knee_target)

        self._write_servos(self._yaw_cmd, self._hip_cmd, self._knee_cmd)

        return self.is_at_target()

    def is_at_target(self) -> bool:
        """Return True if all joints are within epsilon of their targets."""
        return (
            abs(self._yaw_cmd - self._yaw_target) <= self.angle_epsilon and
            abs(self._hip_cmd - self._hip_target) <= self.angle_epsilon and
            abs(self._knee_cmd - self._knee_target) <= self.angle_epsilon
        )

    # -------------------- Readbacks (software state) --------------------
    def get_targets(self) -> Tuple[float, float, float]:
        return (self._yaw_target, self._hip_target, self._knee_target)

    def get_commands(self) -> Tuple[float, float, float]:
        return (self._yaw_cmd, self._hip_cmd, self._knee_cmd)

    # -------------------- Internal hardware write --------------------
    def _write_servos(self, yaw: float, hip: float, knee: float) -> None:
        self.kit.servo[self.yaw_ch].angle = yaw
        self.kit.servo[self.hip_ch].angle = hip
        self.kit.servo[self.knee_ch].angle = knee
