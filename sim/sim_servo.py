"""
sim/sim_servo.py — Drop-in servo driver backends.

HardwareDriver  wraps adafruit_servokit channels for real PCA9685 hardware.
SimDriver       logs angles to stdout (or a callback) without any hardware.

Both implement the ServoDriver protocol defined in leg.py.
"""

from __future__ import annotations
from typing import Callable, Dict, Optional, Tuple


# ─── Hardware driver ─────────────────────────────────────────────────────────

class HardwareDriver:
    """
    Drives three PCA9685 channels for one leg.
    Handles left-side mirroring (hardware angle = 180 - logical angle).
    """

    def __init__(
        self,
        kit,
        *,
        pan_ch: int,
        hip_ch: int,
        knee_ch: int,
        mirror: bool,
        pulse_ranges: Dict[str, Tuple[int, int]],
    ) -> None:
        self._kit    = kit
        self._pan_ch = pan_ch
        self._hip_ch = hip_ch
        self._knee_ch = knee_ch
        self._mirror = mirror

        # Configure pulse widths
        kit.servo[pan_ch].set_pulse_width_range(*pulse_ranges["pan"])
        kit.servo[hip_ch].set_pulse_width_range(*pulse_ranges["hip"])
        kit.servo[knee_ch].set_pulse_width_range(*pulse_ranges["knee"])

    def _hw(self, angle: float) -> float:
        """Convert logical angle to hardware angle."""
        return 180.0 - angle if self._mirror else angle

    def write_angles(self, pan_deg: float, hip_deg: float, knee_deg: float) -> None:
        self._kit.servo[self._pan_ch].angle  = self._hw(pan_deg)
        self._kit.servo[self._hip_ch].angle  = self._hw(hip_deg)
        self._kit.servo[self._knee_ch].angle = self._hw(knee_deg)

    def set_pulse_range(
        self,
        pan: Tuple[int, int],
        hip: Tuple[int, int],
        knee: Tuple[int, int],
    ) -> None:
        self._kit.servo[self._pan_ch].set_pulse_width_range(*pan)
        self._kit.servo[self._hip_ch].set_pulse_width_range(*hip)
        self._kit.servo[self._knee_ch].set_pulse_width_range(*knee)


# ─── Simulation driver ────────────────────────────────────────────────────────

class SimDriver:
    """
    Simulation servo driver — no hardware required.

    Stores current angles and optionally invokes a callback so a visualiser
    (matplotlib, pybullet, etc.) can react to angle changes.
    """

    def __init__(
        self,
        name: str,
        mirror: bool = False,
        on_update: Optional[Callable[[str, float, float, float], None]] = None,
    ) -> None:
        self.name    = name
        self._mirror = mirror
        self._on_update = on_update

        self.pan_deg:  float = 90.0
        self.hip_deg:  float = 90.0
        self.knee_deg: float = 90.0

    def write_angles(self, pan_deg: float, hip_deg: float, knee_deg: float) -> None:
        self.pan_deg  = pan_deg
        self.hip_deg  = hip_deg
        self.knee_deg = knee_deg
        if self._on_update:
            self._on_update(self.name, pan_deg, hip_deg, knee_deg)

    def set_pulse_range(
        self,
        pan: Tuple[int, int],
        hip: Tuple[int, int],
        knee: Tuple[int, int],
    ) -> None:
        pass  # no-op in simulation
