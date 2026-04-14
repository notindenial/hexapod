#!/usr/bin/env python3
"""
main.py - Hexapod initialization / bring-up with joint limits

Limits:
- Hip  : 10..135 deg
- Knee : 10..135 deg
- Pan/yaw:
    Front & Back: neutral-15 .. neutral+45
    Mid         : neutral-15 .. neutral+15

Default sitting pose:
- Pan (yaw)  = 90 deg
- Hip        = 120 deg
- Knee       = 0 deg  (NOTE: will be clamped to knee min=10)
"""

from dataclasses import dataclass
from typing import Dict, Tuple
from adafruit_servokit import ServoKit
import time

from leg import Leg  # adjust if your file name differs


# ----------------------------
# USER CONFIG (EDIT ME)
# ----------------------------

LEFT_ADDR = 0x40
RIGHT_ADDR = 0x41
PCA_CHANNELS = 16

# Geometry (mm)
L1_MM = 55.0
L2_MM = 85.0

# Default sitting pose (degrees)
SIT_YAW_DEG = 90.0
SIT_HIP_DEG = 165.0
SIT_KNEE_DEG = 0.0  # will clamp to 10 due to limits below



# Servo calibration
PULSE_MIN_US = 500
PULSE_MAX_US = 2500

# Speed limiting (used only if you call update(dt))
MAX_SPEED_DEG_S = 120.0
ANGLE_EPS_DEG = 0.5

# Joint limits (servo-space)
HIP_LIMITS = (10.0, 170.0)
KNEE_LIMITS = (10.0, 170.0)

# Pan “swing” relative to neutral (degrees)
PAN_INWARD_DEG = 15.0
PAN_OUTWARD_FB_DEG = 45.0   # front/back outward
PAN_OUTWARD_MID_DEG = 15.0  # mid outward


@dataclass(frozen=True)
class LegCfg:
    name: str
    side: str          # "L" or "R"
    knee: int
    hip: int
    pan: int           # pan channel == yaw_ch in Leg class
    group: str         # "F", "M", "B"
    yaw_neutral: float = 90.0
    hip_neutral: float = 90.0
    knee_neutral: float = 90.0


LEG_CFGS = [
    # Left (0x40)
    LegCfg(name="LF", side="L", knee=0, hip=1, pan=2, group="F"),
    LegCfg(name="LM", side="L", knee=4, hip=5, pan=6, group="M"),
    LegCfg(name="LB", side="L", knee=8, hip=9, pan=10, group="B"),

    # Right (0x41)
    LegCfg(name="RF", side="R", knee=12, hip=13, pan=14, group="F"),
    LegCfg(name="RM", side="R", knee=8,  hip=9,  pan=10, group="M"),
    LegCfg(name="RB", side="R", knee=4,  hip=5,  pan=6,  group="B"),
]


# ----------------------------
# INIT HELPERS
# ----------------------------

def make_kits() -> Dict[str, ServoKit]:
    return {
        "L": ServoKit(channels=PCA_CHANNELS, address=LEFT_ADDR),
        "R": ServoKit(channels=PCA_CHANNELS, address=RIGHT_ADDR),
    }


def yaw_limits_for(cfg: LegCfg) -> Tuple[float, float]:
    """
    Returns (min_yaw, max_yaw) in servo-space degrees.
    Convention used:
      inward  = neutral - 15
      outward = neutral + (45 for F/B, 15 for M)
    """
    inward = cfg.yaw_neutral - PAN_INWARD_DEG
    outward = cfg.yaw_neutral + (PAN_OUTWARD_FB_DEG if cfg.group in ("F", "B") else PAN_OUTWARD_MID_DEG)

    # Ensure order
    lo = min(inward, outward)
    hi = max(inward, outward)

    # Hard clamp to servo range
    lo = max(0.0, lo)
    hi = min(180.0, hi)
    return (lo, hi)


def make_legs(kits: Dict[str, ServoKit]) -> Dict[str, Leg]:
    legs: Dict[str, Leg] = {}

    for cfg in LEG_CFGS:
        kit = kits[cfg.side]
        yaw_limits = yaw_limits_for(cfg)

        leg = Leg(
            kit,
            cfg.pan,   # yaw_ch
            cfg.hip,   # hip_ch
            cfg.knee,  # knee_ch
            side=cfg.side,
            L1_mm=L1_MM,
            L2_mm=L2_MM,
            yaw_neutral_deg=cfg.yaw_neutral,
            hip_neutral_deg=cfg.hip_neutral,
            knee_neutral_deg=cfg.knee_neutral,
            pulse_min_us=PULSE_MIN_US,
            pulse_max_us=PULSE_MAX_US,
            max_speed_deg_s=MAX_SPEED_DEG_S,
            angle_epsilon_deg=ANGLE_EPS_DEG,
            name=cfg.name,
            yaw_limits=yaw_limits,
            hip_limits=HIP_LIMITS,
            knee_limits=KNEE_LIMITS,
        )

        legs[cfg.name] = leg

    return legs


def sit_immediate(legs: Dict[str, Leg]) -> None:
    for leg in legs.values():
        leg.set_angles(SIT_YAW_DEG, SIT_HIP_DEG, SIT_KNEE_DEG, immediate=True)

def stand(legs: Dict[str, Leg]) -> None:
def stand(legs: Dict[str, Leg], *, dt: float = 0.02, timeout_s: float = 3.0) -> None:
    """
    Smoothly move all legs to a standing pose using Leg.update(dt).

    dt: control period in seconds (0.02 = 50 Hz)
    timeout_s: safety timeout so we don't hang forever
    """
    # 1) Set targets (non-immediate)
    for leg in legs.values():
        # Keep per-leg yaw neutral (respects your front/back offsets)
        leg.set_angles(
            leg.yaw_neutral,   # <-- uses each leg's own neutral yaw
            STAND_HIP_DEG,
            STAND_KNEE_DEG,
            immediate=False
        )

    # 2) Update until all legs report done (or timeout)
    t0 = time.time()
    while True:
        all_done = True
        for leg in legs.values():
            done = leg.update(dt)
            all_done = all_done and done

        if all_done:
            return

        if (time.time() - t0) > timeout_s:
            print("WARNING: stand() timed out; legs may not have fully reached targets.")
            return

        time.sleep(dt)  

def main() -> None:
    kits = make_kits()
    legs = make_legs(kits)

    sit_immediate(legs)

    print("Hexapod initialized with limits. Leg targets after sit():")
    for name in sorted(legs.keys()):
        yaw_t, hip_t, knee_t = legs[name].get_targets()
        print(f"  {name}: yaw={yaw_t:.1f} hip={hip_t:.1f} knee={knee_t:.1f}")

    time.sleep(1.0)
    stand(legs)



if __name__ == "__main__":
    main()
