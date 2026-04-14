"""
animations.py — Canned motion sequences for the hexapod.

Each animation is a function that accepts a Body and plays back a scripted
sequence of poses/moves, blocking until complete.

All animations are designed to start and end in the standing pose so they
can be chained without intermediate transitions.
"""

from __future__ import annotations
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from body import Body


def wave(body: "Body", *, leg_name: str = "RF", num_waves: int = 3) -> None:
    """
    Lift one leg and wave it up and down.
    Starts and ends in standing pose.
    """
    leg = body.legs.get(leg_name)
    if leg is None:
        raise ValueError(f"Leg '{leg_name}' not found. Available: {list(body.legs)}")

    # Lift the leg up
    leg.set_foot_xyz(40.0, -40.0, 60.0)
    _wait_leg(leg, dt=0.02)

    # Wave: oscillate knee angle
    for _ in range(num_waves):
        for phase in range(20):
            import math
            t = phase / 20
            z = 60.0 + 30.0 * math.sin(2 * math.pi * t)
            leg.set_foot_xyz(40.0, -40.0, z, immediate=True)
            time.sleep(0.05)

    # Return to stand
    leg.home(immediate=False)
    _wait_leg(leg, dt=0.02)


def bow(body: "Body", *, depth_mm: float = 30.0) -> None:
    """
    Bow forward: pitch the body forward and back.
    """
    # Pitch forward
    for i in range(20):
        t = i / 20
        import math
        pitch = math.radians(20.0) * math.sin(math.pi * t)
        body.set_body_pose(pitch_rad=pitch)
        _tick_all(body, dt=0.02)
        time.sleep(0.02)

    # Return to neutral
    body.set_body_pose()
    _tick_all(body, dt=0.02, ticks=20)


def stand_up(body: "Body") -> None:
    """Transition from sitting to standing."""
    body.stand()


def sit_down(body: "Body") -> None:
    """Transition from standing to sitting."""
    body.sit()


def dance(body: "Body", *, cycles: int = 4) -> None:
    """
    Simple rhythmic body sway left/right.
    """
    import math
    steps = 40
    for cycle in range(cycles):
        for i in range(steps):
            t = i / steps
            y = 20.0 * math.sin(2 * math.pi * t)
            roll = math.radians(10.0) * math.sin(2 * math.pi * t)
            body.set_body_pose(y_mm=y, roll_rad=roll)
            _tick_all(body, dt=0.02, ticks=1)
            time.sleep(0.02)
    body.set_body_pose()


# ─── Internal helpers ─────────────────────────────────────────────────────────

def _wait_leg(leg, *, dt: float, timeout_s: float = 2.0) -> None:
    t0 = time.time()
    while not leg.is_at_target():
        leg.update(dt)
        time.sleep(dt)
        if (time.time() - t0) > timeout_s:
            break


def _tick_all(body: "Body", *, dt: float, ticks: int = 1) -> None:
    for _ in range(ticks):
        body.update(dt)
