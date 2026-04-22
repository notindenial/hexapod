"""
main.py — Hexapod entry point.

Loads config.yaml, initialises hardware (or simulation), builds the Body,
then runs the 50 Hz control loop.

Usage:
    python main.py           # real hardware (RPi + PCA9685)
    python main.py --sim     # simulation backend (no hardware required)
"""

from __future__ import annotations
import argparse
import time
import yaml

from body import Body
from leg import Leg


# ─── Config ──────────────────────────────────────────────────────────────────

def load_config(path: str = "config.yaml") -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


# ─── Driver factories ────────────────────────────────────────────────────────

def make_hardware_driver(board_label: str, pan_ch: int, hip_ch: int, knee_ch: int,
                          mirror: bool, kits: dict, pulse_overrides: dict,
                          leg_name: str):
    """Build a real PCA9685-backed ServoDriver for one leg."""
    from sim.sim_servo import HardwareDriver  # lazily imported to avoid import error on non-RPi
    kit = kits[board_label]
    pulse_min = 500
    pulse_max_pan  = 2500
    pulse_max_hip  = 2500
    pulse_max_knee = pulse_overrides.get(f"{leg_name}_knee_pulse_max_us", 2500)
    return HardwareDriver(
        kit=kit,
        pan_ch=pan_ch, hip_ch=hip_ch, knee_ch=knee_ch,
        mirror=mirror,
        pulse_ranges={
            "pan":  (pulse_min, pulse_max_pan),
            "hip":  (pulse_min, pulse_max_hip),
            "knee": (pulse_min, pulse_max_knee),
        },
    )


def make_sim_driver(leg_name: str, mirror: bool):
    """Build a simulation ServoDriver (prints angles, no hardware)."""
    from sim.sim_servo import SimDriver
    return SimDriver(name=leg_name, mirror=mirror)


# ─── Initialisation ──────────────────────────────────────────────────────────

def init_legs(cfg: dict, sim: bool) -> dict[str, Leg]:
    kin   = cfg["kinematics"]
    coxa_lateral = float(kin["coxa_lateral_mm"])
    coxa_drop    = float(kin["coxa_drop_mm"])
    femur        = float(kin["femur_mm"])
    tibia        = float(kin["tibia_mm"])
    lims  = cfg["limits"]
    ctrl  = cfg["control"]
    servo = cfg["servo"]
    pulse_overrides = servo.get("overrides", {})

    kits = {}
    if not sim:
        from adafruit_servokit import ServoKit
        for driver_cfg in cfg["hardware"]["servo_drivers"]:
            kits[driver_cfg["label"]] = ServoKit(
                channels=16, address=int(driver_cfg["i2c_address"], 16)
            )

    legs: dict[str, Leg] = {}
    for name, lcfg in cfg["legs"].items():
        pan_ch  = lcfg["pan_ch"]
        hip_ch  = lcfg["hip_ch"]
        knee_ch = lcfg["knee_ch"]
        mirror  = lcfg.get("mirror", False)
        mount   = tuple(lcfg.get("mount_xyz_mm", [0.0, 0.0, 0.0]))
        mount_yaw = float(lcfg.get("mount_yaw_deg", 0.0))
        pan_neu = float(lcfg.get("pan_neutral_deg", 90.0))

        if sim:
            driver = make_sim_driver(name, mirror)
        else:
            driver = make_hardware_driver(
                lcfg["board"], pan_ch, hip_ch, knee_ch,
                mirror, kits, pulse_overrides, name
            )

        # Pan limits derived from group (F/M/B)
        group = name[1]   # "F", "M", or "B"
        pan_out = float(lims["pan_outward_front_back_deg"] if group in ("F", "B")
                        else lims["pan_outward_mid_deg"])
        pan_in  = float(lims["pan_inward_deg"])
        pan_lim = (
            max(0.0,   pan_neu - pan_in),
            min(180.0, pan_neu + pan_out),
        )

        legs[name] = Leg(
            driver,
            name=name,
            coxa_lateral_mm=coxa_lateral,
            coxa_drop_mm=coxa_drop,
            femur_mm=femur,
            tibia_mm=tibia,
            pan_neutral_deg=pan_neu,
            pan_limits=pan_lim,
            hip_limits=tuple(lims["hip"]),
            knee_limits=tuple(lims["knee"]),
            mount_xyz_mm=mount,
            mount_yaw_deg=mount_yaw,
            max_speed_deg_s=float(ctrl["max_speed_deg_s"]),
            angle_epsilon_deg=float(ctrl["angle_epsilon_deg"]),
        )

    return legs


# ─── Main loop ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Hexapod controller")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode")
    parser.add_argument("--config", default="config.yaml", help="Path to config file")
    args = parser.parse_args()

    cfg  = load_config(args.config)
    ctrl = cfg["control"]
    dt   = 1.0 / float(ctrl["loop_rate_hz"])

    legs = init_legs(cfg, sim=args.sim)
    body = Body(legs, cfg)

    print("Sitting...")
    body.sit()
    time.sleep(0.5)

    print("Standing...")
    body.stand()
    time.sleep(1.0)

    print("Control loop running. Ctrl-C to exit.")
    try:
        while True:
            # TODO: read joystick / planner velocity commands
            # body.set_velocity(vx, vy, omega)
            body.update(dt)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nShutting down.")
        body.sit()


if __name__ == "__main__":
    main()
