"""
sim/show_robot.py — Open a 3D matplotlib window showing the hexapod in a
static pose (stand by default). No terrain, no gait, no animation — just a
sanity check that the geometry and FK chain render correctly.

Usage:
    python -m sim.show_robot              # stand pose
    python -m sim.show_robot --pose neutral
    python -m sim.show_robot --pose sit
"""

from __future__ import annotations
import argparse
import os
import sys

# Allow running as a script from the repo root
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import yaml

from leg import Leg
from sim.sim_servo import SimDriver
from sim.sim_body import render_hexapod


def build_legs(cfg: dict):
    kin  = cfg["kinematics"]
    lims = cfg["limits"]
    ctrl = cfg["control"]

    legs = {}
    for name, lcfg in cfg["legs"].items():
        mirror = bool(lcfg.get("mirror", False))
        driver = SimDriver(name=name, mirror=mirror)

        pan_neu = float(lcfg.get("pan_neutral_deg", 90.0))
        group = name[1]  # F, M, B
        pan_out = float(lims["pan_outward_front_back_deg"] if group in ("F", "B")
                        else lims["pan_outward_mid_deg"])
        pan_in  = float(lims["pan_inward_deg"])
        pan_lim = (max(0.0, pan_neu - pan_in), min(180.0, pan_neu + pan_out))

        legs[name] = Leg(
            driver,
            name=name,
            coxa_lateral_mm=float(kin["coxa_lateral_mm"]),
            coxa_drop_mm=float(kin["coxa_drop_mm"]),
            femur_mm=float(kin["femur_mm"]),
            tibia_mm=float(kin["tibia_mm"]),
            pan_neutral_deg=pan_neu,
            pan_limits=pan_lim,
            hip_limits=tuple(lims["hip"]),
            knee_limits=tuple(lims["knee"]),
            mount_xyz_mm=tuple(lcfg.get("mount_xyz_mm", [0.0, 0.0, 0.0])),
            mount_yaw_deg=float(lcfg.get("mount_yaw_deg", 0.0)),
            max_speed_deg_s=float(ctrl["max_speed_deg_s"]),
            angle_epsilon_deg=float(ctrl["angle_epsilon_deg"]),
        )
    return legs


def main() -> None:
    parser = argparse.ArgumentParser(description="Show hexapod in 3D")
    parser.add_argument("--config", default="config.yaml",
                        help="Path to config.yaml (default: repo root)")
    parser.add_argument("--pose", default="stand", choices=["neutral", "stand", "sit"],
                        help="Which pose to display")
    args = parser.parse_args()

    cfg_path = args.config if os.path.isabs(args.config) else os.path.join(_REPO_ROOT, args.config)
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    legs = build_legs(cfg)

    import math
    pose = cfg["poses"][args.pose]
    pan_base = float(pose["pan"])
    hip      = float(pose["hip"])
    knee     = float(pose["knee"])
    bias     = float(pose.get("front_back_pan_bias_deg", 0.0))

    for name, leg in legs.items():
        group = name[1]  # F, M, B
        yaw_sign = math.copysign(1.0, leg.mount_yaw_deg) if leg.mount_yaw_deg != 0.0 else 1.0
        if group == "F":
            pan = pan_base - yaw_sign * bias
        elif group == "B":
            pan = pan_base + yaw_sign * bias
        else:
            pan = pan_base
        leg.set_angles(pan, hip, knee, immediate=True)

    ground_z = -float(cfg["kinematics"].get("stand_clearance_mm", 50.0))

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    render_hexapod(ax, legs, ground_z_mm=ground_z, title=f"Hexapod — {args.pose} pose")
    ax.view_init(elev=22, azim=-60)

    # Print a sanity summary
    print(f"Pose: {args.pose}  (pan={pan}, hip={hip}, knee={knee})")
    print("Foot positions in body frame (mm):")
    from sim.sim_body import _leg_chain_body_frame
    for name, leg in legs.items():
        chain = _leg_chain_body_frame(leg, *leg.commands)
        foot = chain[-1]
        print(f"  {name}: ({foot[0]:7.1f}, {foot[1]:7.1f}, {foot[2]:7.1f})")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
