"""
visualizer.py — Render the planner's plan.txt over a heightmap.

Given a map directory (contains heightmap.csv + meta.yaml) and a plan.txt
produced by the C++ planner, produces:
  - preview.png in the map directory (terrain + start/goal, regenerated)
  - plan.gif   : top-down animation of the stance sequence
                  (body, support triangle, feet dots, goal marker)

Usage:
  python3 visualizer.py --map maps/flat --plan plan.txt
  python3 visualizer.py --map maps/bumps --plan plan.txt --gif plan.gif --fps 12 --interp 6
"""

from __future__ import annotations
import argparse
import math
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

import yaml


TRIPODS = {0: ["RF", "RB", "LM"], 1: ["LF", "LB", "RM"]}
LEGS = ["LF", "LM", "LB", "RF", "RM", "RB"]


def load_map(map_dir: str):
    with open(os.path.join(map_dir, "meta.yaml")) as f:
        meta = yaml.safe_load(f)
    grid = meta["grid"]
    nx, ny = int(grid["nx"]), int(grid["ny"])
    cell = float(grid["cell_size_mm"])
    ox = float(grid["origin_x_mm"])
    oy = float(grid["origin_y_mm"])

    heights = np.loadtxt(os.path.join(map_dir, "heightmap.csv"), delimiter=",")
    assert heights.shape == (ny, nx), f"heightmap shape {heights.shape} != ({ny},{nx})"

    return {
        "name": meta.get("name", os.path.basename(map_dir)),
        "nx": nx, "ny": ny, "cell": cell,
        "origin_x": ox, "origin_y": oy,
        "heights": heights,
        "start": meta["start"],
        "goal":  meta["goal"],
    }


def map_extent(m):
    x0 = m["origin_x"] - 0.5 * m["cell"]
    x1 = m["origin_x"] + (m["nx"] - 0.5) * m["cell"]
    y0 = m["origin_y"] - 0.5 * m["cell"]
    y1 = m["origin_y"] + (m["ny"] - 0.5) * m["cell"]
    return (x0, x1, y0, y1)


def parse_plan(path: str):
    """Parse plan.txt. Returns (meta_dict, steps list of dicts)."""
    meta = {}
    steps = []
    col_names = None
    with open(path) as f:
        for line in f:
            line = line.rstrip()
            if not line:
                continue
            if line.startswith("#"):
                if "columns:" in line:
                    col_names = line.split("columns:", 1)[1].strip().split()
                elif "total_cost" in line:
                    # "# steps=N total_cost=C total_expansions=E"
                    for tok in line[1:].split():
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            meta[k] = v
                continue
            toks = line.split()
            # columns: step action body_x body_y body_yaw active_tripod LFx LFy LFz ... RBz
            step = {
                "step":  int(toks[0]),
                "action": toks[1],
                "bx":    float(toks[2]),
                "by":    float(toks[3]),
                "byaw":  float(toks[4]),
                "active_tripod": int(toks[5]),
                "feet": {},
            }
            offset = 6
            for leg in LEGS:
                step["feet"][leg] = (
                    float(toks[offset]),
                    float(toks[offset+1]),
                    float(toks[offset+2]),
                )
                offset += 3
            steps.append(step)
    return meta, steps


def draw_terrain(ax, m, title=None):
    ext = map_extent(m)
    im = ax.imshow(m["heights"], origin="lower", extent=ext, cmap="terrain",
                   vmin=-5, vmax=max(30.0, float(m["heights"].max())))
    ax.set_xlabel("x (mm, forward)")
    ax.set_ylabel("y (mm, left)")
    ax.set_aspect("equal")
    if title:
        ax.set_title(title)
    return im


def draw_static_markers(ax, m):
    start = m["start"]
    goal = m["goal"]
    ax.scatter([start["x_mm"]], [start["y_mm"]], marker="o",
               s=120, facecolors="none", edgecolors="lime", linewidths=2,
               label="start", zorder=5)
    ax.scatter([goal["x_mm"]], [goal["y_mm"]], marker="*",
               s=220, color="yellow", edgecolors="black", linewidths=1,
               label="goal", zorder=5)
    tol = float(goal.get("tolerance_mm", 40.0))
    th = np.linspace(0, 2*np.pi, 64)
    ax.plot(goal["x_mm"] + tol*np.cos(th), goal["y_mm"] + tol*np.sin(th),
            "--", color="yellow", linewidth=1, alpha=0.8)


def regenerate_preview(map_dir: str, m):
    fig, ax = plt.subplots(figsize=(9, 6))
    im = draw_terrain(ax, m, title=f"Map: {m['name']}")
    plt.colorbar(im, ax=ax, label="elevation (mm)")
    draw_static_markers(ax, m)
    ax.legend(loc="upper right")
    out = os.path.join(map_dir, "preview.png")
    plt.tight_layout()
    plt.savefig(out, dpi=120)
    plt.close(fig)
    return out


def interpolate_steps(steps, n_interp: int = 6, lift_mm: float = 30.0):
    """Insert n_interp sub-frames between each pair of plan steps.

    Anchored feet stay fixed. Swinging feet follow cubic-xy + sine-arc-z.
    Body pose is cubically eased.
    """
    frames = []
    for i, s in enumerate(steps):
        frames.append(s)
        if i + 1 >= len(steps):
            continue
        s1 = steps[i + 1]
        swing = set(TRIPODS[s["active_tripod"]])
        for k in range(1, n_interp):
            t = k / n_interp
            ease = 3 * t**2 - 2 * t**3
            feet = {}
            for leg in LEGS:
                x0, y0, z0 = s["feet"][leg]
                x1, y1, z1 = s1["feet"][leg]
                if leg in swing:
                    fx = x0 + (x1 - x0) * ease
                    fy = y0 + (y1 - y0) * ease
                    fz = z0 + (z1 - z0) * ease + lift_mm * math.sin(math.pi * t)
                else:
                    fx, fy, fz = x0, y0, z0
                feet[leg] = (fx, fy, fz)
            frames.append({
                "step":          s["step"] + t,
                "action":        s1["action"],
                "bx":            s["bx"]   + (s1["bx"]   - s["bx"])   * ease,
                "by":            s["by"]   + (s1["by"]   - s["by"])   * ease,
                "byaw":          s["byaw"] + (s1["byaw"] - s["byaw"]) * ease,
                "active_tripod": s["active_tripod"],
                "feet":          feet,
            })
    return frames


def render_animation(m, steps, meta, gif_path: str, fps: int = 4, n_interp: int = 6):
    if n_interp > 1:
        steps = interpolate_steps(steps, n_interp=n_interp)

    # Hold on the final pose for ~1 second so the robot appears to arrive.
    hold_frames = max(1, fps)
    steps = steps + [steps[-1]] * hold_frames

    fig, ax = plt.subplots(figsize=(10, 6.5))
    im = draw_terrain(ax, m, title=f"Plan: {m['name']}")
    plt.colorbar(im, ax=ax, label="elevation (mm)")
    draw_static_markers(ax, m)

    # Body trail line (accumulated).
    trail_x, trail_y = [s["bx"] for s in steps], [s["by"] for s in steps]
    ax.plot(trail_x, trail_y, "-", color="white", alpha=0.35, linewidth=1, zorder=3)

    # Artists we update per frame.
    body_dot, = ax.plot([], [], "o", color="red", markersize=8, zorder=8, label="body")
    heading_line, = ax.plot([], [], "-", color="red", linewidth=2, zorder=8)
    foot_scatters = {
        leg: ax.plot([], [], "o", markersize=6,
                     color=("steelblue" if leg[0] == "L" else "orange"),
                     markeredgecolor="black", zorder=7, label=leg)[0]
        for leg in LEGS
    }
    support_poly = Polygon(np.zeros((3, 2)), closed=True,
                           facecolor=(0.2, 0.8, 0.2, 0.25),
                           edgecolor=(0.0, 0.5, 0.0, 0.9),
                           linewidth=1.2, zorder=6)
    ax.add_patch(support_poly)

    step_txt = ax.text(0.02, 0.97, "", transform=ax.transAxes,
                       va="top", ha="left", color="white",
                       fontsize=10, bbox=dict(facecolor="black", alpha=0.5, pad=4))

    # Deduplicate legend entries.
    handles, labels = ax.get_legend_handles_labels()
    seen = set(); uniq_h = []; uniq_l = []
    for h, l in zip(handles, labels):
        if l in seen: continue
        seen.add(l); uniq_h.append(h); uniq_l.append(l)
    ax.legend(uniq_h, uniq_l, loc="upper right", ncol=2, fontsize=8)

    def update(i):
        s = steps[i]
        body_dot.set_data([s["bx"]], [s["by"]])
        hx = s["bx"] + 40 * np.cos(np.deg2rad(s["byaw"]))
        hy = s["by"] + 40 * np.sin(np.deg2rad(s["byaw"]))
        heading_line.set_data([s["bx"], hx], [s["by"], hy])
        for leg in LEGS:
            x, y, _ = s["feet"][leg]
            foot_scatters[leg].set_data([x], [y])

        # Support triangle = tripod that is currently grounded (NOT the
        # active_tripod, which is the one that lifts next). The three grounded
        # feet form the polygon. Equivalently: feet NOT in TRIPODS[active].
        anchored = [leg for leg in LEGS if leg not in TRIPODS[s["active_tripod"]]]
        pts = np.array([[s["feet"][leg][0], s["feet"][leg][1]] for leg in anchored])
        support_poly.set_xy(pts)

        step_txt.set_text(
            f"step {s['step']}  action: {s['action']}\n"
            f"body ({s['bx']:.0f}, {s['by']:.0f})  yaw {s['byaw']:.1f}°\n"
            f"support: {','.join(anchored)}"
        )
        return [body_dot, heading_line, support_poly, step_txt] + list(foot_scatters.values())

    anim = animation.FuncAnimation(fig, update, frames=len(steps),
                                   interval=1000 // fps, blit=False)
    anim.save(gif_path, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
    return gif_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="map directory (contains meta.yaml, heightmap.csv)")
    ap.add_argument("--plan", default="plan.txt")
    ap.add_argument("--gif", default=None, help="output gif path (default: <map>/plan.gif)")
    ap.add_argument("--fps", type=int, default=2)
    ap.add_argument("--interp", type=int, default=6,
                    help="interpolated sub-frames between plan steps (1=none)")
    ap.add_argument("--preview-only", action="store_true")
    args = ap.parse_args()

    m = load_map(args.map)
    preview = regenerate_preview(args.map, m)
    print(f"wrote {preview}")

    if args.preview_only:
        return

    meta, steps = parse_plan(args.plan)
    print(f"plan: {len(steps)} frames  {meta}")
    gif = args.gif or os.path.join(args.map, "plan.gif")
    render_animation(m, steps, meta, gif, fps=args.fps, n_interp=args.interp)
    print(f"wrote {gif}")


if __name__ == "__main__":
    main()
