# Carcinization — Hexapod Robot

A 6-legged walking robot built for CMU Build18. Each leg has 3 DOF (pan/yaw, hip, knee), giving 18 servo channels total split across two PCA9685 drivers.

---

## Hardware Stack

| Component | Details |
|-----------|---------|
| **Computer** | Raspberry Pi 5 |
| **Servo Driver (Left)** | Adafruit PCA9685 — I²C address `0x40` (default) |
| **Servo Driver (Right)** | Adafruit PCA9685 — I²C address `0x41` (A0 pulled high) |
| **Servos** | Generic MG996R-class metal gear, 180° range |
| **Power** | 5V logic (Pi + PCA9685 VCC), 6V servo rail |

---

## Servo Wiring

Legs are named by side (L/R) and position (F/M/B). Each leg has three servos: **pan** (yaw about vertical), **hip** (pitch, upper link), **knee** (pitch, lower link).

Left legs are **mirror-mounted** — the physical servo is installed upside-down relative to the right side. The firmware compensates with `angle = 180 - logical_angle` for all left-side joints.

### Left Board — PCA9685 @ 0x40

| Leg | Pan ch | Hip ch | Knee ch | Notes |
|-----|--------|--------|---------|-------|
| LF (Left Front) | 2 | 1 | 0 | knee pulse max 2700 µs |
| LM (Left Mid)   | 6 | 5 | 4 | knee pulse max 2700 µs |
| LB (Left Back)  | 10 | 9 | 8 | knee pulse max 2700 µs |

### Right Board — PCA9685 @ 0x41

| Leg | Pan ch | Hip ch | Knee ch |
|-----|--------|--------|---------|
| RF (Right Front) | 6  | 5  | 4  |
| RM (Right Mid)   | 10 | 9  | 8  |
| RB (Right Back)  | 14 | 13 | 12 |

**Pulse range:** 500 – 2500 µs (left knees: 500 – 2700 µs)  
**PWM frequency:** 50 Hz

---

## Leg Geometry

```
Body center
    │
    ├── coxa  (L1 = 55 mm)  ← pan joint rotates here
    │
    └── femur (L2 = 85 mm)  ← hip pitch, knee pitch at end
```

Two-link planar IK in the (r, z) plane after yaw is resolved. See `leg.py` for the closed-form solution and `legacy/helper.py` for the DH-parameter / Jacobian formulation.

**Coordinate frame (per leg, origin at mount point):**
- `x` — forward (robot heading)
- `y` — outward (away from body)
- `z` — up

---

## Control Stack

```
main.py          Entry point. Loads config, inits hardware, runs control loop.
body.py          Body pose and gait coordination across all 6 legs.
leg.py           Single leg: IK/FK, joint limits, servo rate limiting.
animations.py    Canned motion sequences (wave, bow, dance, etc.).
planner.py       Global path planner — ARA* (stub, to be implemented).
config.yaml      All hardware constants, wiring, and tuning parameters.
```

### Module Responsibilities

**`leg.py` — `Leg` class**
- Local coordinate frame centered at leg mount point
- Closed-form 2-DOF IK (`set_foot_xyz`) and servo-angle interface (`set_angles`)
- Rate-limited `update(dt)` that ramps joints toward targets at `max_speed_deg_s`
- Hardware-agnostic: accepts a `ServoDriver` protocol so the same class runs on hardware and in simulation

**`body.py` — `Body` class**
- Owns all 6 `Leg` instances
- Transforms foot targets from world frame → body frame → leg-local frame
- Body pose control: translation (x, y, z) + roll/pitch/yaw of the chassis
- Gait primitives: `tripod_step(vx, vy, omega)`, `wave_gait(...)` (planned)
- Calls `leg.update(dt)` for all legs each tick

**`main.py`**
- Loads `config.yaml` via PyYAML
- Instantiates PCA9685 `ServoKit` objects and passes them to `Body`
- Runs the 50 Hz control loop; handles keyboard / joystick input

**`planner.py`**
- Global (x, y, θ) path planner using ARA* (Anytime Repairing A*)
- Outputs velocity commands `(vx, vy, omega)` consumed by `body.py`
- Stub only — to be implemented

**`animations.py`**
- Pre-scripted keyframe sequences played back through `Body`
- Examples: `wave()`, `bow()`, `stand_up()`, `sit_down()`

---

## Simulation

A `sim/` package mirrors the hardware classes so the full control stack can run without a physical robot.

```
sim/
  sim_servo.py    Drop-in replacement for adafruit_servokit — logs or visualises angles.
  sim_body.py     Wraps Body with a visualiser (e.g. matplotlib or pybullet).
  urdf/           Robot URDF for pybullet (planned).
```

Set `SIM=1` in the environment (or pass `--sim` to `main.py`) to swap in simulation backends.

---

## Legacy Code

The original Build18 scripts are preserved in `legacy/` for reference:

| File | Description |
|------|-------------|
| `legacy/main.py` | Structured bring-up with joint limits and rate limiting |
| `legacy/leg.py` | Working 2-DOF IK leg class (basis for new `leg.py`) |
| `legacy/helper.py` | DH-parameter FK + Jacobian IK (experimental, not wired to hardware) |
| `legacy/servo.py` | Hardcoded tripod gait — forward walk |
| `legacy/crab.py` | Hardcoded tripod gait — lateral/crab walk |
| `legacy/thankyou.py` | Stand/sit demo sequence |
| `legacy/leg_stupid.py` | Early `Leg_Stupid` prototype with `Bot` class |

---

## Quick Start

```bash
# Install dependencies (on RPi)
pip install adafruit-circuitpython-servokit pyyaml numpy

# Run on hardware
python main.py

# Run in simulation
python main.py --sim
```
