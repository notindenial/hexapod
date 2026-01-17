# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
from adafruit_servokit import ServoKit

# ---- Configuration ----
SERVO_CHANNEL = 0
CHANNELS = 8
STEP_DEG = 10        # angle step size
HOLD_TIME = 2.0      # seconds to pause at each angle

kit = ServoKit(channels=CHANNELS, address=0x41)
servo = kit.servo[SERVO_CHANNEL]

servo.set_pulse_width_range(min_pulse=500, max_pulse=2500)

print("setting 0 angle.")
servo.angle = 0 
time.sleep(5)

print("setting 90 angle.")
servo.angle = 90
time.sleep(5)

print("setting 180 angle.")
servo.angle = 180
time.sleep(5)

servo.angle = 0
time.sleep(1)

for a in range(0, 181, 2):
    servo.angle = a
    time.sleep(0.02)

for a in range(180, -1, -2):
    servo.angle = a
    time.sleep(0.02)

print("Test complete.")
