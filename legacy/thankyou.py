# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
from adafruit_servokit import ServoKit

# ---- Configuration ----
CHANNELS = 16
STEP_DEG = 10        # angle step size
HOLD_TIME = 2.0      # seconds to pause at each angle

r_kit = ServoKit(channels=CHANNELS, address=0x41)
r_knee_front  = r_kit.servo[4]
r_hip_front   = r_kit.servo[5]
r_pan_front   = r_kit.servo[6]
r_knee_front.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_hip_front.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_pan_front.set_pulse_width_range(min_pulse=500, max_pulse=2500) 
r_knee_front.angle  = 0
r_hip_front.angle = 165
r_pan_front.angle = 90

r_knee_mid    = r_kit.servo[8]
r_hip_mid     = r_kit.servo[9]
r_pan_mid   = r_kit.servo[10]
r_knee_mid.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_hip_mid.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_pan_mid.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_knee_mid.angle  = 0
r_hip_mid.angle = 165 
r_pan_mid.angle = 90 

r_knee_back   = r_kit.servo[12]
r_hip_back    = r_kit.servo[13]
r_pan_back    = r_kit.servo[14]
r_knee_back.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_hip_back.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_pan_back.set_pulse_width_range(min_pulse=500, max_pulse=2500)
r_knee_back.angle  = 0
r_hip_back.angle = 165
r_pan_back.angle = 90

l_kit = ServoKit(channels=CHANNELS)
l_knee_front  = l_kit.servo[0]
l_hip_front   = l_kit.servo[1]
l_pan_front   = l_kit.servo[2]
l_knee_front.set_pulse_width_range(min_pulse=500, max_pulse=2700)
l_hip_front.set_pulse_width_range(min_pulse=500, max_pulse=2500)
l_pan_front.set_pulse_width_range(min_pulse=500, max_pulse=2500) 
l_knee_front.angle  = 180
l_hip_front.angle = 180 - 165
l_pan_front.angle = 90

l_knee_mid    = l_kit.servo[4]
l_hip_mid     = l_kit.servo[5]
l_pan_mid   = l_kit.servo[6]
l_knee_mid.set_pulse_width_range(min_pulse=500, max_pulse=2700)
l_hip_mid.set_pulse_width_range(min_pulse=500, max_pulse=2500)
l_pan_mid.set_pulse_width_range(min_pulse=500, max_pulse=2500)
l_knee_mid.angle  = 180
l_hip_mid.angle = 180 - 165 
l_pan_mid.angle = 90 

l_knee_back   = l_kit.servo[8]
l_hip_back    = l_kit.servo[9]
l_pan_back    = l_kit.servo[10]
l_knee_back.set_pulse_width_range(min_pulse=500, max_pulse=2700)
l_hip_back.set_pulse_width_range(min_pulse=500, max_pulse=2500)
l_pan_back.set_pulse_width_range(min_pulse=500, max_pulse=2500)
l_knee_back.angle  = 180
l_hip_back.angle = 180 - 165
l_pan_back.angle = 90






time.sleep(2)
print("Standing ")
for ang in range(165, 70, -1):
    r_hip_front.angle = ang
    r_hip_mid.angle = ang
    r_hip_back.angle = ang
    l_hip_front.angle = 180 - ang
    l_hip_mid.angle = 180 - ang
    l_hip_back.angle = 180 - ang
    time.sleep(0.02)



print("Standing ")
for ang in range(165, 70, -1):
    r_hip_front.angle = ang
    r_hip_mid.angle = ang
    r_hip_back.angle = ang
    l_hip_front.angle = 180 - ang
    l_hip_mid.angle = 180 - ang
    l_hip_back.angle = 180 - ang
    time.sleep(0.02)
print("Standing ")
for ang in range(165, 70, -1):
    r_hip_front.angle = ang
    r_hip_mid.angle = ang
    r_hip_back.angle = ang
    l_hip_front.angle = 180 - ang
    l_hip_mid.angle = 180 - ang
    l_hip_back.angle = 180 - ang
    time.sleep(0.02)
print("Standing ")
for ang in range(165, 70, -1):
    r_hip_front.angle = ang
    r_hip_mid.angle = ang
    r_hip_back.angle = ang
    l_hip_front.angle = 180 - ang
    l_hip_mid.angle = 180 - ang
    l_hip_back.angle = 180 - ang
    time.sleep(0.02)


time.sleep(1)
for ang in range(90, 166, 1):
    r_hip_front.angle = ang
    r_hip_mid.angle = ang
    r_hip_back.angle = ang
    l_hip_front.angle = 180 - ang
    l_hip_mid.angle = 180 - ang
    l_hip_back.angle = 180 - ang
    time.sleep(0.02)

print("Test complete.")
