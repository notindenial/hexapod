import time
#import keyboard
from adafruit_servokit import ServoKit

#Make Leg Class
#Lift Function
#Drop Function
#Stand Function(L Leg position)
#Right leg(0) Left Leg(180) for L Leg
# ---- Configuration ----
CHANNELS = 16
STEP_DEG = 10        # angle step size
HOLD_TIME = 2.0      # seconds to pause at each angle

L_FRONT = 0
L_MID = 4
L_BACK = 8

R_FRONT = 12
R_MID = 8
R_BACK = 4

r_kit = ServoKit(channels=CHANNELS, address=0x41)
l_kit = ServoKit(channels=CHANNELS)



class Leg_Stupid:
    def __init__(self, kit, side, position):
        center=90
        offset=0
        if side == "RIGHT":
            center= -90
        self.side = side
        self.center = center
        self.knee = kit.servo[position]
        self.hip = kit.servo[position+1]
        self.pan = kit.servo[position+2]
        self.knee.set_pulse_width_range(min_pulse=500, max_pulse=2700)
        self.hip.set_pulse_width_range(min_pulse=500, max_pulse=2700)
        self.pan.set_pulse_width_range(min_pulse=500, max_pulse=2700) 
    
    def knee_control(self, angle):
        self.knee.angle  = abs((self.center+angle))
    
    def hip_control(self, angle):
        self.hip.angle = abs((self.center+angle))
        
    def pan_control(self, angle):
        self.pan.angle = abs((self.center+angle))
        
    def leg_standard(self):
        self.knee_control(90)
        self.hip_control(-75)
        self.pan_control(0)
        
    def leg_lift(self):
        for ang in range(0, 31, 1):
            self.hip_control(ang)
    


class Bot:
    def __init__(self, legs):
        self.legs = legs
    
    def default(self):
        for leg in legs:
            leg.leg_standard()
    
    def stand(self):
        for ang in range(-75, 1, 1):
            for leg in legs:
                leg.hip_control(ang)
            time.sleep(0.02)
    
    def sit(self):
        for ang in range(0, -75, -1):
            for leg in legs:
                leg.hip_control(ang)
            time.sleep(0.01)

    def wave(self, num_waves):
        for ang in range(0, -75, -1):
            self.legs[0].hip_control(ang)
            time.sleep(0.01)
        for i in range(0,num_waves,1):
            for ang in range(90,-30,1):
                self.legs[0].knee_control(ang)
                time.sleep(0.005)
            for ang in range(-30,90,1):
                self.legs[0].knee_control(ang)
                time.sleep(0.005)
    def double_wave(self, num_waves):
        for ang in range(0, -75, -1):
            self.legs[1].hip_control(ang)
            self.legs[4].hip_control(ang)
            time.sleep(0.01)
        for i in range(0,num_waves,1):
            for ang in range(90,-30,1):
                self.legs[1].knee_control(ang)
                self.legs[4].knee_control(ang)
                time.sleep(0.001)
            for ang in range(-30,90,1):
                self.legs[1].knee_control(ang)
                self.legs[4].knee_control(ang)
                time.sleep(0.001)
    def store(self):
        for leg in legs:
            leg.knee_control(-80)
    def rave(self):
        for ang in range(0, 20, 1):
            for leg in legs:
                leg.hip_control(ang)
            time.sleep(0.01)
        for ang in range(90, 80, -1):
            for i in range(0,3,1):
                leg.knee_control(ang)
            time.sleep(0.01)
#Initialize right legs
r_front = Leg_Stupid(r_kit,"RIGHT",R_FRONT)
r_mid = Leg_Stupid(r_kit,"RIGHT",R_MID)
r_back = Leg_Stupid(r_kit,"RIGHT",R_BACK)

#initialize left legs
l_front = Leg_Stupid(l_kit,"LEFT",L_FRONT)
l_mid = Leg_Stupid(l_kit,"LEFT",L_MID)
l_back = Leg_Stupid(l_kit,"LEFT",L_BACK)

legs = [l_front,l_mid,l_back,r_front,r_mid,r_back]

Hex = Bot(legs)
Hex.default()

time.sleep(1)
Hex.store()

# time.sleep(1)
# print("Standing")
# Hex.stand()

# time.sleep(1)
# print("Waving")
# Hex.wave(3)

# time.sleep(.5)
# print("Sitting")
# Hex.sit()

# time.sleep(.5)
# print("Standing")
# Hex.stand()

# time.sleep(1)
# print("Waving")
# Hex.double_wave(10)

# time.sleep(1)
# print("Sitting")
# Hex.sit()
