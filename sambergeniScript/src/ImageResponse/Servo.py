import time
from adafruit_servokit import ServoKit
import Adafruit_PCA9685
import math

pwm = Adafruit_PCA9685.PCA9685()
SERVOMIN = 125
SERVOMAX = 625
pwm.set_pwm_freq(60)

a1 = 8
a2 = 14.5

servo1_offset = 90
servo2_offset = 90
servo1_min = 0
servo1_max = 180
servo2_min = 0
servo2_max = 180

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class JointAngles:
    def __init__(self, theta1=0, theta2=0, theta3=0):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3

def inverse_kinematics(target):
    angles = JointAngles()
    
    q0 = math.degrees(math.atan2(target.y, target.x))
    r = math.sqrt(target.y ** 2 + target.z ** 2)
    
    if r > (a1 + a2) or r < abs(a1 - a2):
        print("Target position is not reachable!")
        angles.theta1 = servo1_offset
        angles.theta2 = servo2_offset
        return angles
    
    q2 = math.degrees(math.acos((r ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2)))
    a2_sin_q2 = a2 * math.sin(math.radians(q2))
    a2_cos_q2 = a2 * math.cos(math.radians(q2))
    
    beta = math.degrees(math.atan2(a2_sin_q2, a1 + a2_cos_q2))
    gamma = math.degrees(math.atan2(target.z, target.y))
    q1 = gamma - beta
    
    reverse = abs(servo2_offset - q2)
    
    angles.theta1 = q0
    angles.theta2 = max(min(q1 + servo1_offset, servo1_max), servo1_min)
    angles.theta3 = max(min(servo2_offset - reverse, servo2_max), servo2_min)
    
    return angles

def angle_to_pulse(angle):
    pulse = int((angle - 0) * (SERVOMAX - SERVOMIN) / (180 - 0) + SERVOMIN)
    # print(f"Angle: {angle} pulse: {pulse}")
    return pulse

class ServoController():
    def __init__(self):
        self.points = [
            Point(0, 8, 14.5),
            Point(0, 5.853531763034923, 8.648622196577978),
            Point(5, 8.239610763246004, 6.41648770513905),
            Point(10, 8.239610763246004, 6.41648770513905)
            # Point(10, 8.239610763246004, 6.41648770513905)
        ]
        
        for i in range(4):
            self.angles = inverse_kinematics(self.points[i])
            self.move_servo_smooth(self.angles)
        
    def move_servo_smooth(self, angles):
        pwm.set_pwm(2,0, angle_to_pulse(angles.theta1))
        pwm.set_pwm(1,0, angle_to_pulse(angles.theta2))
        pwm.set_pwm(0,0, angle_to_pulse(angles.theta3))
        time.sleep(0.1)