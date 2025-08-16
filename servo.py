import math

SERVOMIN = 125
SERVOMAX = 625

a1 = 8
a2 = 14.5

servo1Offset = 90
servo2Offset = 90;  
servo1Min = 0;      
servo1Max = 180;    
servo2Min = 0;      
servo2Max = 180;    

class Point :
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class JointAngles:
    def __init__(self, theta1, theta2, theta3):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3


def inverseKinematics(Point target):
    JointAngles angles

    q0 = math.atan2(target.y, target.x) * 180/math.PI
    r = math.sqrt(target.y**2 + target.z**2)

    if r > (a1 + a2) OR r < abs(a1-a2):
        print
