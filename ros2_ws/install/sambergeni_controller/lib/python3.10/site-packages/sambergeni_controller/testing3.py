#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from adafruit_servokit import ServoKit
import Adafruit_PCA9685


pwm = Adafruit_PCA9685.PCA9685()


class MyNode(Node):
    def __init__(self):
        super().__init__("second_node")
        
        self.create_timer(1.0, self.timer)

    # def set_servo_pulse(channel, pulse):
    #     pulse_length = 1000000    # 1,000,000 us per second
    #     pulse_length //= 60       # 60 Hz
    #     print('{0}us per period'.format(pulse_length))
    #     pulse_length //= 4096     # 12 bits of resolution
    #     print('{0}us per bit'.format(pulse_length))
    #     pulse *= 1000
    #     pulse //= pulse_length
    #     pwm.set_pwm(channel, 0, pulse)


    def timer(self):
        def map_value(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        def angle_to_pulse(ang, servomin, servomax):
            pulse = map_value(ang, 0,180,servomin,servomax)
            return pulse

        self.get_logger().info("Hello dunia")
        servo_min = 125
        servo_max = 625
        pwm.set_pwm_freq(60)
        
        angle = 90
        pulse = angle_to_pulse(angle, servo_min, servo_max)

        pwm.set_pwm(0,0,servo_min)
        time.sleep(1)
        pwm.set_pwm(0,0,int(pulse))
        time.sleep(1)
        pwm.set_pwm(0,0,servo_max)    
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


