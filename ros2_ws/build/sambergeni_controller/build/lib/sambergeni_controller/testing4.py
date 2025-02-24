#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from adafruit_servokit import ServoKit

class MyNode(Node):
    def __init__(self):
        super().__init__("fourth_node")
        self.create_timer(1.0, self.timer)

    def timer(self):
        self.get_logger().info("Hello dunia")
        
        kit = ServoKit(channels=8)
    
        kit.servo[1].angle = 0
        time.sleep(1)
        kit.servo[1].angle = 90
        time.sleep(1)
        kit.servo[1].angle = 180
        time.sleep(1)
        kit.servo[1].angle = 270
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


