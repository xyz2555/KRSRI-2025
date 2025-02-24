#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

TRIGGER_PIN = 23
ECHO_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

class MyNode(Node):
    def __init__(self):
        super().__init__("second_node")
        
        # self.get_logger().info("halooo")
        self.create_timer(0.1, self.timer)

    def timer(self):
        def get_distance():
            GPIO.output(TRIGGER_PIN, GPIO.HIGH)
            time.sleep(0.00001)  # 10µs pulse
            GPIO.output(TRIGGER_PIN, GPIO.LOW)

            start_time = time.time()
            while GPIO.input(ECHO_PIN) == 0:
                start_time = time.time()

            end_time = time.time()
            while GPIO.input(ECHO_PIN) == 1:
                end_time = time.time()

            duration = end_time - start_time
            distance = (duration * 34300) / 2  # Convert to cm
            return round(distance, 2)

        self.get_logger().info("Hello dunia")
        distance = get_distance()
        print(f"Distance: {distance} cm")
        time.sleep(0.5)  # Delay between measurements

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

#from gpiozero import DistanceSensor
#ultrasonic = DistanceSensor(echo=17, trigger=4)
#while True:
#    print(ultrasonic.distance)

      
