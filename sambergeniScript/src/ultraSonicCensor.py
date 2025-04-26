import RPi.GPIO as GPIO
import time

# Define GPIO pins
TRIGGER_PIN = 23
ECHO_PIN = 24

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    # Send trigger pulse
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(TRIGGER_PIN, GPIO.LOW)

    # Wait for echo signal to start
    start_time = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    # Wait for echo signal to end
    end_time = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    # Calculate distance (speed of sound = 343 m/s)
    duration = end_time - start_time
    distance = (duration * 34300) / 2  # Convert to cm
    return round(distance, 2)

try:
    while True:
        distance = get_distance()
        print(f"Distance: {distance} cm")
        time.sleep(0.5)  # Delay between measurements

except KeyboardInterrupt:
    print("\nMeasurement stopped by user.")
    GPIO.cleanup()


