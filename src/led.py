# led.py
#!/usr/bin/env python

import Jetson.GPIO as GPIO
import time

# Pin Definitons:
led_pin = 18  # BOARD pin 12

def main():
    prev_value = None

    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output

    # Initial state for LEDs:
    GPIO.output(led_pin, GPIO.LOW)
    time.sleep(2)
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            GPIO.output(led_pin, GPIO.HIGH)
            print 'high'
            time.sleep(1)
            GPIO.output(led_pin, GPIO.LOW)
            print 'low'
            time.sleep(1)

    finally:
        GPIO.cleanup()  # cleanup all GPIO

if __name__ == '__main__':
    main()
