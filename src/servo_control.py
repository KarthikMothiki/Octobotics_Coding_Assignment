#!/usr/bin/env python

import Jetson.GPIO as GPIO
import time

servo_pin = 13

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(servo_pin, GPIO.OUT)
    p = GPIO.PWM(servo_pin, 50)
    
    val = 90
    incr = 1
    p.start(0)

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            time.sleep(0.25)
            #print(val)
            for angle in range(0, val+1, 1):
                duty_cycle = angle / 18.0 + 2.0
                p.ChangeDutyCycle(duty_cycle)   
                time.sleep(0.05) 
            print 'Moved to', val, ' deg'  

            time.sleep(1)          

            for angle in range(val, 0, -1):
                duty_cycle = angle / 18.0 + 2.0
                p.ChangeDutyCycle(duty_cycle)   
                time.sleep(0.05)
            print 'Moved to', 0, ' deg' 

    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
