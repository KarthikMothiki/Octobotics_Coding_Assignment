#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import time
import rospy
from std_srvs.srv import Trigger, TriggerResponse

servo_pin = 13
gpio_pin = 22

def pwm_control_callback(req):
    # Your PWM control logic here
    # For example, set the servo to a specific angle
    # Adjust val or any other parameters based on req data
    val = 90
    # Add any additional logic as needed

    # Return success message
    return TriggerResponse(success=True, message="PWM signal sent")

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # Set servo pin as an output pin with optional initial state of HIGH
    GPIO.setup(servo_pin, GPIO.OUT)
    p = GPIO.PWM(servo_pin, 50)
    
    val = 90
    incr = 1
    p.start(0)

    # Set GPIO pin 22 as input
    GPIO.setup(gpio_pin, GPIO.IN)

    # Initialize ROS node
    rospy.init_node('servo_control')

    # Create ROS service
    rospy.Service('/servo_control', Trigger, pwm_control_callback)

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            time.sleep(0.25)

            # Check if GPIO pin 22 is high or service is called
            if GPIO.input(gpio_pin) == GPIO.HIGH:
                # Send PWM signal
                for angle in range(0, val+1, 1):
                    duty_cycle = angle / 18.0 + 2.0
                    p.ChangeDutyCycle(duty_cycle)   
                    time.sleep(0.05) 
                print('Moved to', val, 'deg')  

                time.sleep(1)          

                for angle in range(val, 0, -1):
                    duty_cycle = angle / 18.0 + 2.0
                    p.ChangeDutyCycle(duty_cycle)   
                    time.sleep(0.05)
                print('Moved to', 0, 'deg') 

    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
