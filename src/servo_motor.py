#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import time
import rospy
from std_srvs.srv import Trigger, TriggerResponse

servo_pin = 15
trigger_pin = 7  # GPIO pin to trigger PWM signal
GPIO.setwarnings(False)

p = None  # Declare p globally

def servo_move(angle):
    print("Came in to the servo_move func")
    duty_0_deg = 2.5
    duty_180_deg = 12.5
    duty_cycle = ((angle /180.0) * (duty_180_deg - duty_0_deg)) + duty_0_deg
    p.ChangeDutyCycle(duty_cycle)
    print('Moved to', angle, ' deg')
    time.sleep(1)          
    for angle in range(angle, 0, -1):
        duty_cycle = angle / angle + 2.0
        p.ChangeDutyCycle(duty_cycle)
        time.sleep(0.05) 
    print('Moved to', 0, ' deg')

def pwm_control_callback(req):
    val = 8
    print("Came in to the callback func")
    print("PWM running. Press CTRL+C to exit.")
    try:
        if GPIO.input(trigger_pin) == GPIO.HIGH or rospy.get_param('/servo_control_1', True):
            servo_move(val)
            print("Executed servo_move function")
        else: 
            print("No signal received from Pin 7 or Service Call")
    except KeyboardInterrupt:
        pass
    # Return success message
    return TriggerResponse(success=True, message="PWM signal sent")

def main():
    global p

    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)

    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(trigger_pin, GPIO.IN)
    GPIO.setup(servo_pin, GPIO.OUT)

    p = GPIO.PWM(servo_pin, 50)
    p.start(0)

    # Initialize ROS node
    rospy.init_node('servo_control_1', anonymous=True)

    # Create ROS service
    rospy.Service('/servo_control_1', Trigger, pwm_control_callback)

    try:
        rospy.spin()  # Enter ROS event loop
    except KeyboardInterrupt:
        pass
    finally:
        # Stop PWM and cleanup GPIO pins
        if p is not None:
            p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
