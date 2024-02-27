#!/usr/bin/env python

import Jetson.GPIO as GPIO
import time
import rospy
from std_srvs.srv import Trigger, TriggerResponse

servo_pin = 18
gpio_pin = 7
p = None  # Global variable to hold PWM object

def move_motor(angle):
    """
    Move the servo motor to the specified angle.

    Args:
        angle (int): The angle to move the servo motor to.

    Returns:
        str: A message indicating that the PWM signal has been sent.
    """
    global p  # Access the global PWM object

    # Start PWM with duty cycle corresponding to the specified angle
    p.start(0)
    duty_0_deg = 2.5
    duty_cycle = ((angle / 180.0) * (12.5 - duty_0_deg)) + duty_0_deg
    p.ChangeDutyCycle(duty_cycle)

    print('Moved to', angle, ' deg')
    time.sleep(1)

    # Set duty cycle for 0 degrees to bring motor back to 0 position
    duty_cycle_0_deg = ((0 / 180.0) * (12.5 - duty_0_deg)) + duty_0_deg
    p.ChangeDutyCycle(duty_cycle_0_deg)

    print('Moved to', 0, ' deg')

    mes = "PWM signal sent"
    return mes

def pwm_control_callback(req):
    """
    ROS service callback function to handle servo control.

    Args:
        req: The request object.

    Returns:
        std_srvs.srv.TriggerResponse: A response indicating the success of the operation.
    """
    # Move the motor to 90 degrees
    mes = move_motor(90)

    # Return success message
    return TriggerResponse(success=True, message=mes)

def main():
    """
    Main function to initialize GPIO pins and ROS node, and start servo control service.
    """
    global p  # Access the global PWM object
    try:
        # Pin Setup:
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        # Set servo pin as an output pin with optional initial state of HIGH
        GPIO.setup(servo_pin, GPIO.OUT)
        GPIO.setup(gpio_pin, GPIO.IN)
        p = GPIO.PWM(servo_pin, 50)

        # Set GPIO pin as input
        GPIO.setup(gpio_pin, GPIO.IN)

        # Initialize ROS node
        rospy.init_node('servo_control')

        # Create ROS service
        rospy.Service('/servo_control', Trigger, pwm_control_callback)

        print ('PWM running. Press CTRL+C to exit.')
        while not rospy.is_shutdown():
            time.sleep(0.25)

            # Check if GPIO pin is high or service is called
            if GPIO.input(gpio_pin) == GPIO.HIGH or rospy.get_param('/servo_control', False):

                # Send PWM signal
                pwm_control_callback(None)

    except KeyboardInterrupt:
        pass
    finally:
        if p is not None:
            p.stop()
            GPIO.cleanup()

if __name__ == '__main__':
    main()
