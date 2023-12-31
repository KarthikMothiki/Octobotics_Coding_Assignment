#!/usr/bin/env python

import rospy
from math import pi
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState
from std_msgs.msg import Float64

class PIDTuner:
    def __init__(self):
        rospy.init_node("pid_tuner", anonymous=True)

        # PID controller parameters
        self.Kp = 1.0
        self.Ki = 0.5
        self.Kd = 0.01

        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        print(self.integral)

        # Set up publishers and subscribers
        self.control_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
        self.theta_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.theta_callback)

    def theta_callback(self, msg):
        # Desired angle for the pendulum to be balanced (inverted)
        desired_theta = pi

        # Calculate error terms
        error = desired_theta - msg.curr_theta
        self.integral += error * 0.1 # Adjust the integration time

        # Calculate control force using PID formula
        control_force = self.Kp * error + self.Ki * self.integral + self.Kd * (error - self.prev_error) / 0.1

        # Apply the control force to the pendulum
        control_msg = ControlForce()
        control_msg.force = control_force
        self.control_force_pub.publish(control_msg)

        # Update previous error for the next iteration
        self.prev_error = error
        print(self.prev_error )

if __name__ == '__main__':
    pid_tuner = PIDTuner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[PIDTuner]: Shutting down node")
