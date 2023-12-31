#!/usr/bin/env python

import rospy
from inverted_pendulum_sim.msg import CurrentState, ControlForce
from simple_pid import PID

class InvertedPendulumController:
    def __init__(self):
        rospy.init_node("inverted_pendulum_controller", anonymous=True)
        self.control_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.state_callback)
        
        # Initialize PID controllers for theta and x
        self.pid_theta = PID(3.95, 3.1, 0.01, setpoint=0)  # Tune these gains as needed
        self.pid_x = PID(4.3, 1.3, 0.01, setpoint=0)  # Tune these gains as needed

    def state_callback(self, msg):
        theta_feedback = msg.curr_theta
        x_feedback = msg.curr_x

        # Update setpoints for the PID controllers (desired theta and x)
        self.pid_theta.setpoint = 0  # Set desired theta
        self.pid_x.setpoint = 0  # Set desired x

        # Compute PID outputs
        control_force_theta = self.pid_theta(theta_feedback)
        control_force_x = self.pid_x(x_feedback)

        # Combine or manipulate these control forces as needed for your specific control strategy
        total_control_force = control_force_theta + control_force_x  # Modify this according to your requirements

        # Publish the combined control force
        control_msg = ControlForce()
        control_msg.force = total_control_force
        self.control_force_pub.publish(control_msg)

if __name__ == '__main__':
    pendulum_controller = InvertedPendulumController()
    rospy.spin()