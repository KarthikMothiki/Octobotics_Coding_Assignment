#!/usr/bin/env python

import rospy
from inverted_pendulum_sim.msg import ControlForce
from math import sin, pi

def control_force_publisher():
    rospy.init_node("control_force_publisher", anonymous=True)
    ctrl_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
    rate = rospy.Rate(100)  # Adjust the rate based on your requirements

    amplitude = 10.0  # Adjust the amplitude of the sinusoidal force
    frequency = 0.5  # Adjust the frequency of the sinusoidal force in Hz
    phase = 0.0  # Adjust the phase of the sinusoidal force

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_time).to_sec()
        # elapsed_time = 1
        force_value = amplitude * sin(2 * pi * frequency * elapsed_time + phase)

        force_msg = ControlForce()
        force_msg.force = force_value

        ctrl_force_pub.publish(force_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_force_publisher()
    except rospy.ROSInterruptException:
        pass
