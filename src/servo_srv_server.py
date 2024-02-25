import rospy
from std_srvs.srv import Trigger

rospy.init_node('service_client')  # Initialize ROS node

# Wait for the service to become available
rospy.wait_for_service('/servo_control')

try:
    # Create a handle to the service
    servo_control = rospy.ServiceProxy('/servo_control', Trigger)

    # Call the service
    response = servo_control()

    # Process the response
    if response.success:
        print("Service call successful:", response.message)
    else:
        print("Service call failed:", response.message)

except rospy.ServiceException as e:
    print("Service call failed:", e)
