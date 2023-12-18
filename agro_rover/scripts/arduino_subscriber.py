#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32  # Change from Int16 to Int32

def twist_callback(msg):
    # Scale joystick values to match the expected range (0 to 255)
    x_axis_value = int((msg.linear.x + 1.0) * 127.5)  # Map [-1.0, 1.0] to [0, 255]
    Y_axis_value = int((msg.angular.z + 1.0) * 127.5)

    # Construct the command strings in the specified format
    x_axis_command = Int32(data=x_axis_value)  # Change from Int16 to Int32
    Y_axis_command = Int32(data=Y_axis_value)  # Change from Int16 to Int32

    # Print the commands to be sent to the Arduino
    rospy.loginfo("Sending to Arduino: %s", x_axis_command)

    # Publish the X and Y axis values to ROS topics
    x_axis_publisher.publish(x_axis_command)
    Y_axis_publisher.publish(Y_axis_command)

def main():
    global x_axis_publisher, Y_axis_publisher

    rospy.init_node('arduino_publisher')

    # Create publishers for X-axis and Y-axis values
    x_axis_publisher = rospy.Publisher('x_axis_command', Int32, queue_size=10)  # Change from Int16 to Int32
    Y_axis_publisher = rospy.Publisher('Y_axis_command', Int32, queue_size=10)  # Change from Int16 to Int32

    # Subscribe to the joystick_twist topic
    rospy.Subscriber('joystick_twist', Twist, twist_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

