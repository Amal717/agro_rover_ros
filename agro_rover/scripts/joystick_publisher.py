#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Threshold for joystick movement
MOVEMENT_THRESHOLD = 0.1

def joystick_callback(msg):
    twist_msg = Twist()

    # Check if there is significant movement in the joystick
    if abs(msg.axes[1]) > MOVEMENT_THRESHOLD or abs(msg.axes[2]) > MOVEMENT_THRESHOLD:
        # Customize the mapping of joystick axes to Twist components as needed
        twist_msg.linear.x = msg.axes[1]
        twist_msg.angular.z = msg.axes[2]
        pub.publish(twist_msg)

        # Print joystick values to the screen
        rospy.loginfo(f"Joystick values - Linear: {twist_msg.linear.x}, Angular: {twist_msg.angular.z}")

def main():
    rospy.init_node('joystick_publisher')
    global pub
    pub = rospy.Publisher('joystick_twist', Twist, queue_size=10)
    rospy.Subscriber('joy', Joy, joystick_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
