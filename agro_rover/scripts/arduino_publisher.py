#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, Int32

def axis_values_callback(msg):
    # Extract relevant axis values directly from the received message
    x_axis_value = msg.data[0]
    y_axis_value = msg.data[1]
    trigger_left_value = msg.data[4]
    trigger_right_value = msg.data[5]

    # Publish the values to ROS topics
    x_axis_publisher.publish(Int32(data=x_axis_value))
    y_axis_publisher.publish(Int32(data=y_axis_value))
    trigger_left_publisher.publish(Int32(data=trigger_left_value))
    trigger_right_publisher.publish(Int32(data=trigger_right_value))

    # Print the values for debugging
    rospy.loginfo("X Axis: %s, Y Axis: %s, Trigger Left: %s, Trigger Right: %s",
                  x_axis_value, y_axis_value, trigger_left_value, trigger_right_value)

def main():
    global x_axis_publisher, y_axis_publisher, trigger_left_publisher, trigger_right_publisher

    rospy.init_node('arduino_publisher')

    # Create publishers for X-axis, Y-axis, and trigger values
    x_axis_publisher = rospy.Publisher('X_axis_command', Int32, queue_size=10)
    y_axis_publisher = rospy.Publisher('Y_axis_command', Int32, queue_size=10)
    trigger_left_publisher = rospy.Publisher('TriggerL_command', Int32, queue_size=10)
    trigger_right_publisher = rospy.Publisher('TriggerR_command', Int32, queue_size=10)

    # Subscribe to the 'axis_values' topic
    rospy.Subscriber('axis_values', Int32MultiArray, axis_values_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
