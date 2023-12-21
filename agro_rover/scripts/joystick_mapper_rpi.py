#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Int32MultiArray

# Global dictionary to store the previous button and axis values
prev_values = {'buttons': {}, 'axes': {}}

# Updated button and axis mapping
button_mapping = {
    0: "A",
    1: "B",
    2: "N",
    3: "X",
    4: "Y",
    5: "GUIDE",
    6: "LEFTSTICK",
    7: "RIGHTSTICK",
    8: "RIGHTSTICK",
    9: "LEFTSHOULDER",
    10: "SELECT",
    11: "START",
    12: "HOME",
    13: "DPAD_LEFT",
    14: "DPAD_RIGHT",
}

axis_mapping = {
    0: "LEFTX",
    1: "LEFTY",
    2: "RIGHTX",
    3: "RIGHTY",
    4: "TRIGGERLEFT",
    5: "TRIGGERRIGHT",
    6: "cross key left/right",
    7: "cross key up/down",
}

# Specify whether each axis should be inverted
inverted_axes = {
    "LEFTX": False,
    "LEFTY": True,
    "RIGHTX": False,
    "RIGHTY": True,
    "TRIGGERLEFT": False,
    "TRIGGERRIGHT": False,
    "cross key left/right": False,
    "cross key up/down": False,
}

def map_to_100(value, inverted):
    # Map the value from -1 to 1 to 0 to 100, with optional inversion
    if inverted:
        mapped_value = (value + 1) * 50
    else:
        mapped_value = (1 - value) * 50
    return int(max(0, min(100, mapped_value)))

def joystick_callback(msg):
    global prev_values

    # Publish button values
    button_values = [msg.buttons[index] for index in button_mapping]
    button_pub.publish(Int32MultiArray(data=button_values))

    # Publish axis values
    axis_values = [map_to_100(msg.axes[index], inverted_axes[axis_mapping[index]]) for index in axis_mapping]
    axis_pub.publish(Int32MultiArray(data=axis_values))

    # Check and publish button values if there's a change
    for index, button_name in button_mapping.items():
        if msg.buttons[index] != prev_values['buttons'].get(index):
            rospy.loginfo(f"Button {button_name}: {msg.buttons[index]}")
            prev_values['buttons'][index] = msg.buttons[index]

    # Check and publish axis values if there's a change
    for index, axis_name in axis_mapping.items():
        if msg.axes[index] != prev_values['axes'].get(index):
            rospy.loginfo(f"Axis {axis_name}: {msg.axes[index]}")
            prev_values['axes'][index] = msg.axes[index]

def main():
    global prev_values, button_pub, axis_pub
    rospy.init_node('joystick_publisher')

    # Create publishers for button and axis values
    button_pub = rospy.Publisher('button_values', Int32MultiArray, queue_size=10)
    axis_pub = rospy.Publisher('axis_values', Int32MultiArray, queue_size=10)

    rospy.Subscriber('joy', Joy, joystick_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

