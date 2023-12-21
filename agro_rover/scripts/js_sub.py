#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

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
    7: "cross key up/down"
}

# Global dictionaries to store the previous button and axis values
prev_button_values = {index: 0 for index in button_mapping}
prev_axis_values = {index: 0 for index in axis_mapping}

def button_callback(msg):
    global prev_button_values
    rospy.loginfo("Received Button Values:")
    for i, value in enumerate(msg.data):
        button_name = button_mapping.get(i, f"Unknown Button {i}")
        if value != prev_button_values[i]:
            rospy.loginfo(f"{button_name}: {value}")
            prev_button_values[i] = value

def axis_callback(msg):
    global prev_axis_values
    rospy.loginfo("Received Axis Values:")
    for i, value in enumerate(msg.data):
        axis_name = axis_mapping.get(i, f"Unknown Axis {i}")
        if value != prev_axis_values[i]:
            rospy.loginfo(f"{axis_name}: {value}")
            prev_axis_values[i] = value

def main():
    rospy.init_node('joystick_subscriber')

    rospy.Subscriber('button_values', Int32MultiArray, button_callback)
    rospy.Subscriber('axis_values', Int32MultiArray, axis_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
