#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def webcam_publisher():
    rospy.init_node('webcam_publisher', anonymous=True)
    image_pub = rospy.Publisher('webcam/image', Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)  # 0 for the default webcam

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_pub.publish(image_message)
            rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass

