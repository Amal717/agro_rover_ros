#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class FrameImageNode:
    def __init__(self):
        rospy.init_node('frame_image', anonymous=True)
        self.image_sub = rospy.Subscriber('webcam/image', Image, self.image_callback)
        self.image_pub = rospy.Publisher('frame/image', Image, queue_size=1)
        self.bridge = CvBridge()
        self.last_published_time = rospy.get_rostime()

    def image_callback(self, data):
        # Check if it's time to publish a new image
        if rospy.get_rostime() - self.last_published_time >= rospy.Duration.from_sec(5):
            try:
                # Convert the ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

                # Resize the image to 256x256 pixels
                resized_image = cv2.resize(cv_image, (256, 256))

                # Convert the resized image back to ROS Image message
                resized_image_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")

                # Publish the resized image
                self.image_pub.publish(resized_image_msg)

                # Update the last published time
                self.last_published_time = rospy.get_rostime()
                rospy.loginfo("Published a new image (256x256) to frame/image topic.")
            except CvBridgeError as e:
                rospy.logerr(e)

if __name__ == '__main__':
    try:
        node = FrameImageNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

