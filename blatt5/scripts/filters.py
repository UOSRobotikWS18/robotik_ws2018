#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('blatt5')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_filter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_shift", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_rect_color", Image, self.imageCallback, queue_size=1)

    def imageCallback(self, img):
        # use CvBridge to convert messages: sensor_msgs::Image <-> cv::Mat
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # filter image
        gray2 = self.shift(gray)

        # convert back to bgr
        cv_image = cv2.cvtColor(gray2, cv2.COLOR_GRAY2BGR)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def shift(self, img):
        result = np.zeros(img.shape, dtype=img.dtype)

        # for loop style
        for i in range(0,img.shape[0]):
            for j in range(30,img.shape[1]):
                result[i,j] = img[i,j-30]

        # numpy style:
        # result[:,30:] = img[:,:-30]

        return result

def main(args):
    rospy.init_node('image_filter_node')
    i_filter = image_filter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)