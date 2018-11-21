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
        gray2 = self.gauss(gray, kernel_size=17, sigma=0.1)

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

    def gauss(self, img, kernel_size=5, sigma=0.1):
        kernel = self.generate_gauss_kernel(kernel_size)
        result = cv2.filter2D(img,-1,kernel)

        return result


    def generate_gauss_kernel(self, size, sigma=0.1):

        kernel = np.zeros((size,size))
        center = int( (size-1) / 2)

        for i in range(size):
            for j in range(size):
                x = i-center
                y = j-center
                value = self.gauss_func(x,y,sigma)
                kernel[i,j] = value

        # normalize kernel
        kernel /= np.sum(kernel)
        return kernel
        
                
    def gauss_func(self, x, y, sigma=1.0):
        # https://de.wikipedia.org/wiki/Gau%C3%9F-Filter
        return 1.0 / (2.0 * np.pi * sigma**2.0) * np.exp(- (x**2.0 + y**2.0) / 2*sigma**2.0 )



def main(args):
    rospy.init_node('image_filter_node')
    i_filter = image_filter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)