#!/usr/bin/env python 

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud, LaserScan
from geometry_msgs.msg import Point32

pub = rospy.Publisher('/cloud', PointCloud, queue_size=10)
filter_size = 5


def laser_callback(scan):
    cloud = PointCloud()
    cloud.header = scan.header

    cloud.header = scan.header;
    for index, point in enumerate(scan.ranges):
        x = point * np.cos(scan.angle_min + index * scan.angle_increment)
        y = point * np.sin(scan.angle_min + index * scan.angle_increment)
        cloud.points.append(Point32(x, y, 0))

    pub.publish(cloud)


def node():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
