#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

pub = rospy.Publisher('/scan_median', LaserScan, queue_size=10)

def laser_callback(scan):

    # get current filter_size from parameter server
    filter_size = rospy.get_param("~filter_size", 5)
    # rospy.loginfo('current filter_size: %d', filter_size)

    medians = []

    # iterate over laserscan
    for i in range(len(scan.ranges)):
        # calculate start_idx of median range
        start_idx = int(i - filter_size / 2.0)
        if start_idx < 0:
            start_idx = 0
        
        # calculate end_idx of median range
        end_idx = int(i + filter_size/2.0)
        if end_idx >= len(scan.ranges):
            end_idx = len(scan.ranges) - 1

        # collect valid range values in a list
        window = []
        for j in range(start_idx, end_idx):
            # first check if current range is in allowed range interval
            if scan.ranges[j] <= scan.range_max and scan.ranges[j] >= scan.range_min:
                window.append(scan.ranges[j])

        median_value = 0.0
        # set median to range max + 1 if window empty
        if len(window) > 0:
            median_value = np.median(window)
        else:

            median_value = scan.range_max + 1

        medians.append(median_value)


    # initial set output scan (median_scan) to scan. 
    median_scan = scan
    median_scan.ranges = medians

    pub.publish(median_scan)

def node():
    rospy.init_node('median_filter_node')
    filter_size = rospy.get_param("~filter_size", 5)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
