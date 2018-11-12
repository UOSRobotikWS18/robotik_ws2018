#!/usr/bin/env python

import rospy
import numpy as np
from collections import deque
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

imu_values = deque()
filter_size = 0


def callback(data):
    global imu_values, filter_size
    if len(imu_values) >= filter_size:
        _ = imu_values.popleft()

        roll_mean, pitch_mean, yaw_mean = np.mean(imu_values, 0)
        roll_std, pitch_std, yaw_std = np.std(imu_values, 0)
        rospy.loginfo("Got IMU-Data (mean): (%f, %f, %f)", roll_mean, pitch_mean, yaw_mean)
        rospy.loginfo("Got IMU-Data (std) : (%f, %f, %f)\n", roll_std, pitch_std, yaw_std)

    quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    imu_values.append([roll, pitch, yaw])


def node():
    global filter_size
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("/imu_corrected", Imu, callback)
    filter_size = rospy.get_param("~filter_size", 20)
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
