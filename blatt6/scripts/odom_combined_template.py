#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('blatt6')
import sys
import rospy
import numpy as np

# import to receive two messages in one callback
from message_filters import ApproximateTimeSynchronizer, Subscriber
# imu and odometry messages to fuse
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# broadcasting tf transformation
import tf

class Kalman1D:
    def __init__(self, process_var, action_var, sensor_var):
        """
            1D Kalman Filter for Rotation Fusion
            Prediction: Odometry Sensor
            Correction: Imu Sensor

            State Space n = 1
            Action Space m = 1
            Measurement Space l = 1
        """

        n = 1
        m = 1
        l = 1

        # define initial state
        self.x = np.array([0.0])
        self.C = np.identity(self.x.shape[0])*process_var

        # TODO: define initial transition model, sensor model 
        
    def predict(self, u):
        """
            Prediction with Odometry Message
            u: odometry angular z
        """
        # TODO: implement prediction step
        return self.x, self.C

    def correct(self, z):
        """
            Correction with Imu Message
            z: imu angular z
        """
        # TODO: implement correction step
        return self.x,self.C

    
class OdomCombined:
    def __init__(self):

        # global vars
        self.br = tf.TransformBroadcaster()
        self.source_frame = "base_footprint"
        self.target_frame = "odom_combined"

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.rot_z = 0.0

        self.stamp = rospy.Time.now()

        # TODO: 1) Define member of Kalman1D
        #       2) Use Approximate Time Synchronizer for /odom and /imu -> callback: self.imu_odom_callback
        #           - https://answers.ros.org/question/81126/how-to-use-approximatetime-in-python/
        #       

    def imu_odom_callback(self, imu, odom):

        # velocity x
        twist_linear_x = odom.twist.twist.linear.x
        
        # angular velocity of odom around z axis
        twist_angular_z = odom.twist.twist.angular.z

        # angular velocity of imu around z axis
        imu_angular_z = imu.angular_velocity.z


        # TODO: 1) use Kalman-Filter to fuse angular z from odom and imu
        #       2) update global pose pos_x, pos_y, rot_z


        # broadcast
        self.broadcastTransformation()

    def broadcastTransformation(self):
        """
            Broadcast current transformation with tf
        """
        self.br.sendTransform((self.pos_x, self.pos_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.rot_z),
                     self.stamp,
                     self.source_frame,
                     self.target_frame)


def main(args):
    rospy.init_node('odom_combined')
    odom = OdomCombined()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)