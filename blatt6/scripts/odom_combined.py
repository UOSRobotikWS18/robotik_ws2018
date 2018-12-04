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
    def __init__(self, process_var=1000.0, action_var=1.1, sensor_var=0.04):
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

        # define initial transition model
        self.A = np.identity(n)
        self.B = np.zeros( (n,m) )
        self.B[0,0] = 1.0
        self.B_C = np.identity(n) * action_var

        # define initial sensor model
        self.H = np.zeros( (l,n) )
        self.H[0,0] = 1
        self.C_H = np.identity(l) * sensor_var
        
    def predict(self, u):
        """
            Prediction with Odometry Message
            u: odometry angular z
        """
        # get old state
        x = self.x
        # get old covariance
        C = self.C

        # get transition model
        A = self.A
        B = self.B
        B_C = self.B_C

        x_pred = A.dot(x) + np.matmul(B, u)
        C_pred = A.dot(C).dot(A.transpose()) + B_C

        self.x = x_pred
        self.C = C_pred
        return x_pred, C_pred

    def correct(self, z):
        """
            Correction with Imu Message
            z: imu angular z
        """
        # get prediction
        x_pred = self.x
        C_pred = self.C

        # get sensor model
        H = self.H
        C_H = self.C_H

        # calculated Kalman-Gain
        K = np.matmul(C_pred.dot(H.transpose() ), np.linalg.inv( H.dot(C_pred).dot(H.transpose()) + C_H ) )
        
        # correct state
        x_correction = K.dot(z - H.dot(x_pred) )
        x = x_pred + x_correction

        # correct Covariance 
        Gained_tmp = K.dot(H)
        C = (np.identity(Gained_tmp.shape[0]) - Gained_tmp).dot(C_pred)

        self.x = x
        self.C = C
        return x,C

    
class OdomCombined:
    def __init__(self):

        # global vars
        self.br = tf.TransformBroadcaster()
        self.source_frame = "base_footprint"
        self.target_frame = "odom_combined"

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.rot_z = 0.0

        self.kalman = Kalman1D(1000.0, 1.1**2.0, 0.04**2.0)
        self.stamp = rospy.Time.now()

        # Subscriber 
        imu_sub = Subscriber("/imu/data", Imu)
        odom_sub = Subscriber("/odom", Odometry)

        ats = ApproximateTimeSynchronizer([imu_sub, odom_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.imu_odom_callback)

        

    def imu_odom_callback(self, imu, odom):
        # print('imu frame:')
        # print(imu.header.frame_id)
        # print('odom frame:')
        # print(odom.header.frame_id)

        # velocity x
        twist_linear_x = odom.twist.twist.linear.x
        
        # angular velocity of odom around z axis
        twist_angular_z = odom.twist.twist.angular.z

        # angular velocity of imu around z axis
        imu_angular_z = imu.angular_velocity.z

        # define action u
        u = np.array([twist_angular_z])
        # define measurement z
        z = np.array([imu_angular_z])

        self.kalman.predict(u)
        x,C = self.kalman.correct(z)
        angular_z = x[0]
        
        # update global pose

        current_stamp = rospy.Time.now()
        dT = (current_stamp - self.stamp).to_sec()
        
        # rotation: -pi < x <= pi 
        self.rot_z += angular_z * dT
        if self.rot_z > np.pi:
            self.rot_z -= 2*np.pi
        elif self.rot_z <= -np.pi:
            self.rot_z += 2*np.pi
        # translation with new angle
        self.pos_x += np.cos(self.rot_z) * twist_linear_x * dT
        self.pos_y += np.sin(self.rot_z) * twist_linear_x * dT

        # print('Pose: (%f, %f, %f)'% (self.pos_x, self.pos_y, self.rot_z) )

        # update time
        self.stamp = current_stamp

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