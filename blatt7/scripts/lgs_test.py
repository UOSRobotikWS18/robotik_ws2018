#!/usr/bin/env python

import rospy
import numpy as np
import laser_geometry as lg
import tf
import ros_numpy
import tf.transformations
import time
# from pyquaternion import Quaternion
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

class ICP:
    """
    Class that performs ICP
    """

    def __init__(self, laser_frame='scanner', base_frame='base_footprint', corr_window_size=5):

        self.laser_frame = 'scanner'
        self.base_frame = 'base_footprint'
        self.corr_window_size = corr_window_size
        self.tf = tf.TransformListener()

        t = None
        while t is None:
            try:
                t = self.tf.getLatestCommonTime(self.laser_frame, self.base_frame)
            except:
                pass
            
        print('found transformation')

        # defining initial state
        self.pos = np.array([0.0,0.0,0.0])
        self.rot = tf.transformations.quaternion_about_axis(0.0, (0,0,1))

        self.trans_foot, self.rot_foot = self.tf.lookupTransform(self.laser_frame, self.base_frame, t)

        self.lp = lg.LaserProjection()
        self.past_pcl = None
        self.first_scan = True
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback, queue_size=1)

    def laser_callback(self, scan_msg):
        start = time.time()

        if self.first_scan:
            self.past_pcl = self.laser_to_2dcloud(scan_msg)
            self.first_scan = False
            return

        curr_pcl = self.laser_to_2dcloud(scan_msg)

        # find correspondences
        correspondences = self.find_correspondences(curr_pcl, self.past_pcl)

        # find transformation
        trans, theta = self.find_transformation(curr_pcl, self.past_pcl, correspondences)
        
        # boradcast transformation

        delta_theta = theta
        delta_trans = np.zeros(3)
        delta_trans[:2] = trans[:,0]

        delta_rot = tf.transformations.quaternion_about_axis(delta_theta, (0,0,1))

        self.rot = tf.transformations.quaternion_multiply(self.rot, delta_rot)
        self.pos += delta_trans

        print(self.rot)
        print(self.pos)

        self.tf_broadcaster.sendTransform(self.pos,
                                          self.rot,
                                          scan_msg.header.stamp,
                                          self.base_frame,
                                          "odom_combined")

        self.past_pcl = curr_pcl
        elapsed_time = time.time() - start
        print('elapsed calc time: %f' % elapsed_time)

    def find_correspondences(self, past_pcl, curr_pcl):
        # TODO: correspondences 1D

        N_curr = curr_pcl.shape[0]
        N_past = past_pcl.shape[0]

        N_smaller = min(N_curr, N_past)

        N = 2
        correspondences = np.zeros((2,N_smaller-(2*N)+1), dtype=int)

        iteration = 0
        for i in range(N, N_smaller - N+1):
            past_point = past_pcl[i]
            corr_slice = curr_pcl[i-N:i+N]
            distances = [distance(past_point, curr_p) for curr_p in corr_slice]
            corr_index = np.argmin(distances)+iteration
            correspondences[0,iteration] = corr_index
            iteration += 1

        correspondences[1,:] = correspondences[0,:]
        return correspondences
                    
    def find_transformation(self, past_pcl, curr_pcl, correspondences):
        N = correspondences.shape[1]

        A = np.zeros((N*2, 6))

        # print "trans:", trans, "rot", theta[0]
        b = np.zeros((N*2, 1))

        # fill A
        X = np.ones( (N,3) )
        X[:,:2] = curr_pcl[correspondences[1,:]]
        A[:N,:3] = X
        A[N:,3:] = X

        # fill b

        b[:N,0] = past_pcl[correspondences[0,:],0]
        b[N:,0] = past_pcl[correspondences[0,:],1]

        # linear least squares

        # x = (A^t*A)^(-1)*A^t*b 

        B = np.matmul(A.transpose(), A)
        c = np.matmul(A.transpose(), b)
    
        x = np.matmul(np.linalg.inv(B), c)

        trans = np.array([x[2], x[5]])

        # arcsin kann glaube ich oft fehlschlagen...die maximalen werte sind da -1,..,1,
        # aber durch rundungsfehler kann da mal 1.00000023 stehen -> error
        # zB np.round(-x[1], 4)
        theta = np.arcsin(-x[1]) 

        return trans, theta[0]
    
    def distance(self, p, q):
        """
        Euclidean distance for point in 2d space
        """
        return np.sqrt((q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2)

    def laser_to_2dcloud(self, scan_msg):
        cloud = self.lp.projectLaser(scan_msg)
        xyz_array = self.pointcloud_to_numpy_array(cloud)
        xyz_array_transformed = self.transform_points(xyz_array, self.trans_foot, self.rot_foot)
        return xyz_array_transformed[:,:2]

    def pointcloud_to_numpy_array(self, pointcloud):
        """
            Converting Pointcloud2 to Numpy Array (3,N)
        """
        pointcloud2_array = ros_numpy.numpify(pointcloud)
        return ros_numpy.point_cloud2.get_xyz_points(pointcloud2_array)

    def numpy_array_to_pointcloud(self, numpy_array, frame_id, stamp=None):
        """
            Converting Numpy Array (3,N) to Pointcloud2
        """
        if stamp is None:
            stamp = rospy.Time.now()
        out_cloud = self.xyz_array_to_pointcloud2(numpy_array, frame_id=frame_id, stamp=stamp)
        return out_cloud

    def transform_points(self, np_points, trans, quaternion):
        """
            Transforms points. np_points.shape == (N,3)
        """
        pos = np.zeros((3,1))
        pos[0] = trans[0]
        pos[1] = trans[1]
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]

        R = np.array([[np.cos(theta), -np.sin(theta), 0.0 ], \
                      [np.sin(theta),  np.cos(theta), 0.0 ], \
                      [          0.0,            0.0, 1.0 ] ])

        # p' = R * p + t
        np_points = R.dot(np_points.transpose())
        np_points = np.add(np_points, pos).transpose()
        return np_points

    
def node():
    rospy.init_node('scanmatching_lgs_py')

    icp = ICP()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass