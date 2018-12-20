#!/usr/bin/env python

import rospy
import sys
from tf.transformations import quaternion_from_euler
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point32, Point, Vector3, Transform, Quaternion
from visualization_msgs.msg import Marker
from message_filters import ApproximateTimeSynchronizer, Subscriber
import laser_geometry.laser_geometry as lg
import tf2_geometry_msgs

class ICP:
    def __init__(self):
        """
        ICP algorithm implementation

        :param scan_topic: scan topic to subscribe to
        :param model_topic: model topic to subscribe to
        """
        self.global_transform = (Vector3, Quaternion)
        self.lp = lg.LaserProjection()
        self.marker = self.__init_marker()

        self.pre_trans_pub = rospy.Publisher('/pre_trans', PointCloud2, queue_size=10)
        self.pose_trans_pub = rospy.Publisher('/pre_trans', PointCloud2, queue_size=10)
        self.marker_pub = rospy.Publisher('/corr_markers', Marker, queue_size=10)

    def __init_marker(self):
        marker = Marker()
        marker.header.frame_id = "/model"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientation
        marker.pose.orientation.w = 1.0

        # marker position

        # marker line points
        marker.points = []
        return marker

    def __draw_lines(self, correspondences):
        self.marker.points = []
        point = Point()
        for corr in correspondences:
            point.x = corr[0].x
            point.y = corr[0].y
            self.marker.points.append(point)
            point.x = corr[1].x
            point.y = corr[1].y
            self.marker.points.append(point)


    def __calc_error_min_term(self, corr1, corr2, c1, c2):
        return (corr1 - c1) * (corr2 - c2)

    def __calc_transform(self, correspondences):
        # compute centroids
        c1_x = np.sum([corr[0].x for corr in correspondences]) / len(correspondences)
        c1_y = np.sum([corr[0].y for corr in correspondences]) / len(correspondences)
        c2_x = np.sum([corr[1].x for corr in correspondences]) / len(correspondences)
        c2_y = np.sum([corr[1].y for corr in correspondences]) / len(correspondences)

        # compute the other terms needed for error minimization (see slide 234)
        s_xx, s_xy, s_yx, s_yy = 0.0, 0.0, 0.0, 0.0
        for corr in correspondences:
            s_xx += (corr[0].x - c1_x) * (corr[1].x - c2_x)
            s_xy += (corr[0].x - c1_x) * (corr[1].y - c2_y)
            s_yx += (corr[0].y - c1_y) * (corr[1].x - c2_x)
            s_yy += (corr[0].y - c1_y) * (corr[1].y - c2_y)

        theta = np.arctan2(s_yx - s_xy, s_xx + s_yy)
        tx = c1_x - (c2_x * np.cos(theta) - c2_y * np.sin(theta));
        ty = c1_y - (c2_x * np.sin(theta) + c2_y * np.cos(theta));

        # create and return origin and rotation between the two scans
        rotation = quaternion_from_euler(0.0, 0.0, theta)
        origin = Vector3(tx, ty, 0.0)
        return origin, rotation

    def __transform_pcl(self, in_cloud, out_cloud, global_transform):
        point32 = Point32
        cloud_points = in_cloud.points

        for point in cloud_points:
            v = Vector3(point[0], point[1], point[2])
            # vt = tf2_geometry_msgs.do_transform_vector3(v, global_transform)
            # out_cloud.append(Point32(vt.x, vt.y, vt.z))
            rospy.loginfo("%s", global_transform)

    def laser_callback(self, scan, model):
        temp_cloud = self.lp.projectLaser(scan)
        model_cloud = self.lp.projectLaser(model)
        scan_cloud = PointCloud2

        correspondences = []
        self.pre_trans_pub.publish(temp_cloud)

        self.__transform_pcl(temp_cloud, scan_cloud, self.global_transform)

        self.__draw_lines(correspondences)

        rospy.loginfo("Published transform: translation %f %f %f, rotation %f %f %f %f", 0, 0, 0, 0, 0, 0, 0)

def main(args):
    rospy.init_node('scanmatching_py')
    icp = ICP()
    try:
        scan_sub = Subscriber("/scan", LaserScan)
        model_sub = Subscriber("/model", LaserScan)
        ats = ApproximateTimeSynchronizer([scan_sub, model_sub], queue_size=5, slop=0.1)
        ats.registerCallback(icp.laser_callback)

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)