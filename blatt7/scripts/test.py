#!/usr/bin/env python

import rospy
import numpy as np
import laser_geometry as lg
import tf
from pyquaternion import Quaternion
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from sensor_msgs import point_cloud2


class Transform():
    def __init__(self):
        self.orientation = Quaternion()
        self.point = np.zeros(3)

    def apply(self, point, rotation):
        self.point += point
        self.orientation *= rotation

    def __str__(self):
        str = "Point: %f %f %f; " %  (self.point[0], self.point[1], self.point[2])
        str += "Rotation: %f %f %f %f\n" %  (self.orientation[1], self.orientation[2], self.orientation[3], self.orientation[0])
        return str

class ICP:
    """
    Class that performs ICP
    """

    def __init__(self, topic, max_dist):
        self.lp = lg.LaserProjection()
        self.lines = self.init_lines_marker("scanner")
        self.line_pub = rospy.Publisher("/line_marker", Marker, queue_size=10)
        self.past_pcl = PointCloud2()
        self.first_scan = True
        self.max_distance = max_dist
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.global_transform = Transform()
        rospy.Subscriber(topic, LaserScan, self.callback, queue_size=1)

    def distance(self, p, q):
        return np.sqrt((q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2)

    def init_lines_marker(self, topic):
        marker = Marker()
        marker.header.frame_id = topic
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

    def draw_lines(self, correspondences):
        self.lines.points = []
        point = Point()
        for corr in correspondences:
            point.x = corr[0].x
            point.y = corr[0].y
            self.lines.points.append(point)
            point.x = corr[1].x
            point.y = corr[1].y
            self.lines.points.append(point)

        self.line_pub.publish(self.lines)

    def to_point_tuple(self, past_p, curr_p):
        return (Point(past_p[0], past_p[1], past_p[2]), Point(curr_p[0], curr_p[1], curr_p[2]))

    def calc_correspondences(self, past_pcl, curr_pcl):
        """
        Calculates correspondences between two point clouds: euclidean distance is minimal for each bla bla

        :return:
        :param past_pcl:
        :param curr_pcl:
        :return: correspondences: Tuple of best match and scan point
        """
        correspondences = []
        past_points = list(point_cloud2.read_points(past_pcl, skip_nans=True, field_names=("x", "y", "z")))
        curr_points = list(point_cloud2.read_points(curr_pcl, skip_nans=True, field_names=("x", "y", "z")))

        for i, past_p in enumerate(past_points):
            distances = [self.distance(past_p, curr_p) for curr_p in curr_points]
            # if i == 130: print distances[130]
            min_corr_index = np.argmin(distances)
            correspondences.append(self.to_point_tuple(curr_points[min_corr_index], past_p))

        return correspondences

    def to_ros_quaternion(self, py_quaternion):
        return np.array([py_quaternion[1], py_quaternion[2], py_quaternion[3], py_quaternion[0]])

    def calc_and_pub_transform(self, correspondences):
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
        tx = c1_x - (c2_x * np.cos(theta) - c2_y * np.sin(theta))
        ty = c1_y - (c2_x * np.sin(theta) + c2_y * np.cos(theta))

        origin = np.array([-tx, -ty, 0.0])
        rotation = Quaternion(axis=[0, 0, 1], angle=-theta)
        self.global_transform.apply(origin, rotation)

        rospy.loginfo("%s", "-- Publishing Transformation --")
        rospy.loginfo("%s", self.global_transform)

    def callback(self, laser_scan):
        if self.first_scan:
            self.past_pcl = self.lp.projectLaser(laser_scan)
            self.first_scan = False
            return

        curr_pcl = self.lp.projectLaser(laser_scan)

        correspondences = self.calc_correspondences(self.past_pcl, curr_pcl)

        self.calc_and_pub_transform(correspondences)

        self.draw_lines(correspondences)

        self.tf_broadcaster.sendTransform(self.global_transform.point,
                                          self.to_ros_quaternion(self.global_transform.orientation),
                                          laser_scan.header.stamp,
                                          "base_footprint",
                                          "odom_combined")

        # set up for next iteration
        self.past_pcl = curr_pcl


def node():
    rospy.init_node('scanmatching_py')

    max_distance = rospy.get_param("~max_distance", 0.28)
    icp = ICP("/scan", max_distance)
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
