#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import laser_geometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ScanToCloudConverter:
    def __init__(self, input_topic, output_topic):

        self.lp = laser_geometry.LaserProjection()

        rospy.Subscriber(input_topic, LaserScan, self.__scan_callback, queue_size=1)
        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=1)

    def __scan_callback(self, scan):
        cloud = self.lp.projectLaser(scan)
        self.pub.publish(cloud)

class LineFinder:
    def __init__(self, max_distance, min_line_points, line_end_epsilon):
        rospy.Subscriber("/cloud", PointCloud2, self.__cloud_callback)
        self.pub = rospy.Publisher('/lines', Marker, queue_size=10)

        self.min_line_points = min_line_points
        self.max_distance = max_distance
        self.line_end_epsilon = line_end_epsilon

        rospy.loginfo("%s", "Starting Line Finder")
        

    def __del__(self):
        rospy.loginfo("%s", "Shutting down Line Finder")

    @staticmethod
    def distance(p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

    def __valid_sum(self, points, start, end):
        if start == end:
            return True

        direct = LineFinder.distance(points[start], points[end])
        total = np.sum([LineFinder.distance(points[i], points[i+1]) for i in range(start, end)])

        return (direct / total) >= 1 - self.line_end_epsilon / len(points[start:end])

    def __valid_end_of_line(self, points, step):
        if step == 0:
            return True
        if step + 1 >= len(points):
            return True

        direct = LineFinder.distance(points[step - 1], points[step + 1])
        individual = LineFinder.distance(points[step - 1], points[step]) + \
                     LineFinder.distance(points[step], points[step + 1])

        if individual is 0 or direct is 0:
            return False

        return (direct / individual) >= 1 - self.line_end_epsilon

    def __valid_direct_dist(self, points, step):
        return LineFinder.distance(points[step], points[step + 1]) < self.max_distance

    def __cloud_callback(self, cloud):
        marker = Marker()
        marker.header = cloud.header
        marker.id = 0
        marker.ns = 'lines'
        marker.action = marker.ADD
        marker.type = marker.LINE_STRIP
        marker.color.r, marker.color.a = 1, 1
        marker.scale.x = .05

        start, end = 0, 0

        point_generator = point_cloud2.read_points(cloud)

        points = [Point(p[0],p[1],p[2]) for p in point_generator]

        marker_id = 0

        for i in range(len(points)-1):
            if self.__valid_end_of_line(points, end) and \
                    self.__valid_sum(points, start, end) and \
                    self.__valid_direct_dist(points, end):
                end += 1
            else:
                start_point = points[start]
                end_point = points[end]

                if start_point is not end_point and len(points[start:end]) > self.min_line_points:
                    marker.points.append(start_point)
                    marker.points.append(end_point)
                start = end
                end = start + 1

        self.pub.publish(marker)

        for i,point in enumerate(marker.points):
            self.pub.publish(self.__gen_point_marker(cloud.header, point, i))

    def __gen_point_marker(self, header, point, id):
        marker = Marker()
        marker.header = header
        marker.id = id
        marker.ns = 'points'
        marker.action = marker.ADD
        marker.type = marker.SPHERE
        marker.color.g, marker.color.a = 1, 1
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .1
        marker.pose.position = point
        return marker


def node():
    rospy.init_node('online_line_finder')

    max_distance = rospy.get_param("~max_distance", 0.28)
    min_line_points = rospy.get_param("~min_line_points", 5)
    line_end_epsilon = rospy.get_param("~line_end_epsilon", 0.1)
    rospy.loginfo("Set param '~min_line_points': %d", min_line_points)
    rospy.loginfo("Set param '~max_distance': %f", max_distance)
    rospy.loginfo("Set param '~line_end_epsilon': %f", line_end_epsilon)

    # uncomment if topic with type PointCloud2 already exist
    scan_to_cloud = ScanToCloudConverter('scan', 'cloud')

    lf = LineFinder(max_distance, min_line_points, line_end_epsilon)

    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
