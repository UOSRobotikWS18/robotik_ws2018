import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf

cloud_pub = rospy.Publisher('/transformed_cloud', PointCloud, queue_size=10)
tf_listener = tf.TransformListener()


def laser_callback(scan1, scan2):
    cloud1, cloud2, transformed_cloud = PointCloud(), PointCloud(), PointCloud()

    cloud1.header = scan1.header;
    for index, point in enumerate(scan1.ranges):
        x = point * np.cos(scan1.angle_min + index * scan1.angle_increment)
        y = point * np.sin(scan1.angle_min + index * scan1.angle_increment)
        cloud1.points.append(Point32(x, y, 0))

    cloud2.header = scan2.header;
    for index, point in enumerate(scan2.ranges):
        x = point * np.cos(scan2.angle_min + index * scan2.angle_increment)
        y = point * np.sin(scan2.angle_min + index * scan2.angle_increment)
        cloud2.points.append(Point32(x, y, 0))

    try:
        transformed_cloud = tf_listener.transformPointCloud("base_laser_link", cloud2)
    except (tf.Exception):
        rospy.logerr("Transform into %s failed", "base_laser_link")
        return

    for point in transformed_cloud.points:
        cloud1.points.append(point)

    cloud_pub.publish(cloud1)

def node():
    rospy.init_node("scan_merger", anonymous=True)

    scan_sub_1 = Subscriber("/laserscan", LaserScan)
    scan_sub_2 = Subscriber("/scan", LaserScan)

    ats = ApproximateTimeSynchronizer([scan_sub_1, scan_sub_2], queue_size=5, slop=0.1)
    ats.registerCallback(laser_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass
