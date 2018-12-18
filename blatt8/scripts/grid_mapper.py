#! /usr/bin/env python
import rospy
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid
from laser_geometry import laser_geometry

class GridMapper:
    def __init__(self):

        self.cell_size = 0.02

        self.lp = laser_geometry.LaserProjection()
        self.vis_pub = rospy.Publisher('grid_map', OccupancyGrid, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laserCb, queue_size=1)

    def laserCb(self, scan_msg):
        cloud = self.lp.projectLaser(scan_msg)
        points = self.cloudToNumpy(cloud)

        max_pos = np.max(points, axis=0)
        min_pos = np.min(points, axis=0)

        cloud_dim = max_pos - min_pos

        nr_cells = np.ceil(cloud_dim / self.cell_size) + 1
        nr_cells = nr_cells.astype(int)

        map = np.ones(nr_cells, dtype=np.uint8)*128


    def cloudToNumpy(self, cloud):
        point_generator = point_cloud2.read_points(cloud)

        points = []

        for p_idx,point in enumerate(point_generator):
            if not np.isnan(point[0]) and not np.isnan(point[1]):
                points.append([point[0], point[1]])

        return np.array(points)

    # def getCloudDimensions(self, points):
        

        


def node():
    rospy.init_node('grid_mapper_py')

    gm = GridMapper()
    
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
