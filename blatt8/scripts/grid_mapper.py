#! /usr/bin/env python
import rospy
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
from nav_msgs.msg import OccupancyGrid
from laser_geometry import laser_geometry
from tf import TransformListener
import tf.transformations
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import pcl_ros
import ros_numpy

class GridMapper:
    def __init__(self):
        self.cell_size = 0.05
        self.tf = TransformListener()
        self.lp = laser_geometry.LaserProjection()
        self.vis_pub = rospy.Publisher('grid_map', OccupancyGrid, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laserCb, queue_size=1)
        self.cloudPub = rospy.Publisher("cloud", PointCloud2, queue_size=1)

    def laserCb(self, scan_msg):
        cloud = self.lp.projectLaser(scan_msg)

        xyz_array = self.pointcloud_to_numpy_array(cloud)
        
        src_frame = scan_msg.header.frame_id
        target_frame = 'odom_combined'

        try:
            pos, quat = self.getCurrentTransformation(target_frame, src_frame)
        except Exception as a:
            print(a)
            return
        
        xyz_array_transformed = self.transformPoints(xyz_array, pos, quat)
        
        
        # test
        self.publish_xyz_array(xyz_array_transformed, frame_id=target_frame)
        
        xy_array_transformed = xyz_array_transformed[:,:2]
        
        max_pos = np.max(xy_array_transformed, axis=0)
        min_pos = np.min(xy_array_transformed, axis=0)

        cloud_dim = max_pos - min_pos

        nr_cells = np.ceil(cloud_dim / self.cell_size) + 1
        nr_cells = nr_cells.astype(int)

        nr_cells_2 = nr_cells.copy()
        nr_cells_2[0] = nr_cells[1]
        nr_cells_2[1] = nr_cells[0]
        # print(nr_cells)

        grid_map = np.ones(nr_cells_2, dtype=np.int8) * -1

        grid_map = self.fill_grid(grid_map, xy_array_transformed, min_pos, self.cell_size)

        # grid_map = self.fill_grid(grid_map, xy_array_transformed)

        oc_grid = ros_numpy.occupancy_grid.numpy_to_occupancy_grid(grid_map)

        oc_grid.header.frame_id = target_frame
        oc_grid.header.stamp = scan_msg.header.stamp
        oc_grid.info.width = nr_cells_2[1]
        oc_grid.info.height = nr_cells_2[0]
        oc_grid.info.resolution = self.cell_size
        oc_grid.info.origin.position.x = min_pos[0]
        oc_grid.info.origin.position.y = min_pos[1]
        
        self.vis_pub.publish(oc_grid)

    def pointcloud_to_numpy_array(self, pointcloud):
        pointcloud2_array = ros_numpy.numpify(pointcloud)
        return ros_numpy.point_cloud2.get_xyz_points(pointcloud2_array)

    def fill_grid(self, grid_map, xy_array_transformed, grid_min_point, grid_res):
        grid_coords = self.points_to_grid(xy_array_transformed, grid_min_point, grid_res)
        for i in range(grid_coords.shape[0]):
            coord = grid_coords[i,:]
            grid_map[coord[1], coord[0]] = 100
        return grid_map

    def publish_xyz_array(self, xyz_array, frame_id):
        test_cloud = self.xyz_array_to_pointcloud2(xyz_array, frame_id=frame_id, stamp=rospy.Time.now())
        self.cloudPub.publish(test_cloud)

    def xyz_array_to_pointcloud2(self, points, stamp=None, frame_id=None):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        '''
        msg = PointCloud2()
        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()

        return msg

    def getCurrentTransformation(self, src_frame, target_frame, stamp=None):
        """
            Get Current Transformation
        """
        t = stamp
        if t is None:
            t = self.tf.getLatestCommonTime(src_frame, target_frame)
            
        position, quaternion = self.tf.lookupTransform(src_frame, target_frame, t)
        return position, quaternion

    def transformPoints(self, np_points, trans, quaternion):
        """
            Transforms points. np_points.shape == (N,3)
        """
        print(np_points.shape)
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

    def point_to_grid(self, point, grid_min_point, grid_res):
        grid_point = point - grid_min_point
        grid_coord = grid_point / grid_res
        return grid_coord.astype(int)

    def points_to_grid(self, xy_points, grid_min_point, grid_res):
        grid_points = xy_points - grid_min_point
        grid_coords = grid_points / grid_res
        return grid_coords.astype(int)
        


def node():
    rospy.init_node('grid_mapper_py')

    gm = GridMapper()
    
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
