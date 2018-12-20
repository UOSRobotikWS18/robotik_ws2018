#! /usr/bin/env python
import rospy
import numpy as np

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
from nav_msgs.msg import OccupancyGrid
from laser_geometry import laser_geometry
from tf import TransformListener
import tf.transformations
import ros_numpy


class GridMapper:
    def __init__(self, cell_size=0.05, grid_size = [2000,2000] ):
        """
            1) Initialize a fixed sized map
            2) Convert Laserscan to Pointcloud
            3) Transform Pointcloud to odom_combined frame
            4) Project Pointcloud into map
            5) Publish Map
        """
        self.cell_size = cell_size
        self.grid_size = np.array(grid_size)

        self.initGrid()

        self.tf = TransformListener()
        self.lp = laser_geometry.LaserProjection()
        self.vis_pub = rospy.Publisher('grid_map', OccupancyGrid, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laserCb, queue_size=1)
        self.cloudPub = rospy.Publisher("cloud", PointCloud2, queue_size=1)
        self.current_time = rospy.Time.now()

    def initGrid(self):
        self.min_pos = -self.grid_size/2 * self.cell_size
        self.grid_map = np.ones(self.grid_size, dtype=np.int8) * -1

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
        
        # test correct transformation
        # out_cloud = self.numpy_array_to_pointcloud(xyz_array_transformed, frame_id=target_frame)
        # self.cloudPub.publish(out_cloud)
        
        xy_array_transformed = xyz_array_transformed[:,:2]

        self.grid_map = self.fill_grid(self.grid_map, xy_array_transformed, self.min_pos, self.cell_size, pos)

        oc_grid = ros_numpy.occupancy_grid.numpy_to_occupancy_grid(self.grid_map)

        oc_grid.header.frame_id = target_frame
        oc_grid.header.stamp = scan_msg.header.stamp
        oc_grid.info.width = self.grid_size[0]
        oc_grid.info.height = self.grid_size[1]
        oc_grid.info.resolution = self.cell_size
        oc_grid.info.origin.position.x = self.min_pos[0]
        oc_grid.info.origin.position.y = self.min_pos[1]
        
        self.vis_pub.publish(oc_grid)

        # save map if time exceeded
        if rospy.Time.now() - self.current_time > rospy.Duration(10.0):
            
            self.save_map('map.pgm')
            self.current_time = rospy.Time.now()
            print('map saved')

    def save_map(self, filename):

        x_dim = self.grid_size[0]
        y_dim = self.grid_size[1]
        pixels = self.grid_map

        fout=open(filename, 'wb')
        pgmHeader = 'P2' + '\n' + str(y_dim) + ' ' + str(x_dim) + '\n' + str(255) +  '\n'
        fout.write(pgmHeader)

        for i in range(x_dim):
            line = ""
            for j in range(y_dim):
                
                pixel = pixels[(x_dim - 1) - i, (y_dim - 1) - j]
                pixel_out = 128
                if pixel == -1:
                    pixel_out = 128
                elif pixel == 100:
                    pixel_out = 0
                else:
                    pixel_out = 255
                line += str(pixel_out) + " "
            line += "\n"
            fout.write(line)

        fout.close()
        
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

    def fill_grid(self, grid_map, xy_array_transformed, grid_min_point, grid_res, my_pos):
        
        my_coord = self.point_to_grid(my_pos, grid_min_point, grid_res)
        grid_coords = self.points_to_grid(xy_array_transformed, grid_min_point, grid_res)
        
        grid_map[my_coord[1], my_coord[0]] = 0
        
        # BresenhamLine
        
        src_x = my_coord[1]
        src_y = my_coord[0]

        for i in range(grid_coords.shape[0]):
            coord = grid_coords[i,:]

            # Draw BresenhamLine from my_coord to coord
            target_x = coord[1]
            target_y = coord[0]

            dx = np.abs( target_x - src_x )
            dy = np.abs( target_y - src_y )
            
            if src_x < target_x:
                sx = 1
            else:
                sx = -1

            if src_y < target_y:
                sy = 1
            else:
                sy = -1

            error = dx - dy
            x = src_x
            y = src_y

            while x != target_x or y != target_y:

                if x < 0 or y < 0:
                    continue

                grid_map[int(x),int(y)] = 0
                e2 = 2 * error
                if e2 > -dy:
                    error = error - dy
                    x += sx
                
                if e2 < dx:
                    error = error + dx
                    y += sy
            
            # draw black line endings
            grid_map[coord[1], coord[0]] = 100

        return grid_map

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
        grid_point = point[:2] - grid_min_point
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
