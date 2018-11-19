#! /usr/bin/env python
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# scan to cloud 
from laser_geometry import laser_geometry

# other includes
import numpy as np 


# global vars
# laser projector
lp = None

# hough image publisher
hough_image_pub = None
# hough line publisher
hough_line_pub = None

def generate_marker(header):
    lines = Marker()
    lines.header = header
    lines.ns = "hough"
    lines.id = 0
    lines.action = lines.ADD
    lines.type = lines.LINE_LIST
    lines.pose.orientation.w = 1.0
    lines.scale.x = 0.1
    lines.color.g = 1.0
    lines.color.a = 1.0
    return lines


def array_to_image(array, max_val):

    # define output image
    hough_image = Image()
    
    hough_image.width = array.shape[1]
    hough_image.height = hough_image.step = array.shape[0]
    hough_image.encoding = 'mono8'

    # convert array data to uint8
    array = array.astype(np.float64)

    max_val = float(max_val)
    if max_val > 0:
        array /= max_val

    array = 255.0 * array
    array = array.astype(np.uint8)
    # set data to ros image
    hough_image.data = array.flatten().tolist()
    return hough_image


def laser_callback(scan):
    global lp, hough_image_pub

    buckets_phi = rospy.get_param("~buckets_phi", 360)
    buckets_range = rospy.get_param("~buckets_range", 300)

    buckets_thresh = rospy.get_param("~num_points_for_line", 30)

    

    # convert LaserScan to PointCloud2
    cloud = lp.projectLaser(scan)
    
    # convert it to a generator of the individual points
    point_generator = point_cloud2.read_points(cloud)

    # define discrete hough space

    hough_space = np.zeros([buckets_range, buckets_phi], dtype=int)

    # python generate 3d array: x -> range, y -> phi, z -> points
    hough_spaces_points = [ [[] for _ in range(buckets_phi)] for _ in range(buckets_range)]
    
    # a) compute hough space

    max_val = 0
    max_range_idx = 0
    max_phi_idx = 0

    for p_idx,point in enumerate(point_generator):
        if not np.isnan(point[0]) and not np.isnan(point[1]):
            for phi_idx in range(buckets_phi):
                phi = (phi_idx - (buckets_phi/2.0)) * np.pi/float(buckets_phi)
                rang = np.abs(point[0] * np.cos(phi) + point[1] * np.sin(phi))
                range_idx = int( (buckets_range-1)*rang/scan.range_max)

                # increment hough space entry
                hough_space[range_idx, phi_idx] += 1

                # update maximum variables
                if hough_space[range_idx, phi_idx] > max_val:
                    max_val = hough_space[range_idx, phi_idx]
                    max_range_idx = range_idx
                    max_phi_idx = phi_idx

                # update points
                if len(hough_spaces_points[range_idx][phi_idx]) > 1:
                    hough_spaces_points[range_idx][phi_idx][1].x = point[0]
                    hough_spaces_points[range_idx][phi_idx][1].y = point[1]
                else:
                    gp = Point()
                    gp.x = point[0]
                    gp.y = point[1]
                    hough_spaces_points[range_idx][phi_idx].append(gp)

    # SHOW LINE
    line = generate_marker(scan.header)

    # find the lines
    
    # Excercise. How to find all the best lines ?
    # hough_indices = np.argwhere(hough_space > buckets_thresh)
    # for hough_idx in hough_indices:
    #     range_idx = hough_idx[0]
    #     phi_idx = hough_idx[1]
    #     line.points.append(hough_spaces_points[range_idx][phi_idx][0] )
    #     line.points.append(hough_spaces_points[range_idx][phi_idx][1] )
    # problem: threshold leads to multiple lines at one peak

    # Use only the highest voted line in this solution 
    line.points.append(hough_spaces_points[max_range_idx][max_phi_idx][0])
    line.points.append(hough_spaces_points[max_range_idx][max_phi_idx][1])

    hough_line_pub.publish(line)
    
    # c) Generate Image from hough space

    # convert to uint8
    
    hough_image = array_to_image(hough_space, max_val)
    hough_image.header = scan.header
    
    hough_image_pub.publish(hough_image)


def node():
    rospy.init_node('hough_node')

    # ros params
    if not rospy.has_param("~buckets_phi"):
        rospy.set_param("~buckets_phi", 360)
    if not rospy.has_param("~buckets_range"):
        rospy.set_param("~buckets_range", 300)

    global lp, hough_image_pub, hough_line_pub
    lp = laser_geometry.LaserProjection()

    # Publisher
    hough_image_pub = rospy.Publisher('/houghspace', Image, queue_size=10)
    hough_line_pub = rospy.Publisher('/houghlines', Marker, queue_size=1)

    # Subscriber
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass

