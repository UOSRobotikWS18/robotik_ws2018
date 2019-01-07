#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# define state recovery
class FreeSpace:
    def __init__(self):

        self.__max_vel_x = rospy.get_param('max_vel_x', 0.4)
        self.__max_rotational_vel = rospy.get_param('max_rotational_vel', 0.9)
        self.__x_region = rospy.get_param('x_region', 0.5)
        self.__y_region = rospy.get_param('y_region', 0.5)
        self.__dto_stop_turning = rospy.get_param('dto_stop_turning', 1.0)
        self.__dto_start_turning = rospy.get_param('dto_start_turning', 0.5)

        self.__state_turn = 0
        self.__omega = 0
        self.__turn_omega = 0
        self.__u = 0
        self.__u_turning = 0
        self.__front_max_distance = 0
        self.__twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.Subscriber('scan', LaserScan, self.autonomous_behaviour, queue_size = 1)

    def calc_freespace(self, scan):
        sinsum = 0.0
        cossum = 0.0
        alpha = 0.0
        orientation_weight = 0.0

        for i in range(len(scan.ranges)):
            phi = scan.angle_min + i * scan.angle_increment
            if (phi >= -math.pi / 2.25 and phi <= math.pi / 2.25):
                orientation_weight = math.cos(phi / 1.2) * 1.0 / (1.0 + math.exp(-(scan.ranges[i] - 3.0) / 0.3))
                sinsum += math.sin(phi) * orientation_weight
                cossum += math.cos(phi) * orientation_weight

        if (math.fabs(sinsum) > 0.000001):
            alpha = math.atan2(sinsum, cossum)
        return alpha

    def check_range(self, scan, x_region, y_region, index_to_obstacle, distance_to_obstacle):
        general_distance = 65536.0
        distance_to_obstacle[0] = general_distance
        index_to_obstacle[0] = -1
        self.__front_max_distance = 0
        for i in range(len(scan.ranges)):
            phi = scan.angle_min + i * scan.angle_increment
            if (phi >= -math.pi / 2.25 and phi <= math.pi / 2.25):
                if (scan.ranges[i] < 10.0 and scan.ranges[i] > self.__front_max_distance):
                  self.__front_max_distance = scan.ranges[i]
                x = scan.ranges[i] * math.sin(phi)
                y = scan.ranges[i] * math.cos(phi)
                if ((math.fabs(x) < x_region) and (y < y_region)):
                    if (distance_to_obstacle[0] > y):
                        distance_to_obstacle[0] = y
                        index_to_obstacle[0] = i

                if ((math.fabs(x) < x_region)):
                    if (general_distance > y):
                        general_distance = y
        if (index_to_obstacle[0] == -1):
            distance_to_obstacle[0] = general_distance
        return

    def autonomous_behaviour(self, scan):
        
        max_vel_x = self.__max_vel_x
        max_vel_z = self.__max_rotational_vel
        x_region = self.__x_region
        y_region = self.__y_region
        distance_to_stop_turning = self.__dto_stop_turning
        distance_to_start_turning = self.__dto_start_turning


        index_to_obstacle = [-1]
        distance_to_obstacle = [float("inf")]
        self.__omega = self.calc_freespace(scan)
        self.__u = max_vel_x
        self.check_range(scan, x_region, y_region, index_to_obstacle, distance_to_obstacle)

        if (index_to_obstacle[0] != -1):
            self.__u = distance_to_obstacle[0] / y_region * max_vel_x

        if (self.__state_turn == 1):
            if (distance_to_obstacle[0] > distance_to_stop_turning):
                self.__state_turn = 0
            else:
                self.__omega = self.__turn_omega
                self.__u = min(self.__u, self.__u_turning)

        if ((self.__front_max_distance < distance_to_stop_turning or distance_to_obstacle[0] < distance_to_start_turning) and self.__state_turn == 0):
            if (self.__omega > 0.0):
                self.__omega = -max_vel_z
            if (self.__omega < 0.0):
                self.__omega = max_vel_z
            self.__state_turn = 1
            self.__turn_omega = self.__omega
            self.__u = min(self.__u, self.__u_turning)

        vel = Twist()
        vel.linear.x = self.__u
        vel.angular.z = self.__omega
        self.__twist_publisher.publish(vel)
        return

def node():
    rospy.init_node('freespace_node')

    # ros params
    fs = FreeSpace()
    
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
