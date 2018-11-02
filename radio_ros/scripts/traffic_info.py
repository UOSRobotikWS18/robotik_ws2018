#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import random

TRAFFIC_INFOS = [
        "Traffic jam on route 68!",
        "Attention! Ghost driver on highway to hell.",
        "Slow moving traffic in the sesame street."
    ]


if __name__ == '__main__':
    rospy.init_node('traffic_info_node')
    pub = rospy.Publisher('traffic', String, queue_size=10)

    r = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        traffic_info = random.choice(TRAFFIC_INFOS)
        pub.publish(traffic_info)
        r.sleep()