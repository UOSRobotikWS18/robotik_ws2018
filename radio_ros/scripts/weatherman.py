#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import random

WEATHER_INFOS = [
        "It's gonna be cloudy today!",
        "Rain everywhere!",
        "Sunny!"
    ]

if __name__ == '__main__':
    rospy.init_node('weatherman_node')
    pub = rospy.Publisher('weather', String, queue_size=10)

    r = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        weather_info = random.choice(WEATHER_INFOS)
        pub.publish(weather_info)
        r.sleep()