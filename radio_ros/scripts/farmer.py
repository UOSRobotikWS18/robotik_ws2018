#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from radio_ros.msg import Music

def weather_callback(msg):
    rospy.loginfo("Weather info received: %s", msg.data)
    
def music_callback(msg):
    print(msg)
    print("")

if __name__ == '__main__':
    rospy.init_node('farmer_node')
    
    rospy.Subscriber('weather', String, weather_callback)
    rospy.Subscriber('music', Music, music_callback)

    rospy.spin()
