#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from radio_ros.msg import Music

feedback_pub = None

def traffic_callback(msg):
    rospy.loginfo("Traffic info received: %s", msg.data)
    
def music_callback(msg):
    feedback_pub.publish("I don't like %s" % msg.artist)

if __name__ == '__main__':
    rospy.init_node('trucker_node')
    
    feedback_pub = rospy.Publisher('feedback', String, queue_size=10)
    rospy.Subscriber('traffic', String, traffic_callback)
    rospy.Subscriber('music', Music, music_callback)
    

    r = rospy.Rate(1)

    rospy.spin()

