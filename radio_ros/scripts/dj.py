#! /usr/bin/env python
import rospy
from radio_ros.msg import Music
import random

MUSIC_LIST = []

def generate_music_list():
    music = Music()
    music.title = 'A Sky Full of Stars'
    music.album = 'Ghost Stories'
    music.artist = 'Coldplay'
    MUSIC_LIST.append(music)

    music = Music()
    music.title = 'Highway to Hell'
    music.album = 'Highway to Hell'
    music.artist = 'AC/DC'
    MUSIC_LIST.append(music)

if __name__ == '__main__':
    generate_music_list()

    rospy.init_node('dj_node')
    pub = rospy.Publisher('music', Music, queue_size=10)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        musik = random.choice(MUSIC_LIST)
        pub.publish(musik)
        r.sleep()