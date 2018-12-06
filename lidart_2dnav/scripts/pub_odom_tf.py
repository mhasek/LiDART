#!/usr/bin/env python
import rospy
import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from race.msg import drive_param
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster

translation = (0, 0, 0)
rotation = (0, 0, 0, 0)
child = "base_link"
parent = "map"
time = 0

def callback(data):
    global translation
    global rotation
    global time
    global odom_trans
    translation = (data.pose.pose.position.x, data.pose.pose.position.y, 0)
    rotation = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    time = rospy.get_rostime()

if __name__ == '__main__':
    rospy.init_node('odom_transform')
    rospy.Subscriber('/pf/pose/odom', Odometry, callback, queue_size=1)
    broadcaster = TransformBroadcaster()
    time = rospy.get_rostime()	
    while not rospy.is_shutdown():
        broadcaster.sendTransform(translation, rotation, time, child, parent)
        # print("we published! :D")
	# rospy.spin()
