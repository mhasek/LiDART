#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from LiDART_wall_following.msg import pid_analysis
import numpy as np

error_vals = []

pub = rospy.Publisher('wall_following_analysis',pid_analysis, queue_size=1)


def pid_callback(data):
	error_vals.append(data.data)
	msg = pid_analysis()
	msg.pid_error_ave = mean(error_vals)
	msg.pid_error_max = max(error_vals)
	pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node('lidart_analysis_node', anonymous = True)
	rospy.Subscriber("pid_error", Float64, pid_callback)
	rospy.spin()
