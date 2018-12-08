#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import tf.transformations
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from occupancy_grid.msg import local_rrt_result
import os
import sys
import pdb
import random
import matplotlib as mpl
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
from nav_msgs.msg import Odometry
from occupancy_grid.msg import OccupancyGrid
from geometry_msgs.msg import Point

pub = rospy.Publisher('local_rrt_result', local_rrt_result, queue_size=1)


scale = 0.25 # one grid equals 0.25m
buffer = 0.20 # a little more than half width of the car, in meters
buffer_grid = math.ceil(buffer / scale)

# in ROS local coordinate system
start_point = np.array([0,0])

def callback(data):
	# get the map and related info
	grid_map = data.occupancy_grid # this should be a np array
	rows = len(grid_map)
	columns = len(grid_map[0])
	x_range = rows * scale # 4
	y_range = columns * scale # 4

	# plotting the map # DEBUG
	plt.gcf().clear()
    plt.ion()
    plt.imshow(grid_map, cmap='gray_r')
    plt.pause(0.0000001)
    plt.show()

	# get the goal_point
	global_end_point = np.array([data.goal_point.x - 0.01, data.goal_point.y])
	curr_location_point = np.zeros([2,1])
	curr_location_point[0] = data.curr_odom.pose.pose.position.x
	curr_location_point[1] = data.curr_odom.pose.pose.position.y
	end_point = global_end_point - curr_location_point
	
	# get yaw
	euler = euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                   data.pose.pose.orientation.z, data.pose.pose.orientation.w))
    yaw = euler[2]

    

if __name__ == '__main__':
    rospy.init_node('local_rrt')
    # TODO: change to the topic and message published by occupancy grid
    rospy.Subscriber('/grid', OccupancyGrid, callback, queue_size=1) 
    rospy.spin()
