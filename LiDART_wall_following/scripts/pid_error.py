#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
A_ANGLE = 45
B_ANGLE = 90
DESIRED_DISTANCE = 0.05

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  scans = np.array(data.ranges)
  theta_min = data.angle_min
  theta_max = data.angle_max
  theta_delta = data.angle_increment/2.0
  r_min = data.range_min
  r_max = data.range_max
  thetas = np.arange(theta_min,theta_max,theta_delta)
  angle_index = find_nearest(thetas, angle)
  range_val = data.ranges[angle_index]
  return range_val

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  # TODO: implement
  return 0.0

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):

  # NOTE: The lines below initialize the value
  # a_index = find_nearest(thetas, np.deg2rad(45)) # the angle that we want to measure a at
  # angle = thetas[a_index] # 45 degrees
  a = getRange(data, A_ANGLE)
  b = getRange(data, B_ANGLE)

  # Figure out the angle, alpha, Dt using the equations given to us
  theta_diff = A_ANGLE - B_ANGLE
  alpha = np.arctan(a*np.cos(theta_diff) - b)/(a*np.sin(theta_diff))
  Dt = b*np.cos(alpha)

  L = 0.1 # Arbitrary lookahead distance (chnage this)
  Dt_next = Dt + L*np.sin(alpha)

  error = desired_distance - Dt_next

  return error

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  # TODO: implement
  return 0.0

# This method finds the index of value nearest to array
def find_nearest(array, value):
  array = np.asarray(array)
  idx = (np.abs(array - value)).argmin()
  return idx

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):

  error = 0.0 # TODO: replace with followLeft, followRight, or followCenter
  desired_distance = DESIRED_DISTANCE # Randomly decide
  error = followRight(data, desired_distance)
  print(error)
  msg = Float64()
  msg.data = error
  pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
