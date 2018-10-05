#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Int32
import pdb
from track_parser import Direction

pub = rospy.Publisher('pid_error', Float64, queue_size=10)
# pub_dist = rospy.Publisher('curr_distance', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
A_ANGLE = 45
B_ANGLE = 90
C_ANGLE = -45
D_ANGLE = -90

# Global driving instruction booleans
# Exactly one of these should be true at a time
global followingLeft
global followingRight
global followingCenter

OAT = 8
DESIRED_DISTANCE = 0.3
L = 0.25 # Arbitrary lookahead distance (chnage this)

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  scans = np.array(data.ranges)
  theta_min = data.angle_min
  theta_max = data.angle_max
  theta_delta = data.angle_increment
  r_min = data.range_min
  r_max = data.range_max
  thetas = np.arange(theta_min,theta_max,theta_delta)
  angle_index = find_nearest(thetas, np.deg2rad(angle))
  # cap infinite values at OAT
  range_val = data.ranges[angle_index] if data.ranges[angle_index] < OAT else OAT
  
  return range_val

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  a = getRange(data, C_ANGLE)
  b = getRange(data, D_ANGLE)

  # Figure out the angle, alpha, Dt using the equations given to us
  theta_diff = np.deg2rad(C_ANGLE - D_ANGLE)
  # pdb.set_trace()
  alpha = np.arctan(a*np.cos(theta_diff) - b)/(a*np.sin(theta_diff))
  Dt = b*np.cos(alpha)


  Dt_next = Dt + L*np.sin(alpha)

  error = desired_distance - Dt_next

  return error


# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):

  # NOTE: The lines below initialize the value
  # a_index = find_nearest(thetas, np.deg2rad(45)) # the angle that we want to measure a at
  # angle = thetas[a_index] # 45 degrees
  a = getRange(data, A_ANGLE)
  b = getRange(data, B_ANGLE)

  # Figure out the angle, alpha, Dt using the equations given to us
  theta_diff = np.deg2rad(B_ANGLE - A_ANGLE)
  # pdb.set_trace()
  alpha = np.arctan(a*np.cos(theta_diff) - b)/(a*np.sin(theta_diff))
  Dt = b*np.cos(alpha)


  Dt_next = Dt + L*np.sin(alpha)

  error = Dt_next - desired_distance

  return error

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):

  a = getRange(data, A_ANGLE)
  b = getRange(data, B_ANGLE)

  # Figure out the angle, alpha, Dt using the equations given to us
  theta_diff = np.deg2rad(B_ANGLE - A_ANGLE)
  # pdb.set_trace()
  alpha = np.arctan(a*np.cos(theta_diff) - b)/(a*np.sin(theta_diff))
  Dt = b*np.cos(alpha)


  Dt_next_r = Dt + L*np.sin(alpha)

  a = getRange(data, C_ANGLE)
  b = getRange(data, D_ANGLE)

  # Figure out the angle, alpha, Dt using the equations given to us
  theta_diff = np.deg2rad(C_ANGLE - D_ANGLE)
  # pdb.set_trace()
  alpha = np.arctan(a*np.cos(theta_diff) - b)/(a*np.sin(theta_diff))
  Dt = b*np.cos(alpha)
  

  Dt_next_l = Dt + L*np.sin(alpha)


  D_total = Dt_next_r + Dt_next_l

  error = Dt_next_r - D_total/2

  return error

# Execute different driving strategies and return the resulting error
def followInstructions(data, desired_distance):
  if followingCenter:
    return followCenter(data)
  elif followingLeft:
    return followLeft(data, desired_distance)
  elif followingRight:
    return followRight(data, desired_distance)



# This method finds the index of value nearest to array
def find_nearest(array, value):
  array = np.asarray(array)
  idx = (np.abs(array - value)).argmin()
  return idx

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  error = followInstructions(data, DESIRED_DISTANCE)
  # error = followLeft(data, DESIRED_DISTANCE)
  # error = followRight(data, DESIRED_DISTANCE)
  # error = followLeft(data, DESIRED_DISTANCE)
  msg = Float64()
  msg.data = error
  pub.publish(msg)

# Takes next command and sets driving instruction booleans accordingly
def command_callback(next_command):
  global followingRight
  global followingLeft
  global followingCenter
  if next_command == Direction.LEFT:
    followingLeft = True
    followingRight = False
    followingCenter = False
  if next_command == Direction.RIGHT:
    followingLeft = False
    followingRight = True
    followingCenter = False
  if next_command == Direction.CENTER:
    followingLeft = False
    followingRight = False
    followingCenter = True


# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  global followingCenter
  global followingLeft
  global followingRight
  followingCenter = True
  followingLeft = False
  followingRight = False
  rospy.init_node('pid_error_node', anonymous = True)
  rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.Subscriber("command", Int32, command_callback)
  rospy.spin()
