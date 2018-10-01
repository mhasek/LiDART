#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import pdb

# TODO: modify these constants to make the car follow walls smoothly.
KP = 2.5
KD = 1.0

dt = 0.025


e_t0 = 0.0
e_t1 = 0.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(data):
  # TODO: Based on the error (data.data), determine the car's required velocity
  # amd steering angle.
  e_t1 = data.data
  global e_t0
  u = KP * e_t1 + KD * (e_t1 - e_t0)
  u = np.rad2deg(u)
  if u > 30:
      u = 30
  elif u < -30:
      u = -30
  if abs(u) <= 10:
      vel = 1.5
  elif abs(u) <= 20:
      vel = 1.0
  else:
      vel = 0.5
  u = np.deg2rad(u)
  msg = drive_param()
  msg.velocity = vel  # TODO: implement PID for velocity
  msg.angle = u    # TODO: implement PID for steering angle
  print msg
  pub.publish(msg)
  e_t0 = data.data

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", Float64, control_callback)
	rospy.spin()
