#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from numpy import linalg as LA
import tf.transformations
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import csv
import os
import sys
import pdb

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 1.0 # meters
VELOCITY = 1.0 # m/s
PF_FREQUENCY = 40.0 # prone to change if not in simulator

###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
# path_points: (x,y,theta)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [[float(point[0]), float(point[1]), float(point[2])] for point in path_points]
# change path_points into np array to simply processing
path_points = np.array(path_points)[:,:2]
        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


#############
# FUNCTIONS #
#############
    
# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /pf/viz/inferred_pose.
# Runs pure pursuit and publishes velocity and steering angle.


def closest_node(node, nodes):
    deltas = nodes - node
    dist_2 = np.einsum('ij,ij->i', deltas, deltas) # returns squared distance of each xy pair in deltas
    return np.argmin(dist_2)

def callback(data):

    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom) <- /pf/viz/inferred_pose
    # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.
    euler = euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y,
                                   data.pose.orientation.z, data.pose.orientation.w))
    x = data.pose.position.x
    y = data.pose.position.y
    yaw = euler[2]
    pf_point = np.array([x, y])
    # print(pf_point)

    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    next_waypoint = closest_node(pf_point, path_points)
    distance = dist(pf_point, path_points[next_waypoint])
    # for i in range(closest_point_to_pf, len(path_points)):
    while distance < LOOKAHEAD_DISTANCE and next_waypoint < len(path_points) - 1:
        next_waypoint += 1
        distance = dist(pf_point, path_points[next_waypoint])

    # 3. Transform the goal point to vehicle coordinates. 
    transformed_point = path_points[next_waypoint] - pf_point

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    curvature = 2*transformed_point[0]/LOOKAHEAD_DISTANCE**2
    angle = np.arctan2(curvature * VELOCITY / PF_FREQUENCY, 1)
    print(angle)
    # pdb.set_trace()
    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.spin()

