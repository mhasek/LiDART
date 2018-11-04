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

LOOKAHEAD_DISTANCE = 0.25 # meters
VELOCITY = 1.0 # m/s


###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [[float(point[0]), float(point[1]), float(point[2])] for point in path_points]
        
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
    nodes = np.array(nodes)
    # node = node.reshape((1, -1))
    # nodes = nodes.reshape((len(nodes), -1))
    deltas = nodes - node
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

def callback(data):

    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
    # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.
    euler = euler_from_quaternion((data.pose.orientation.x, data.pose.orientation.y,
                                   data.pose.orientation.z, data.pose.orientation.w))
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    yaw = euler[2]
    pf_point = np.array([x, y, z])
    # print(odom_point)


    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    last_distance = sys.maxint
    closest_point_to_pf = closest_node(pf_point, path_points)
    # print((closest_point_to_odom))
    for i in range(closest_point_to_pf, len(path_points)):
    # for i in range(0, len(path_points)):
        distance = dist(pf_point, path_points[i])
        if distance >= LOOKAHEAD_DISTANCE:
            last_distance = distance
            closest_point = np.array(path_points[i])
    	    break

    # 3. Transform the goal point to vehicle coordinates. 
    transformed_point = closest_point - pf_point
    

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    angle = -2*transformed_point[0]/LOOKAHEAD_DISTANCE**2

    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.spin()

