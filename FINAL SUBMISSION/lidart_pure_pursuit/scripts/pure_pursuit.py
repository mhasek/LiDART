#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as LA
import tf.transformations
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from occupancy_grid.srv import *
import csv
import os
import sys
import pdb
from nav_msgs.msg import Odometry

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 2.0 # meters, does nothing
MAX_VELOCITY = 3.0 # m/s
L = 0.325 # meters
FAR_DISTANCE_L_COEFF = 1.5
FAR_DISTANCE_V_COEFF = 0.2
DIFF_ANGLE_V_COEFF = 0.5 #0.5
CURR_ANGLE_V_COEFF = 0.5 #0.5
VELOCITY_BASE = 1.5 #1
VELOCITY = 1.0
MIN_LOOKAHEAD_DISTANCE = 0.5

###########
# GLOBALS #
###########

current_odom = Odometry()

# Import waypoints.csv into a list (path_points)
# path_points: (x,y,theta)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/wide.csv')
#filename = os.path.join(dirname, '../waypoints/gen_waypts.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the ned for casts in the code below.
path_points = [[float(point[0]), float(point[1]), float(point[2])] for point in path_points]
# change path_points into np array to simply processing
path_points = np.array(path_points)[:,:2]
path_points = path_points
        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
pub_waypoint_marker = rospy.Publisher('next_waypoint_viz', Marker, queue_size="1")
pub_pf_marker = rospy.Publisher('pf_point_viz', Marker, queue_size="1")
pub_far_ahead_waypoint_marker = rospy.Publisher('far_ahead_point_viz', Marker, queue_size="1")

#############
# FUNCTIONS #
#############

## give odometry and inner rad and outer rad
def get_next_point_client(odom, distance):
    rospy.wait_for_service('get_next_pursuit_point')
    get_next_waypoint = rospy.ServiceProxy('get_next_pursuit_point', GetNextPursuitPoint)
    try:
        resp1 = get_next_waypoint(odom, distance)
        next_point_ = resp1.pursuit_point
        return np.array([next_point_.x,next_point_.y])

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /pf/viz/inferred_pose.
# Runs pure pursuit and publishes velocity and steering angle.


def closest_node(node, nodes):
    deltas = nodes - node
    dist_2 = np.einsum('ij,ij->i', deltas, deltas) # returns squared distance of each xy pair in deltas
    return np.argmin(dist_2)

def velocity(angle, far_ahead_angle):
    velocity = VELOCITY_BASE
    velocity += max((1-abs(angle/np.deg2rad(18)))*CURR_ANGLE_V_COEFF,0)
    velocity += max((1-abs((angle-far_ahead_angle)/np.deg2rad(18)))*DIFF_ANGLE_V_COEFF,0)
    #print(velocity)
    return min(velocity, MAX_VELOCITY)

def odom_callback(data):
    global current_odom
    current_odom = data

def callback(data):
    global LOOKAHEAD_DISTANCE

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
    # next_waypoint = closest_node(pf_point, path_points)
    # distance = dist(pf_point, path_points[next_waypoint])
    # for i in range(closest_point_to_pf, len(path_points)):

    odom = current_odom
    close_LOOKAHEAD_DISTANCE = LOOKAHEAD_DISTANCE
    close_point = get_next_point_client(odom, close_LOOKAHEAD_DISTANCE)
    distance = dist(pf_point, close_point)


    far_LOOKAHEAD_DISTANCE = LOOKAHEAD_DISTANCE * (FAR_DISTANCE_L_COEFF - 1) + distance + FAR_DISTANCE_V_COEFF * MAX_VELOCITY
    far_point = get_next_point_client(odom, far_LOOKAHEAD_DISTANCE)
    far_distance = dist(pf_point, far_point)

    # print(far_LOOKAHEAD_DISTANCE)

    # while distance < LOOKAHEAD_DISTANCE :
    #     next_waypoint = (next_waypoint + 1) % len(path_points)
    #     distance = dist(pf_point, path_points[next_waypoint])

    # far_ahead_waypoint = next_waypoint
    # far_distance = dist(pf_point, path_points[far_ahead_waypoint])
    # while far_distance < LOOKAHEAD_DISTANCE * (FAR_DISTANCE_L_COEFF - 1) + distance + FAR_DISTANCE_V_COEFF * MAX_VELOCITY :
    #     far_ahead_waypoint = (far_ahead_waypoint + 1) % len(path_points)
    #     far_distance = dist(pf_point, path_points[far_ahead_waypoint])


    # 3. Transform the goal point to vehicle coordinates. 
    # waypoint_value = path_points[next_waypoint,:]
    waypoint_value = close_point
    transform_vector = waypoint_value - pf_point
    transform_mat = np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
    transform_vector_local = np.matmul(transform_mat,transform_vector)

    far_ahead_waypoint_value = far_point
    # far_ahead_waypoint_value = path_points[far_ahead_waypoint,:]
    far_ahead_transform_vector = far_ahead_waypoint_value - pf_point
    far_ahead_transform_vector_local = np.matmul(transform_mat,far_ahead_transform_vector)

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    curvature = 2*transform_vector_local[1]/distance**2
    far_ahead_curvature = 2*far_ahead_transform_vector_local[1]/far_distance**2
    angle = np.arctan2(curvature * L, 1)
    far_ahead_angle = np.arctan2(far_ahead_curvature * L, 1)
    # print(path_points[next_waypoint])
    print waypoint_value
    # print(distance)
    # print(next_waypoint)

    # next waypoint marker
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = waypoint_value[0]
    marker.pose.position.y = waypoint_value[1]
    marker.pose.position.z = 0
    pub_waypoint_marker.publish(marker)

    # far ahead waypoint marker
    marker3 = Marker()
    marker3.header.frame_id = "/map"
    marker3.type = marker3.SPHERE
    marker3.action = marker3.ADD
    marker3.scale.x = 0.2
    marker3.scale.y = 0.2
    marker3.scale.z = 0.2
    marker3.color.a = 1.0
    marker3.color.r = 0.0
    marker3.color.g = 0.0
    marker3.color.b = 0.0
    marker3.pose.orientation.w = 1.0
    marker3.pose.position.x = far_ahead_waypoint_value[0]
    marker3.pose.position.y = far_ahead_waypoint_value[1]
    marker3.pose.position.z = 0
    pub_far_ahead_waypoint_marker.publish(marker3)

    # pf_point marker
    marker2 = Marker()
    marker2.header.frame_id = "/map"
    marker2.type = marker2.SPHERE
    marker2.type = marker2.ADD
    marker2.scale.x = 0.2
    marker2.scale.y = 0.2
    marker2.scale.z = 0.2
    marker2.color.a = 1.0
    marker2.color.r = 0.0
    marker2.color.g = 1.0
    marker2.color.b = 0.0
    marker2.pose.orientation.w = 1.0
    marker2.pose.position.x = pf_point[0]
    marker2.pose.position.y = pf_point[1]
    marker2.pose.position.z = 0
    pub_pf_marker.publish(marker2)

    # pdb.set_trace()
    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
    # print(angle)
    vel = velocity(angle, far_ahead_angle)
    msg = drive_param()
    msg.velocity = vel
    # LOOKAHEAD_DISTANCE = 1.5*np.abs(msg.velocity)/(MAX_VELOCITY) + MIN_LOOKAHEAD_DISTANCE
    vel_th1 = 1.5
    vel_th2 = 2.5
    LA_min = 0.7
    LA_max = 1.7
    if vel > vel_th2:
        LOOKAHEAD_DISTANCE = LA_max
    elif vel < vel_th1:
	LOOKAHEAD_DISTANCE = LA_min
    else:
	LOOKAHEAD_DISTANCE = LA_min + (LA_max-LA_min)*(vel-vel_th1)/(vel_th2-vel_th1) 
	
    print("LOOKAHEAD: ",LOOKAHEAD_DISTANCE)
    print("vel:",vel)
    # msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber("/pf/pose/odom", Odometry, odom_callback)
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
rospy.spin()
