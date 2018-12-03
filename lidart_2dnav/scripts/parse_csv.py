#!/usr/bin/env python
import rospy
import csv
import os
import sys
import pdb
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from lidart_2dnav.msg import Path
from lidart_2dnav.srv import *
import tf.transformations
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

class waypoint_service:

    def __init__(self):
        self.path_points = self.get_path_points()
        self.path_point_client()

    def path_point_client(self):
        print("Starting")
        rospy.init_node('waypoint_service_node')
        s = rospy.Service('get_path_points', GetPath, self.send_back_path)
        rospy.spin()


    def closest_node(self, node, nodes):
        deltas = nodes - node
        dist_2 = np.einsum('ij,ij->i', deltas, deltas) # returns squared distance of each xy pair in deltas
        # print(np.argmin(dist_2))
        return np.argmin(dist_2)


    def get_path_points(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, 'waypoints/test.csv')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        path_points = [[float(point[0]), float(point[1])] for point in path_points]

        # change path_points into np array to simply processing
        return path_points


    def send_back_path(self, req):
        # print("SERVICING")
        start_point = req.start_point
        num_of_laps = req.num_of_laps
        global path_g
        x = start_point.pose.position.x
        y = start_point.pose.position.y
        pf_point = np.array([x, y])
        first_index = self.closest_node(pf_point, self.path_points)
        last_index = first_index - 1
        print(first_index)
        if last_index == -1:
            path_g = self.path_points
            print(len(path_g))
        else:
            path_g = np.concatenate([self.path_points[first_index:len(self.path_points)], self.path_points[0:last_index + 1]], axis=0)
        path_cloned = np.tile(path_g, (num_of_laps, 1))
        path_point_poses = Path()
        for point in path_cloned:
            current_pose = PoseStamped()
            current_pose.header.frame_id = "map"
            current_pose.pose.position.x = point[0]
            current_pose.pose.position.y = point[1]
            current_pose.pose.position.z = 0
            path_point_poses.poses.append(current_pose)
        return path_point_poses


if __name__ == "__main__":
    service_class = waypoint_service()
