#!/usr/bin/env python
import numpy as np
import math
from geometry_msgs.msg import Point
import csv
import os
import sys
import pdb
import rospy
from occupancy_grid.srv import *
from occupancy_grid.msg import LastBoxWaypoint
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

class Planner(object):
  def __init__(self, global_waypoints):
    # global_waypoints are a numpy array of waypoints
    rospy.init_node('planner_node')
    s1 = rospy.Service('get_last_waypoint_in_neighborhood', GetLastBoxPoint, self.getLastWaypointInNeighborhoodService)
    s2 = rospy.Service('get_next_pursuit_point', GetNextPursuitPoint, self.getNextWaypointService)
    self.global_waypoints = global_waypoints
    self.next_lap_plus = np.append(global_waypoints, global_waypoints, axis=0)
    self.next_lap_starts_at = len(global_waypoints)
    rospy.spin()

  ## Service, returns next point
  def getNextWaypoint(self, current_location, radius):
    next_waypoint = Point()
    for i in range(len(self.next_lap_plus)):
      if (np.linalg.norm(current_location - self.next_lap_plus[i, :]) >= radius):
        if (i == 0):
          raise Exception("First waypoint is too far away from current location")

        # Remove waypoints that have been passed
        self.next_lap_plus = self.next_lap_plus[i-1:,:]

        # Update the pointer to the start of the next lap
        self.next_lap_start_at = self.next_lap_starts_at - i + 1

        # If less than a lap remains, add another lap of global waypoints
        if (self.next_lap_starts_at <= 0):
          self.next_lap_starts_at = len(self.next_lap_plus)
          self.next_lap_plus = np.append(self.next_lap_plus, self.global_waypoints, axis=0)

        next_waypoint.x = self.next_lap_plus[0, 0]
        next_waypoint.y = self.next_lap_plus[0, 1]
        return next_waypoint
        # return self.next_lap_plus[0,:]
    raise Exception("All waypoints are too close to current location")

  def getLastWaypointInNeighborhood(self, x, y, theta, neighborhood_length):
    last_point = Point()
    last_box_waypoint = LastBoxWaypoint()
    for i in range(len(self.next_lap_plus)):
      # vector = self.next_lap_plus[i, :] - np.array([x,y])
      # transform_mat = np.array([[np.cos(-theta),-np.sin(-theta)],[np.sin(-theta),np.cos(-theta)]])
      # vector_local = np.matmul(transform_mat, vector.reshape(2,-1))
      # if (vector_local[1] > neighborhood_length):
      #   last_box_waypoint.out_direction = 0
      #   last_point.x = self.next_lap_plus[i - 1, 0]
      #   last_point.y = self.next_lap_plus[i - 1, 1]
      #   print(last_point)
      #   print(last_box_waypoint.out_direction)
      #   last_box_waypoint.last_waypoint_in_neighborhood = last_point
      #   return last_box_waypoint
      # elif (vector_local[0] > neighborhood_length / 2):
      #   last_box_waypoint.out_direction = 1
      #   last_point.x = self.next_lap_plus[i - 1, 0]
      #   last_point.y = self.next_lap_plus[i - 1, 1]
      #   print(last_point)
      #   print(last_box_waypoint.out_direction)
      #   last_box_waypoint.last_waypoint_in_neighborhood = last_point
      #   return last_box_waypoint
      # elif (vector_local[0] < - neighborhood_length / 2):
      #   last_box_waypoint.out_direction = -1
      #   last_point.x = self.next_lap_plus[i - 1, 0]
      #   last_point.y = self.next_lap_plus[i - 1, 1]
      #   print(last_point)
      #   print(last_box_waypoint.out_direction)
      #   last_box_waypoint.last_waypoint_in_neighborhood = last_point
      #   return last_box_waypoint

      dx = self.next_lap_plus[i, 0] - x
      dy = self.next_lap_plus[i, 1] - y

      if (abs(dx * math.cos(theta) + dy * math.sin(theta)) > neighborhood_length):
        # Exit Forward
        # print("Exit Forward")
        last_box_waypoint.out_direction = 0
        last_point.x = self.next_lap_plus[i - 1, 0]
        last_point.y = self.next_lap_plus[i - 1, 1]
        last_box_waypoint.last_waypoint_in_neighborhood = last_point
        return last_box_waypoint
      if (dx * math.sin(theta) - dy * math.cos(theta) > neighborhood_length / 2):
        # Exit Right
        # print("Exit Right")
        last_box_waypoint.out_direction = 1
        last_point.x = self.next_lap_plus[i - 1, 0]
        last_point.y = self.next_lap_plus[i - 1, 1]
        last_box_waypoint.last_waypoint_in_neighborhood = last_point
        return last_box_waypoint
      if (-dx * math.sin(theta) + dy * math.cos(theta) > neighborhood_length / 2):
        # Exit Left
        # print("Exit Left")
        last_box_waypoint.out_direction = -1
        last_point.x = self.next_lap_plus[i - 1, 0]
        last_point.y = self.next_lap_plus[i - 1, 1]
        last_box_waypoint.last_waypoint_in_neighborhood = last_point
        return last_box_waypoint

    raise Exception("All waypoints are in the neighborhood")

  # this is not used
  def updateWaypoints(self, new_waypoints, last_waypoint):
    for i in range(len(self.next_lap_plus)):
      if (np.array_equal(last_waypoint, self.next_lap_plus[i, :])):
        print(new_waypoints)
        print(self.next_lap_plus)
        self.next_lap_plus = np.append(new_waypoints, self.next_lap_plus[i + 1:, :], axis=0)

        # If less than a lap remains, add another lap of global waypoints
        if (self.next_lap_starts_at <= i):
          self.next_lap_starts_at = len(self.next_lap_plus)
          self.next_lap_plus = np.append(self.next_lap_plus, global_waypoints, axis=0)

        return
    raise Exception("That waypoint does not exist")

  def getLastWaypointInNeighborhoodService(self, req):
      neighborhood_len = 3.0
      x = req.current_odom.pose.pose.position.x
      y = req.current_odom.pose.pose.position.y
      euler = euler_from_quaternion((req.current_odom.pose.pose.orientation.x, req.current_odom.pose.pose.orientation.y,
                                     req.current_odom.pose.pose.orientation.z, req.current_odom.pose.pose.orientation.w)) # Make a new service
      theta = euler[2]
      print("odom: ", x, " ", y)
      last_waypoint_in_neighborhood = self.getLastWaypointInNeighborhood(x, y, theta, neighborhood_len)
      # MAKE A MSG TYPE FOR THIS
      return last_waypoint_in_neighborhood

  def getNextWaypointService(self, req):
      position = np.array([req.current_odom.pose.pose.position.x, req.current_odom.pose.pose.position.y])
      radius = req.radius
      next_way = self.getNextWaypoint(position, radius)
      return next_way

def read_csv():
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../waypoints/test.csv')
    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f)]

    # Turn path_points into a list of floats to eliminate the need for casts in the code below.
    path_points = [[float(point[0]), float(point[1]), float(point[2])] for point in path_points]
    # change path_points into np array to simply processing
    path_points = np.array(path_points)[:,:2]
    return path_points

if __name__ == '__main__':
    path_points = read_csv()
    planner = Planner(path_points)
