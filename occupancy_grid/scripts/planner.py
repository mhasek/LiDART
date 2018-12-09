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
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class Planner(object):
  def __init__(self, global_waypoints):
    # global_waypoints are a numpy array of waypoints
    rospy.init_node('planner_node')
    rospy.Subscriber("/pf/pose/odom", Odometry, self.odom_callback)
    s1 = rospy.Service('get_last_waypoint_in_neighborhood', GetLastBoxPoint, self.getLastWaypointInNeighborhoodService)
    s2 = rospy.Service('get_next_pursuit_point', GetNextPursuitPoint, self.getNextWaypointService)
# global_waypoints are a numpy array of waypoints
    self.global_waypoints = np.array(global_waypoints)
    self.pub_waypoint_marker = rospy.Publisher('next_waypoint_planner', Marker, queue_size="1")

    # print(len(global_waypoints))
    # print(int(len(global_waypoints) / 2))
    # print(len(global_waypoints))
    self.global_first_half = self.global_waypoints[:int(len(global_waypoints) / 2),:]
    self.global_second_half = self.global_waypoints[int(len(global_waypoints) / 2):,:]
    
    # next lap has between half a lap and a whole lap
    self.next_lap = np.append(self.global_first_half, self.global_second_half, axis=0)
    self.next_half_starts_at = len(self.global_first_half)
    self.current_half = 1
    print("max x of waypoints is ", np.max(self.global_waypoints[:,0]))
    print("min x of waypoints is ", np.min(self.global_waypoints[:,0]))
    print("max y of waypoints is ", np.max(self.global_waypoints[:,1]))
    print("min y of waypoints is ", np.min(self.global_waypoints[:,1]))
    rospy.spin()
    


  def odom_callback(self, data):
    x = data.pose.pose.position.x
    # print("odom", x)
    y = data.pose.pose.position.y
    self.updateFromOdometry([x, y])
      
  def updateFromOdometry(self, current_location):
    dist = np.linalg.norm(self.next_lap - current_location, axis=1)
    closest_index = np.argmin(dist)
    print(closest_index)
    print("waypoints left in lap ", len(self.next_lap))
    print("waypoints left in half lap ", self.next_half_starts_at)
    self.next_lap = self.next_lap[closest_index:, :]
    self.next_half_starts_at = self.next_half_starts_at - closest_index
    if (self.next_half_starts_at <= 0):
      self.next_half_starts_at = len(self.next_lap)
      if (self.current_half == 1):
        self.current_half = 2
        self.next_lap = np.append(self.next_lap, self.global_first_half, axis=0)
      else:
        self.current_half = 1
        self.next_lap = np.append(self.next_lap, self.global_second_half, axis=0)

  ## Service, returns next point
  def getNextWaypoint(self, current_location, radius):
    point = Point()
    for i in range(len(self.next_lap)):
      print(radius)
      if (np.linalg.norm(current_location - self.next_lap[i, :], axis=0) >= radius):
        if (i == 0):
          raise Exception("First waypoint is too far away from current location")
        
        point.x = self.next_lap[i-1,0]
        point.y = self.next_lap[i-1,1]

        # return self.next_lap[0,:]
        return point
    raise Exception("All waypoints are too close to current location")

  def getLastWaypointInNeighborhood(self, x, y, theta, neighborhood_length):
    last_point = Point()
    last_box_waypoint = LastBoxWaypoint()
    for i in range(len(self.next_lap)):
      # print(self.next_lap)
      dx = self.next_lap[i, 0] - x
      dy = self.next_lap[i, 1] - y
      # print("i:", i)
      # print("dx", dx)
      # print("dy", dy)
      # print("theta", theta)
      if (abs(dx * math.cos(theta) + dy * math.sin(theta)) > neighborhood_length):
        # Exit Forward
        # print("Exit Forward")
        last_box_waypoint.out_direction = 0
        last_point.x = self.next_lap[i - 1, 0]
        last_point.y = self.next_lap[i - 1, 1]
        last_box_waypoint.last_waypoint_in_neighborhood = last_point
        return last_box_waypoint
      if (dx * math.sin(theta) - dy * math.cos(theta) > neighborhood_length / 2):
        # Exit Right
        # print("Exit Right")
        last_box_waypoint.out_direction = 1
        last_point.x = self.next_lap[i - 1, 0]
        last_point.y = self.next_lap[i - 1, 1]
        last_box_waypoint.last_waypoint_in_neighborhood = last_point
        return last_box_waypoint
      if (-dx * math.sin(theta) + dy * math.cos(theta) > neighborhood_length / 2):
        # Exit Left
        # print("Exit Left")
        last_box_waypoint.out_direction = -1
        last_point.x = self.next_lap[i - 1, 0]
        last_point.y = self.next_lap[i - 1, 1]
        last_box_waypoint.last_waypoint_in_neighborhood = last_point
        return last_box_waypoint

    raise Exception("All waypoints are in the neighborhood")

  # this is not used
  def updateWaypoints(self, new_waypoints, last_waypoint):
    for i in range(len(self.next_lap)):
      if (np.array_equal(last_waypoint, self.next_lap[i, :])):
        self.next_lap = np.append(new_waypoints, self.next_lap[i + 1:, :], axis=0)
        
        # If less than a lap remains, add another lap of global waypoints
        if (self.next_half_starts_at <= i):
          if (self.current_half == 1):
            self.next_half_starts_at = len(self.next_lap)
            self.next_lap = np.append(self.next_lap, self.global_first_half, axis=0)
            self.current_lap = 2
          else:
            self.next_half_starts_at = len(self.next_lap)
            self.next_lap = np.append(self.next_lap, self.global_second_half, axis=0)
            self.current_lap = 1
          
        return
    raise Exception("That waypoint does not exist")

  def getLastWaypointInNeighborhoodService(self, req):
      neighborhood_len = 3.0
      x = req.current_odom.pose.pose.position.x
      y = req.current_odom.pose.pose.position.y
      euler = euler_from_quaternion((req.current_odom.pose.pose.orientation.x, req.current_odom.pose.pose.orientation.y,
                                     req.current_odom.pose.pose.orientation.z, req.current_odom.pose.pose.orientation.w)) # Make a new service
      theta = euler[2]
      # print("neighborhood", x)
      # print("odom: ", x, " ", y)
      last_waypoint_in_neighborhood = self.getLastWaypointInNeighborhood(x, y, theta, neighborhood_len)
      pts = last_waypoint_in_neighborhood.last_waypoint_in_neighborhood
        
           # next waypoint marker
      marker = Marker()
      marker.header.frame_id = "/map"
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0
      marker.pose.orientation.w = 1.0
      marker.pose.position.x = pts.x
      marker.pose.position.y = pts.y
      marker.pose.position.z = 0
      self.pub_waypoint_marker.publish(marker)

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
