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
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker

# pub = rospy.Publisher('local_rrt_result', local_rrt_result, queue_size=1)
rst_point_pub = rospy.Publisher('local_planner_next_point', Point, queue_size=1)

scale = 0.125 # one grid equals 0.25m
buffer = 0.126 # a little more than half width of the car, in meters
meter_height = 3.0
meter_width = 3.0
buffer_grid = math.ceil(buffer / scale)
step_size = 0.5
rows = 0
columns = 0
grid_map = np.empty([0,0])

counter = 0 # counter used to plot and debug

# in ROS local coordinate system
start_point = np.array([0,0])

# visualization pub of new local paths
path_pub_ = rospy.Publisher('local_rrt_path_viz', Marker, latch=True, queue_size=10)

MAX_VELOCITY = 5 # m/s
VELOCITY_BASE = 1
LOOKAHEAD_DISTANCE = 2

# --- helper functions --- #
def xy_to_grid(point):
  grid = np.zeros(2).astype(int)
  grid[0] = math.floor(point[0] / scale)
  grid[1] = math.floor(- point[1] / scale) + (columns * scale) / (2 * scale)
  return grid

def find_max_driveable_col_range(row): # O(w)
  col_range = np.array([0,0])
  curr_range = 0
  col_start = 0
  col_end = 0
  
  while col_start < columns:
    while not is_pixel_driveable(row, col_start, True):
      col_start += 1
      if (col_start >= columns): 
        return col_range + np.array([buffer_grid, -buffer_grid])
    col_end = col_start    
    while is_pixel_driveable(row, col_end, True):
      col_end += 1
      if (col_end >= columns): break
    if (col_end - col_start > curr_range):
      col_range[0] = col_start
      col_range[1] = col_end
      curr_range = col_end - col_start
    col_start = col_end
    
  col_range = col_range + np.array([buffer_grid, -buffer_grid])
  
  return col_range

def find_max_driveable_row_range(col): # O(h)
  row_range = np.array([0,0])
  curr_range = 0
  row_start = 0
  row_end = 0
  
  while row_start < rows:
    while not is_pixel_driveable(row_start, col, False):
      row_start += 1
      if (row_start >= rows): 
        return row_range + np.array([1, -1]) # TODO
    row_end = row_start
    while is_pixel_driveable(row_end, col, False):
      row_end += 1
      if (row_end >= rows): break
    if (row_end - row_start > curr_range):
      row_range[0] = row_start
      row_range[1] = row_end
      curr_range = row_end - row_start
    row_start = row_end
    
  row_range = row_range + np.array([1, -1])
  
  return row_range

def is_pixel_driveable(row,col,row_major):
  is_driveable = (grid_map[row][col] == 0)
  if (row_major):
    up_bound = min(row + buffer_grid, rows)
    lower_bound = max(row - buffer_grid, 0)
    is_driveable = is_driveable and (np.sum(grid_map[lower_bound:up_bound + 1,col]) == 0)
  else:
    up_bound = min(col + buffer_grid, columns)
    lower_bound = max(col - buffer_grid, 0)
    is_driveable = is_driveable and (np.sum(grid_map[row, lower_bound:up_bound + 1]) == 0)
  return is_driveable

# return the center (x,y) of a grid
def grid_to_xy(grid):
  xy = np.zeros(2)
  xy[0] = (grid[0] + 0.5) * scale
  xy[1] = - (grid[1] + 0.5 - (columns * scale) / (2 * scale)) * scale
  return xy

# distance(point1, point2)
# what it does: find the distance between 2 points (x,y)
# input: 2 points
# output: float distance
def distance(point1, point2):
  return math.sqrt(np.sum((point1 - point2)**2))

# brute force, not intelligent way, but O(h)
def isValidEdge(start_point, end_point):
  start_grid = xy_to_grid(start_point)
  end_grid = xy_to_grid(end_point)
  
  #print("isValidEdge start point: ", start_point)
  #print("isValidEdge end point: ", end_point)
  lower_point = start_point
  upper_point = np.zeros([2,0])
  vector = end_point - start_point
  if not vector[0] == 0:
    slope = vector[1] / vector[0] # slope = dy / dx
  
  for r in range(start_grid[0], end_grid[0] + 1):
    if vector[0] == 0 and vector[1] > 0:
      upper_point = np.array([lower_point[0], 0])
    elif vector[0] == 0 and vector[1] < 0:
      upper_point = np.array([lower_point[0], columns - 1])
    else:
      upper_point = lower_point + np.array([scale, slope * scale])
    col1 = xy_to_grid(lower_point)[1]
    col2 = xy_to_grid(upper_point)[1]
    if (col1 > col2): col1, col2 = col2, col1
    # if (np.sum(grid_map[r,col1:(col2 + 1)]) > 0):
    if (np.sum(grid_map[r,(col1 - buffer_grid):(col2 + buffer_grid + 1)]) > 0):
      #print(start_point,"," , end_point, " is not a valid edge")
      #print("because of row ",r, " and columns: ", col1, " ", col2)
      return False
    lower_point = upper_point
  return True
    
def modified_RRT(start_point, end_point, step_size, out_direction):
  latest_point = start_point
  path = np.zeros([100,2])
  path[0,:] = start_point
  cnt = 1
  rotations = 0
  max_iterations = 20
  
  if out_direction == 0:
    while(distance(latest_point, end_point) > step_size and rotations <= max_iterations):
      rotations += 1
      #print("cnt: ", cnt)
      latest_point = path[cnt - 1, :]
      #print("latest_point: ", latest_point)
      random_point = latest_point + np.array([step_size, 0])
      curr_r = xy_to_grid(random_point)[0]

      if (curr_r >= rows):
        print("curr_r > rows: ", curr_r)
        break
        
      col_range = find_max_driveable_col_range(curr_r)
      if (col_range[1] - col_range[0] == 0):
        print("no valid grid to sample from!")
      min_y = grid_to_xy(np.array([curr_r, col_range[1]]))[1] - scale * 0.5
      max_y = grid_to_xy(np.array([curr_r, col_range[0]]))[1] + scale * 0.5

      #print("min_y: ", min_y, " max_y: ", max_y)

      random_point[1] = random.random() * (max_y - min_y) + min_y
      #print("random_point: ", random_point)
      vector = random_point - latest_point
      #print("vector: ", vector)
      angle = math.atan2(vector[1], vector[0])
      #print("angle: ", angle)
      next_point = latest_point + step_size * np.array([math.cos(angle), math.sin(angle)])
      #print("next_point: ", next_point)

      if isValidEdge(latest_point, next_point):
        #print(next_point)
        path[cnt,:] = next_point
        #print(path[:cnt+1,:])
        cnt += 1
  else:
    while(distance(latest_point, end_point) > step_size and rotations <= max_iterations):
      rotations += 1
      # print("cnt: ", cnt)
      latest_point = path[cnt - 1, :]
      #print("latest_point: ", latest_point)
      if (out_direction == -1):
        random_point = latest_point + np.array([0, step_size])
      else:
        random_point = latest_point + np.array([0, - step_size])
      #print("random_point: ", random_point)
      curr_c = xy_to_grid(random_point)[1]

      if (curr_c >= columns):
        print("curr_c > columns: ", curr_c)
        break

      row_range = find_max_driveable_row_range(curr_c)
      #print("row_range: ", row_range)
      
      if (row_range[1] - row_range[0] == 0):
        print("no valid grid to sample from!")
      min_x = grid_to_xy(np.array([row_range[0], curr_c]))[0] - scale * 0.5
      max_x = grid_to_xy(np.array([row_range[1], curr_c]))[0] + scale * 0.5

      #print("min_x: ", min_x, " max_x: ", max_x)

      random_point[0] = random.random() * (max_x - min_x) + min_x
      #print("random_point: ", random_point)
      vector = random_point - latest_point
      #print("vector: ", vector)
      angle = math.atan2(vector[1], vector[0])
      #print("angle: ", angle)
      next_point = latest_point + step_size * np.array([math.cos(angle), math.sin(angle)])
      #print("next_point: ", next_point)

      if isValidEdge(latest_point, next_point):
        #print("adding next point: ", next_point)
        path[cnt,:] = next_point
        #print("after adding this point: ", path[:cnt+1,:])
        cnt += 1
  
  if rotations > max_iterations:
    return np.zeros([0,2])
  else:
    print("before adding end point: ", path[:cnt,:])
    path[cnt,:] = end_point
    return path[:cnt + 1,:]

def modified_next_point(lookahead_distance, out_direction):
  if out_direction == 0:
    random_point = np.array([step_size, 0])
    curr_r = xy_to_grid(random_point)[0]
    col_range = find_max_driveable_col_range(curr_r)
    if (col_range[1] - col_range[0] < 0):
      print("no valid grid to sample from!") # TODO: go backwards until you find one row
      
    min_y = grid_to_xy(np.array([curr_r, col_range[1]]))[1] - scale * 0.5
    max_y = grid_to_xy(np.array([curr_r, col_range[0]]))[1] + scale * 0.5
    mid_y = (min_y + max_y)/2

    return np.array([lookahead_distance,mid_y])
  else:
    random_point = np.array([0, step_size])
    curr_c = xy_to_grid(random_point)[1]
    row_range = find_max_driveable_row_range(curr_c)
    if (row_range[1] - row_range[0] < 0):
      print("no valid grid to sample from!") # TODO: go backwards until you find one row
    
    min_x = grid_to_xy(np.array([row_range[0], curr_c]))[0] - scale * 0.5
    max_x = grid_to_xy(np.array([row_range[1], curr_c]))[0] + scale * 0.5
    mid_x = (min_x + max_x)/2

    return np.array([mid_x, lookahead_distance])

def displayMapAndPath(path, grid_map):
  freeSpaceboxes = []
  fig = mpl.pyplot.figure()
  ax = fig.add_subplot(1, 1, 1)

  for r in range(rows):
    for c in range(columns):
      if (grid_map[r, c] == 0):
        center = grid_to_xy(np.array([r,c]))
        rect = mpl.patches.Rectangle((center[0] - scale * 0.5, center[1] - scale * 0.5), scale, scale, color='y')
        ax.add_patch(rect)

  edges = []
  for i in range(1, len(path)):
    point = (path[i,0], path[i,1])
    parent = (path[i-1,0], path[i-1,1])
    edge = [point, parent]
    edges.append(edge)
  edgeCollection = mpl.collections.LineCollection(edges)

  ax.add_collection(edgeCollection)
  ax.autoscale()
  ax.margins(0.1)
  mpl.pyplot.show()
  # mpl.pyplot.savefig('map ' + str(counter) + '.png')

def within_bound(pixel):
  return pixel[0] >= 0 and pixel[0] < rows and pixel[1] >= 0 and pixel[1] < columns

def publish_new_path(global_path):
  marker = Marker(type=Marker.LINE_LIST, action=Marker.ADD)
  marker.header.stamp = stamp if stamp is not None else rospy.Time.now()
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.color.b = 1.0
  marker.color.a = 1.0
  marker.pose.position = Point(0,0,0)
  marker.pose.orientation = Quaternion(0,0,0,1)
  
  marker.points = []
  for j in (range(len(global_path)) - 1): 
      marker.points.extend([Point(global_path[j,0], global_path[j,1], 0), 
                            Point(global_path[j+1,0], global_path[j+1,1], 0)])
                       
  marker.lifetime = rospy.Time(5) # 5 sec
  path_pub_.publish(marker)

# --- The callback function --- #
def callback(data):
  global grid_map
  global rows
  global columns
  global counter

  print(counter)

  startTime = datetime.now()

	# get the map and related info
  # this should be a np array
  if (len(data.grid_path) == 0):
    print("empty grid map received")
    return
  grid_map = np.asarray(data.grid_path).reshape(meter_height / scale, meter_height / scale)  
  grid_map = np.flipud(grid_map)
  rows = len(grid_map)
  columns = len(grid_map[0])
  print("grid_map received: ", rows, " rows ", columns, " columns")

  
	# plotting the map # DEBUG
  # if (counter % 10 == 0):
  #   plt.gcf().clear()
  #   plt.ion()
  #   plt.imshow(grid_map, cmap='gray_r')
  #   plt.pause(0.0000001)
  #   plt.show()

  # result_msg = local_rrt_result()

	# get yaw
  euler = euler_from_quaternion((data.current_odometry.pose.pose.orientation.x, 
                                 data.current_odometry.pose.pose.orientation.y,
                                 data.current_odometry.pose.pose.orientation.z,
                                 data.current_odometry.pose.pose.orientation.w))
  yaw = euler[2]

  # get the goal_point
  global_end_point = np.array([data.next_point.x - 0.01, data.next_point.y])
  curr_location_point = np.array([0.0,0.0]).astype(float)
  curr_location_point[0] = data.current_odometry.pose.pose.position.x
  curr_location_point[1] = data.current_odometry.pose.pose.position.y
  vector = (global_end_point - curr_location_point)
  transform_mat = np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
  end_point = np.matmul(transform_mat,vector)

  # pdb.set_trace()
  # detect whether you need to follow local path
  ep_grid = xy_to_grid(end_point)
  if not within_bound(ep_grid):
    ep_grid = np.clip(ep_grid, 0, columns - 1) # assuming that columns = rows
  follow_local_path = False
  out_direction = data.out_direction
  
  # check: if end point is behind start point, use pure pursuit
  if distance(start_point, end_point) <= step_size: # this puts upper limit on step_size. 
    follow_local_path = False
  if out_direction == 0 and end_point[0] < start_point[0]:
    follow_local_path = False
  elif out_direction == -1 and end_point[1] < start_point[1]:
    follow_local_path = False
  elif out_direction == 1 and end_point[1] > start_point[1]:
    follow_local_path = False
  else:
    # decide end_point collision
    # if exit forward
    if out_direction == 0:
      left_grid = xy_to_grid(end_point + np.array([0,buffer]))
      right_grid = xy_to_grid(end_point - np.array([0,buffer]))
      to_adjust = grid_map[ep_grid[0]][ep_grid[1]] > 0
      if within_bound(left_grid):
        to_adjust = to_adjust or grid_map[left_grid[0]][left_grid[1]] > 0
      if within_bound(right_grid):
        to_adjust = to_adjust or grid_map[right_grid[0]][right_grid[1]] > 0
      if (to_adjust):
        # adjust end_point
        col_range = find_max_driveable_col_range(ep_grid[0])
        ep_grid[1] = math.floor((col_range[1] + col_range[0]) / 2)
        end_point[1] = grid_to_xy(ep_grid)[1]
        if distance(start_point, end_point) <= step_size: # this puts upper limit on step_size. 
          follow_local_path = False
        else:
          follow_local_path = True
      else: 
        # if end_point doesn't need to be adjusted, decide whether every grid that the distance-away lines pass is white
        if isValidEdge(start_point, end_point):
          follow_local_path = False
    else:
      front_grid = xy_to_grid(end_point + np.array([buffer, 0]))
      back_grid = xy_to_grid(end_point - np.array([buffer, 0]))
      to_adjust = grid_map[ep_grid[0]][ep_grid[1]] > 0
      if within_bound(front_grid):
        to_adjust = to_adjust or grid_map[front_grid[0]][front_grid[1]] > 0
      if within_bound(back_grid):
        to_adjust = to_adjust or grid_map[back_grid[0]][back_grid[1]] > 0
      if to_adjust:
        # adjust end_point
        row_range = find_max_driveable_row_range(ep_grid[1])
        ep_grid[0] = math.floor((row_range[1] + row_range[0]) / 2)
        end_point[0] = grid_to_xy(ep_grid)[0]
        if distance(start_point, end_point) <= step_size: # this puts upper limit on step_size. 
          follow_local_path = False
        else:
          follow_local_path = True
      else: 
        # if end_point doesn't need to be adjusted, decide whether every grid that the distance-away lines pass is white
        if isValidEdge(start_point, end_point):
          follow_local_path = False
      
  if (follow_local_path):
    print("Follow local path")
    print("lookahead_distance: ", LOOKAHEAD_DISTANCE)
    next_point_local = modified_next_point(LOOKAHEAD_DISTANCE * 1.5, out_direction)
    print("next_point_local: ", next_point_local)
    print("curr_location_point: ", curr_location_point)

    transform_mat = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    next_point_global = np.matmul(transform_mat, next_point_local.reshape(2,-1)) + curr_location_point.reshape(2,-1)
    print("next_point_global: ", next_point_global)

    rst_point = Point()
    rst_point.x = next_point_global[0,0]
    rst_point.y = next_point_global[1,0]
    rst_point_pub.publish(rst_point)

    # local_path = np.empty([0,2])
    # local_path = modified_RRT(start_point, end_point, step_size, out_direction)
    
    # if len(local_path) == 0:
    #   local_path = modified_RRT(end_point, start_point, step_size, out_direction)

    # if len(local_path) > 0:
    #   # transform local path to global path
    #   transform_mat = np.array([[np.cos(-yaw),-np.sin(-yaw)],[np.sin(-yaw),np.cos(-yaw)]])
    #   global_path = np.matmul(transform_mat, np.transpose(local_path)) - curr_location_point.reshape(2,-1)
    #   global_path = global_path.transpose()
    #   endTime = datetime.now()
    #   print(endTime - startTime)

    #   # publish adjusted global path
    #   result_msg.follow_local_path = True
    #   result_msg.global_path_x = global_path[:,0].tolist()
    #   result_msg.global_path_y = global_path[:,1].tolist()
    #   pub.publish(result_msg)
    #   publish_new_path(global_path)

    #   # print out info for debug
    #   path_len = len(global_path)
    #   print ("ATTENTION!USING LOCAL PATH!!!")
    #   print("out direction: ", out_direction)
    #   print("counter: ", counter)
    #   print("start point: ", curr_location_point)
    #   print("global end point: ", global_end_point)
    #   print("curr location point: ", curr_location_point)
    #   print("original end point: ", global_end_point - curr_location_point)
    #   print("adjusted end point: ", end_point)
    #   print("local_path: ", local_path)
    #   print("path len: ", path_len)
    #   # print("PLOTTING---")
    #   # displayMapAndPath(local_path, grid_map)
      
    # else:
    #   endTime = datetime.now()
    #   print(endTime - startTime)
    #   print("can't find local RRT path. use pure pursuit")
    #   print ("ATTENTION!FAILED USING LOCAL PATH!!!")
    #   print("out direction: ", out_direction)
    #   print("counter: ", counter)
    #   print("start point: ", curr_location_point)
    #   print("global end point: ", global_end_point)
    #   print("curr location point: ", curr_location_point)
    #   print("original end point: ", global_end_point - curr_location_point)
    #   print("adjusted end point: ", end_point)
    #   result_msg.follow_local_path = False

  else:
    # result_msg.follow_local_path = False
    print("use pure pursuit")

  # timing
  # time_difference_in_ms = (endTime - startTime) / timedelta(milliseconds=1)

def drive_param_callback(data):
  global LOOKAHEAD_DISTANCE
  velocity = data.velocity
  LOOKAHEAD_DISTANCE = np.abs(velocity - VELOCITY_BASE)*1.5/MAX_VELOCITY + 0.8
  # print(velocity)

if __name__ == '__main__':
  rospy.init_node('local_planner_simplified')
  # TODO: change to the topic and message published by occupancy grid
  rospy.Subscriber('/grid', OccupancyGrid, callback, queue_size=1) 
  rospy.Subscriber('/drive_parameters', drive_param, drive_param_callback, queue_size=1)
  rospy.spin()