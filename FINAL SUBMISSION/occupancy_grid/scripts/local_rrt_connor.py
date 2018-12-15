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
from geometry_msgs.msg import Point
import copy
import pylab as pl

PUB = rospy.Publisher('local_rrt_result', local_rrt_result, queue_size=1)
BUFFER = 0.3 # a little more than half width of the car, in meters
NEIGHBORHOOD_LENGTH = 3.0
NUM_BOXES = 24
STEP_SIZE = 0.5
K = 100
counter = 0

class LocalMap(object):
  
  def __init__(self, local_grid, root_global, theta, end_global):
    self.local_grid = local_grid
    self.x = root_global[0]
    self.y = root_global[1]
    self.theta = theta
    self.end_x = end_global[0]
    self.end_y = end_global[1]
    self.free_pixels = self.getFreePixels()
    self.sampleable_pixels = copy.deepcopy(self.free_pixels)
    
    
  def toXY(self, h, w):
    global NUM_BOXES
    global NEIGHBORHOOD_LENGTH
    x = self.x + math.sin(self.theta) * (w - NUM_BOXES/2 + 0.5) * NEIGHBORHOOD_LENGTH / NUM_BOXES + math.cos(self.theta) * (h + 0.5) * NEIGHBORHOOD_LENGTH / NUM_BOXES
    y = self.y - math.cos(self.theta) * (w - NUM_BOXES/2 + 0.5) * NEIGHBORHOOD_LENGTH / NUM_BOXES + math.sin(self.theta) * (h + 0.5) * NEIGHBORHOOD_LENGTH / NUM_BOXES
    return [x, y]
    
    
  def toHW(self, x, y):
    global NUM_BOXES
    global NEIGHBORHOOD_LENGTH
    dx = x - self.x
    dy = y - self.y
    
    h_distance = dx * math.cos(self.theta) + dy * math.sin(self.theta) # -2
    w_distance = -dy * math.cos(self.theta) + dx * math.sin(self.theta) # 0
    
    h_float = h_distance * NUM_BOXES / NEIGHBORHOOD_LENGTH # -16
    w_float = w_distance * NUM_BOXES / NEIGHBORHOOD_LENGTH + NUM_BOXES / 2 # 0
    
    h = int(math.floor(h_float))
    w = int(math.floor(w_float))

    return [h, w]
    
    
  def getFreePixels(self):
    free_pixels = set()
    for h in range(int(len(self.local_grid))):
      for w in range(int(len(self.local_grid))):
        if (self.local_grid[h, w] == 0):
          free_pixels.add((h, w))
    return free_pixels
  
  
  def sampleRandomFreePoint(self, last_new_point):
    last_new_pixel = self.toHW(last_new_point[0], last_new_point[1])
    self.sampleable_pixels.discard((last_new_pixel[0], last_new_pixel[1]))
    new_pixel = random.sample(self.free_pixels, 1)[0]
    #print("There are now ", len(self.sampleable_pixels), " pixels left to sample")
    #print("Sampled ", self.toXY(new_pixel[0], new_pixel[1]))
    return self.toXY(new_pixel[0], new_pixel[1])
  
  
  def isValidEdge(self, point1, point2, print_statements=False):
    if (print_statements):
      print("LETS CHECK ", point1, point2)
    global NUM_BOXES
    pixel1 = self.toHW(point1[0], point1[1])
    pixel2 = self.toHW(point2[0], point2[1])
    
    dh = pixel1[0] - pixel2[0]
    dw = pixel1[1] - pixel2[1]
    
    if (print_statements):
      print("dh", dh)
      print("dw", dw)
      print(self.local_grid)

    if (dh + dw == 0):
      return not self.local_grid[pixel1[0], pixel1[1]]
      
    
    if (abs(dh) > abs(dw)):
      # increment h checking the w window the path passes through
      if (pixel1[0] < pixel2[0]):
        # increment from point 1 to point 2
        m = (dw * 1.0) / dh
        #print("Slope is ", m)
        for h in range(int(pixel1[0]), int(pixel2[0])):
          w_low = int(min(math.floor((h - pixel1[0]) * m + pixel1[1]), NUM_BOXES-1))
          w_high = int(min(math.ceil((h - pixel1[0]) * m + pixel1[1]), NUM_BOXES-1))
          if (self.local_grid[h, w_low]):
            if (print_statements):
              print("acan't drive at ", h, w_low)
            return False
          if (self.local_grid[h, w_high]):
            if (print_statements):
              print("bcan't drive at ", h, w_high)
            return False
      else:
        # increment from point 2 to point 1
        m = - (dw * 1.0) / dh
        for h in range(int(pixel2[0]), int(pixel1[0])):
          w_low = int(min(math.floor((h - pixel2[0]) * m + pixel2[1]), NUM_BOXES-1))
          w_high = int(min(math.ceil((h - pixel2[0]) * m + pixel2[1]), NUM_BOXES-1))
          if (self.local_grid[h, w_low]):
            if (print_statements):
              print("ccan't drive at ", h, w_low)
            return False
          if (self.local_grid[h, w_high]):
            if (print_statements):
              print("dcan't drive at ", h, w_high)
            return False
          
    else:
      # increment w checking the h window the path passes through
      if (pixel1[1] < pixel2[1]):
        # increment from point 1 to point 2
        m = dh / (dw * 1.0) 
        for w in range(int(pixel1[1]), int(pixel2[1])):
          h_low = int(min(math.floor((w - pixel1[1]) * m + pixel1[0]), NUM_BOXES-1))
          h_high = int(min(math.ceil((w - pixel1[1]) * m + pixel1[0]), NUM_BOXES-1))
          if (self.local_grid[h_low, w]):
            if (print_statements):
              print("ecan't drive at ", h_low, w)
            return False
          if (self.local_grid[h_high, w]):
            if (print_statements):
              print("fcan't drive at ", h_high, w)
            return False
      else:
        # increment from point 2 to point 1
        m = - dh / (dw * 1.0) 
        for w in range(int(pixel2[1]), int(pixel1[1])):
          h_low = int(min(math.floor((w - pixel2[1]) * m + pixel2[0]), NUM_BOXES-1))
          h_high = int(min(math.ceil((w - pixel2[1]) * m + pixel2[0]), NUM_BOXES-1))
          if (self.local_grid[h_low, w]):
            if (print_statements):
              print("gcan't drive at ", h_low, w)
            return False
          if (self.local_grid[h_high, w]):
            if (print_statements):
              print("hcan't drive at ", h_high, w)
            return False
    if (print_statements):
      print("CAN DRIVE ", point1, point2)
    return True


class Tree(object):
  
  def __init__(self, k, root):
    self.points = np.zeros([2, k], dtype = float)
    self.vertexCount = 1
    self.points[0, 0] = root[0]
    self.points[1, 0] = root[1]
    self.parents = np.zeros(k, dtype = int)
    self.parents[0] = -1
    self.max_iters = k
    
  # Void
  def addPointToTree(self, nextPoint, parentIndex):
    self.points[0, self.vertexCount] = nextPoint[0]
    self.points[1, self.vertexCount] = nextPoint[1]
    self.parents[self.vertexCount] = parentIndex
    self.vertexCount = self.vertexCount + 1
    #print("The tree now has ", self.vertexCount, " points")
    #self.displayTree()
    return self
    
  # Returns triple: (nearestPointX, nearestPointY, nearestPointIndex)
  def findNearestPoint(self, randomPoint):
    dx = self.points[0, 0:self.vertexCount] - randomPoint[0]
    dy = self.points[1, 0:self.vertexCount] - randomPoint[1]
    d_squared = np.square(dx) + np.square(dy)
    index = np.argmin(d_squared)
    return np.array([self.points[0, index], self.points[1, index], index])
  
  # displays the tree # not tested
  def displayTree(self):
    edges = []
    for i in range(1, self.vertexCount):
      point = (self.points[0, i], self.points[1, i])
      parent = (self.points[0, self.parents[i]], self.points[1, self.parents[i]])
      edge = [point, parent]
      edges.append(edge)
    edgeCollection = mpl.collections.LineCollection(edges)
    fig, ax = pl.subplots()
    ax.add_collection(edgeCollection)
    ax.autoscale()
    ax.margins(0.1)
    
  def getPathFromLastVertex(self):
    path = self.points[:, self.vertexCount - 1]
    index = self.parents[self.vertexCount - 1]
    
    while (index != -1):
      path = np.vstack([path, self.points[:, index]])
      index = self.parents[index]
    
    return path

  def getPath(self, end_point):
    dx = self.points[0, 0:self.vertexCount] - end_point[0]
    dy = self.points[1, 0:self.vertexCount] - end_point[1]
    d = np.sqrt(np.square(dx) + np.square(dy))
    i = np.argmin(d)
    path = self.points[:, i]
    index = self.parents[i]

    while (index != -1):
      path = np.vstack([path, self.points[:, index]])
      index = self.parents[index]
    
    return path
  
  def checkEndingCondition(self, end_point, step_size, iterations):
    dx = self.points[0, 0:self.vertexCount] - end_point[0]
    dy = self.points[1, 0:self.vertexCount] - end_point[1]
    d = np.sqrt(np.square(dx) + np.square(dy))
    return np.min(d) < step_size * 2 or iterations > self.max_iters



class LocalRRT(object):
  
  def __init__(self, occupancy_grid, iters, root_point, theta, endpoint):
    
    # use the buffer and scale to edit the occupancy grid
    self.buffered_occupancy_grid = occupancy_grid
    self.endpoint = endpoint
    self.local_map = LocalMap(occupancy_grid, root_point, theta, endpoint)
    self.local_tree = Tree(iters, root_point)
    self.root = root_point
    
    
  def distance(self, point1, point2):
    return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
  
  
  def getNextPoint(self, nearest_point, target_point, step_size):
    d_point = target_point - nearest_point
    distance = math.sqrt(d_point[0]**2 + d_point[1]**2)
    return nearest_point + d_point / distance * step_size
    
    
  def runRRT(self, step_size, k):
    last_new_point = self.local_map.toXY(self.root[0], self.root[1])
    last_index = -1
    iters = 0
    while not self.local_tree.checkEndingCondition([self.local_map.end_x, self.local_map.end_y], step_size, iters):
      iters = iters + 1
      random_point = self.local_map.sampleRandomFreePoint(last_new_point)
      nearest_triple = self.local_tree.findNearestPoint(random_point)
      nearest_point = nearest_triple[:2]
    
      if (self.distance(nearest_point, random_point) < step_size):
        #print("It's too close to ", nearest_point)
        continue
        
      next_point = self.getNextPoint(nearest_point, random_point, step_size)
      
      if (self.local_map.isValidEdge(nearest_point, next_point)):
        self.local_tree.addPointToTree(next_point, nearest_triple[2])
        last_new_point = next_point
        last_index = nearest_triple[2]
    
    self.local_tree.addPointToTree(self.endpoint, last_index)
    return self.local_tree.getPath(self.endpoint)


def bufferizeManhattan(array, buffer):
  if (buffer == 0): return array
  pa = np.pad(array, ((1, 1), (1, 1)), 'constant', constant_values=(0))
  added_array = pa + np.roll(pa, 1, axis=0) + np.roll(pa, -1, axis=0) + np.roll(pa, 1, axis=1) + np.roll(pa, -1, axis=1)
  buffered_array = added_array[1:-1,1:-1]
  return bufferizeManhattan(np.where(buffered_array>0,1,0), buffer-1)


# --- The callback function on occupancy grid --- #
def callback(data):
  global NEIGHBORHOOD_LENGTH
  global NUM_BOXES
  global STEP_SIZE
  global PUB
  global K
  global BUFFER

  startTime = datetime.now()
  boxes_to_buffer = int(math.ceil(BUFFER / NEIGHBORHOOD_LENGTH * NUM_BOXES))

  #print("LOCAL RRT IS CALLING BACK REGARDING OCCUPANCY")
  #print(np.asarray(data.grid_path).shape)
  grid_map = np.asarray(data.grid_path).reshape(NUM_BOXES, NUM_BOXES)  
  grid_map = np.flipud(grid_map)
  buffered_map = bufferizeManhattan(grid_map, boxes_to_buffer)



  x = data.current_odometry.pose.pose.position.x
  y = data.current_odometry.pose.pose.position.y
  euler = euler_from_quaternion((data.current_odometry.pose.pose.orientation.x, 
                                 data.current_odometry.pose.pose.orientation.y,
                                 data.current_odometry.pose.pose.orientation.z,
                                 data.current_odometry.pose.pose.orientation.w))
  yaw = euler[2] 
  #print("let's make the local rrt")
  localRRT = LocalRRT(buffered_map.astype(int), K, [x, y], yaw, [data.next_point.x, data.next_point.y])
  localRRT_no_buffer = LocalRRT(grid_map.astype(int), K, [x, y], yaw, [data.next_point.x, data.next_point.y])
  #print("do we need to run local rrt?")
  if not (localRRT_no_buffer.local_map.isValidEdge([x, y], [data.next_point.x, data.next_point.y], print_statements=False)):
    # print("let's run the local rrt")
    local_path = localRRT.runRRT(STEP_SIZE, 100).reshape((-1, 2))
    
    while (len(local_path) < 2):
      # print("THIS PATH IS TOO SHORT")
      break
      boxes_to_buffer = boxes_to_buffer - 1
      if (boxes_to_buffer < 0):
        # print("COULD NOT FIND A PATH")
        break
      buffered_map = bufferizeManhattan(grid_map, boxes_to_buffer)
      localRRT = LocalRRT(buffered_map.astype(int), K, [x, y], yaw, [data.next_point.x, data.next_point.y])
      local_path = localRRT.runRRT(STEP_SIZE, 100).reshape((-1, 2))
    

    # Send this update
    result_msg = local_rrt_result()
    result_msg.follow_local_path = True
    local_path = np.flipud(local_path)
    result_msg.global_path_x = local_path[:,0].tolist()
    result_msg.global_path_y = local_path[:,1].tolist()
    result_msg.next_point = data.next_point
    #print("NEW LOCAL PATH")
    #print(local_path)

    PUB.publish(result_msg)
    return
  else:
    result_msg = local_rrt_result()
    result_msg.follow_local_path = False
    PUB.publish(result_msg)
    print("No need to update")
  

if __name__ == '__main__':
  print("Hi I'm in Connor's Local RRT node")
  rospy.init_node('local_rrt_node')
  # TODO: change to the topic and message published by occupancy grid
  rospy.Subscriber('/grid', OccupancyGrid, callback, queue_size=10) 
  rospy.spin()
