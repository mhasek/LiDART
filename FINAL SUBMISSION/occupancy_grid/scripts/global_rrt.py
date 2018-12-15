#!/usr/bin/env python

import re
import numpy as np
import pdb
import random
import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import pylab as pl
import copy
import rospy
import os
import contextlib
from numpy.linalg import *
import scipy as sp
from scipy import ndimage
from scipy import io
from scipy.signal import savgol_filter
from scipy import interpolate


# filepath of .pgm file
filePath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'maps/racemap.pgm'))
name = "race_pts"

# -- GLOBAL VARIABLES --
step_size = 0.1
buffer = 0.3
scale = 0.05 # given in the cartographer yaml file

# -- HELPER CLASSES --

@contextlib.contextmanager
def escapable():
    class Escape(RuntimeError): pass
    class Unblock(object):
        def escape(self):
            raise Escape()

    try:
        yield Unblock()
    except Escape:
        pass

class Map(object):
  
  def __init__(self, filepath, buffer, scale, stepSize):
    # filepath is the location of the map pgm
    # buffer is the distance (in real world scale) that edges must stay away from walls
    # scale is real world distance between adjacent pixels
    # stepSize is the real world distance of each edge of the RRT
    self.buffer = buffer
    self.scale = scale
    self.stepSize = stepSize
    self.freePixels = self.loadMap(filepath)
    self.sampleablePixels = copy.deepcopy(self.freePixels)
    self.driveablePixels = self.getDriveablePixels()
    
    
  def loadMap(self, filepath, byteorder='>'):
    freePixels = set()
    with open(filepath, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    pixelValues = np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))
    self.image = pixelValues

    # plotting script for debug
    # mpl.pyplot.figure(figsize=(20,10))
    # mpl.pyplot.matshow(pixelValues, fignum=1)
    # mpl.pyplot.show()

    self.height = int(height)
    self.width = int(width)
    self.startLine = False # the start line must be added by the createStartLine function
    for h in range(int(height)):
      for w in range(int(width)):
        if (pixelValues[h, w] > 0):
          freePixels.add((h, w))
          # print("pixel ", (h, w), " is coordinate ", (h*self.scale, w*self.scale), " and is driveable")
    return freePixels

  def resetMap(self, newStepSize):
    self.sampleablePixels = copy.deepcopy(self.freePixels)
    self.stepSize = newStepSize
  
  
  def getDriveablePixels(self):
    knownDriveables = set()
    newImage = np.copy(self.image)
    for point in self.freePixels:
      topEdge = (point[0] == self.height - 1)
      bottomEdge = (point[0] == 0)
      leftEdge = (point[1] == 0)
      rightEdge = (point[1] == self.width - 1)
      
      up_driveable = (point[0] + 1, point[1]) in knownDriveables
      down_driveable = (point[0] - 1, point[1]) in knownDriveables
      left_driveable = (point[0], point[1] - 1) in knownDriveables
      right_driveable = (point[0], point[1] + 1) in knownDriveables
      
      # if all 4 neighbors are drivable so is this
      if ((up_driveable or topEdge) and (down_driveable or bottomEdge) and (left_driveable or leftEdge) and (right_driveable or rightEdge)):
          knownDriveables.add((point))
          newImage[point[0]][point[1]] = 100
          # print(point[0] * self.scale, ", ", point[1] * self.scale, "is driveable from pixel ", point[0], ", ", point[1])
          continue 
      
      # if none of the neighbors are drivable then check all points
      if not (False):#up_driveable or down_driveable or left_driveable or right_driveable):
          with escapable() as a:
              for h in range(int(max(0, math.ceil(point[0] - self.buffer/self.scale))), int(min(self.height, math.floor(point[0] + self.buffer/self.scale + 1)))):
                  c_squared = (self.buffer/self.scale)**2
                  a_squared = (h - point[0])**2
                  b = math.sqrt(max(0, c_squared - a_squared))
                  for w in range(int(max(0, math.ceil(point[1] - b))), int(min(self.width, math.floor(point[1] + b + 1)))):
                      #print("testing ", (h, w), " for ", point)
                      if (not (h, w) in self.freePixels):
                          a.escape()
              knownDriveables.add((point))
              newImage[point[0]][point[1]] = 100
          continue
          
      # if one of the neighbors is drivable then just check non-overlap
      if (left_driveable):
        with escapable() as f:
          for h in range(int(max(0, math.ceil(point[0] - self.buffer/self.scale))), int(min(self.height, math.floor(point[0] + self.buffer/self.scale + 1)))):
            c_squared = (self.buffer/self.scale)**2
            a_squared = (h - point[0])**2
            b = math.sqrt(c_squared - a_squared)
            w = math.floor(point[1] + b)
            if (w > self.width): continue
            if (not (h, w) in self.freePixels):
              f.escape()
          knownDriveables.add((point))
          newImage[point[0]][point[1]] = 100
        continue
      if (right_driveable):
        with escapable() as c:
          for h in range(int(max(0, math.ceil(point[0] - self.buffer/self.scale))), int(min(self.height, math.floor(point[0] + self.buffer/self.scale + 1)))):
            c_squared = (self.buffer/self.scale)**2
            a_squared = (h - point[0])**2
            b = math.sqrt(c_squared - a_squared)
            w = math.ceil(point[1] - b)
            if (w < 0): continue
            if (not (h, w) in self.freePixels):
              c.escape()
          knownDriveables.add((point))
          newImage[point[0]][point[1]] = 100
        continue
      if (up_driveable):
        with escapable() as d:
          for w in range(int(max(0, math.ceil(point[1] - self.buffer/self.scale))), int(min(self.width, math.floor(point[1] + self.buffer/self.scale + 1)))):
            c_squared = (self.buffer/self.scale)**2
            a_squared = (w - point[1])**2
            b = math.sqrt(c_squared - a_squared)
            h = math.floor(point[0] + b)
            if (h > self.height): continue
            if (not (h, w) in self.freePixels):
              d.escape()
          knownDriveables.add((point))
          newImage[point[0]][point[1]] = 100
        continue
      if (down_driveable):
        with escapable() as e:
          for w in range(int(max(0, math.ceil(point[1] - self.buffer/self.scale))), int(min(self.width, math.floor(point[1] + self.buffer/self.scale + 1)))):
            c_squared = (self.buffer/self.scale)**2
            a_squared = (w - point[1])**2
            b = math.sqrt(c_squared - a_squared)
            h = math.ceil(point[0] - b)
            if (h < 0): continue
            if (not (h, w) in self.freePixels):
              e.escape()
          knownDriveables.add((point))
          newImage[point[0]][point[1]] = 100
        continue
        
        print("should not get here")
    
    # plotting script for debug
    # the result plot is: x-horizontal, y-vertical
    # mpl.pyplot.figure(figsize=(20,10))
    # mpl.pyplot.matshow(newImage, fignum=1)
    # mpl.pyplot.plot([175, 211], [200,200], 'go')
    # mpl.pyplot.show()
    return knownDriveables
  
  # makes a start line between point 1 and point 2
  # does not allow any edge to cross the start line
  def createStartLine(self, point1, point2):
    self.startLine = True
    self.startPoint1 = point1
    self.startPoint2 = point2
    
    
  # takes in the X and Y of the last point added to the tree
  # updates sampleable pixels based on that point so that the next point
  # is an adequate distance from the tree
  # point is np.array(2)
  def sampleRandomFreePoint(self, lastNewPoint):
    #print("last new point ", lastNewPoint)
    pixelRadius = self.stepSize/self.scale
    for h in range(int(max(0, math.ceil(lastNewPoint[0]/self.scale - pixelRadius))), int(min(self.height, math.floor(lastNewPoint[0]/self.scale + pixelRadius + 1)))):
      c_squared = pixelRadius**2
      a_squared = (h - lastNewPoint[0]/self.scale)**2
      b = math.sqrt(c_squared - a_squared)
      for w in range(int(max(0, math.ceil(lastNewPoint[1]/self.scale - b))), int(min(self.width, math.floor(lastNewPoint[1]/self.scale + b + 1)))):
        self.sampleablePixels.discard((h, w))
        #print("discard ", (h, w))
    #print(len(self.sampleablePixels), " sampleable pixels left")
    pixel = random.sample(self.sampleablePixels, 1)[0]
    return [pixel[0] * self.scale, pixel[1] * self.scale]
    
  # helper function for line intersection
  def ccw(self,A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
  
  # return true if line segments AB and CD intersect
  # e.g. AB is the start line and CD is an edge candidate
  def intersect(self, A,B,C,D):
    return self.ccw(A,C,D) != self.ccw(B,C,D) and self.ccw(A,B,C) != self.ccw(A,B,D)
    
  def isValidEdge(self, point1, point2):
    # print("check edge ", point1, " ", point2)
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    
    if (self.startLine):
      if (self.intersect(self.startPoint1, self.startPoint2, point1, point2)):
        #print("edge candidate intersects the start line")
        #print(self.startPoint1)
        #print(self.startPoint2)
        #print(point1)
        #print(point2)
        return False
      
    
    if (abs(dx) > abs(dy)):
      # increment h checking the w window the path passes through
      if (point1[0] < point2[0]):
        # increment from point 1 to point 2
        m = dy / dx
        for h in range(int(math.floor(point1[0] / self.scale)), int(math.ceil(point2[0] / self.scale))):
          # print("check h = ", h)
          w_low = math.floor((h - math.floor(point1[0] / self.scale)) * m + point1[1] / self.scale)
          w_high = math.ceil((h - math.floor(point1[0] / self.scale)) * m + point1[1] / self.scale)
          if not ((h, w_low) in self.driveablePixels):
            # print((h, w_low), " is not driveable")
            return False
          if not ((h, w_high) in self.driveablePixels):
            # print((h, w_low), " is not driveable")
            return False
      else:
        # increment from point 2 to point 1
        m = - dy / dx
        for h in range(int(math.floor(point2[0] / self.scale)), int(math.ceil(point1[0] / self.scale))):
          # print("check h = ", h)
          w_low = math.floor((h - math.floor(point2[0] / self.scale)) * m + point2[1] / self.scale)
          w_high = math.ceil((h - math.floor(point2[0] / self.scale)) * m + point2[1] / self.scale)
          if not ((h, w_low) in self.driveablePixels):
            # print((h, w_low), " is not driveable")
            return False
          if not ((h, w_high) in self.driveablePixels):
            # print((h, w_low), " is not driveable")
            return False
          
    else:
      # increment w checking the h window the path passes through
      if (point1[1] < point2[1]):
        # increment from point 1 to point 2
        m = dx / dy
        for w in range(int(math.floor(point1[1] / self.scale)), int(math.ceil(point2[1] / self.scale))):
          # print("check w = ", w)
          h_low = math.floor((w - math.floor(point1[1] / self.scale)) * m + point1[0] / self.scale)
          h_high = math.ceil((w - math.floor(point1[1] / self.scale)) * m + point1[0] / self.scale)
          if not ((h_low, w) in self.driveablePixels):
            # print((h_low, w), " is not driveable")
            return False
          if not ((h_high, w) in self.driveablePixels):
            # print((h_high, w), " is not driveable")
            return False
      else:
        # increment from point 2 to point 1
        m = - dx / dy
        for w in range(int(math.floor(point2[1] / self.scale)), int(math.ceil(point1[1] / self.scale))):
          # print("check w = ", w)
          h_low = math.floor((w - math.floor(point2[1] / self.scale)) * m + point2[0] / self.scale)
          h_high = math.ceil((w - math.floor(point2[1] / self.scale)) * m + point2[0] / self.scale)
          if not ((h_low, w) in self.driveablePixels):
            # print((h_low, w), " is not driveable")
            return False
          if not ((h_high, w) in self.driveablePixels):
            # print((h_high, w), " is not driveable")
            return False
         
    # print("Driveable")
    return True
  
  def displayMapAndPath(self, path):
    pixelValues = self.image
    print_path = path / self.scale
    print(print_path)

    mpl.pyplot.figure(figsize=(20,10))
    mpl.pyplot.matshow(pixelValues)
    mpl.pyplot.plot(print_path[:,1], print_path[:,0])
    mpl.pyplot.show()


class Tree(object):
  
  def __init__(self, k, root):
    self.points = np.zeros([2, k], dtype = float)
    self.vertexCount = 1
    self.points[0, 0] = root[0]
    self.points[1, 0] = root[1]
    self.parents = np.zeros(k, dtype = int)
    self.parents[0] = -1
    
  # Void
  def addPointToTree(self, nextPoint, parentIndex):
    self.points[0, self.vertexCount] = nextPoint[0]
    self.points[1, self.vertexCount] = nextPoint[1]
    self.parents[self.vertexCount] = parentIndex
    self.vertexCount = self.vertexCount + 1
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
    pl.show()
    
  def getPathFromLastVertex(self):
    path = self.points[:, self.vertexCount - 1]
    index = self.parents[self.vertexCount - 1]
    
    while (index != -1):
      path = np.vstack([path, self.points[:, index]])
      index = self.parents[index]
    
    # print(path)
    return path

# -- HELPER FUNCTIONS --

# RRT(startPoint, map, step_size)
# what it does: main function to create a RRT on the map starting from the startPoint
# input: startPoint, map, step_size
# output: tree
def RRT(startPoint, map, step_size):
  # change the line here to initialize the maximum possible tree size k.
  k = 20000 # hardcoded
  root = [startPoint[0], startPoint[1] + step_size]
  endPoint = [startPoint[0], startPoint[1] - 2 * step_size]
  tree = Tree(k, root)
  lastNewPoint = root
  i = 0
  while not checkEndingCondition(endPoint, tree, step_size):
    i = i + 1
    randomPoint = map.sampleRandomFreePoint(lastNewPoint)
    # print("random point ", randomPoint)
    nearestTriple = tree.findNearestPoint(randomPoint)
    nearestPoint = nearestTriple[:2]
    if (distance(nearestPoint, randomPoint) < step_size):
      # print(randomPoint, " is not a whole step away from ", nearestPoint)
      continue
    nextPoint = getNextPoint(nearestPoint, randomPoint, step_size)
    # print("next point ", nextPoint)
    if (isValidEdge(nearestPoint, nextPoint, map)):
      tree.addPointToTree(nextPoint, nearestTriple[2])
      lastNewPoint = nextPoint
      print(nextPoint)
    #else:
      #print(nearestPoint, " ", nextPoint, " is not a valid edge")
    #if (i % 400 == 0):
    #  tree.displayTree()
  # tree.displayTree()
  return tree

# randomFreePoint(map)
# what it does: generate a random point that is on the track (aka not in obstacles)
# input: map - matrix
# output: a point (x,y)
def randomFreePoint(map): # -> map.sampleRandomFreePoint(lastPoint, ), returns a np array for point
  return map.sampleRandomFreePoint() 

# distance(point1, point2)
# what it does: find the distance between 2 points (x,y)
# input: 2 points
# output: float distance
def distance(point1, point2):
  return math.sqrt(np.sum((point1 - point2)**2))

# getNextPoint(nearestPoint, randomPoint, step_size)
# what it does: finds the point that is step_size away from nearestPoint on the segment between nearestPoint & randomPoint
# input: nearestPoint, randomPoint
# output: a point (x,y)
def getNextPoint(nearestPoint, randomPoint, step_size):
  nextPoint = np.zeros(2)
  diff = (randomPoint - nearestPoint) / distance(nearestPoint, randomPoint) * step_size
  # TODO: add a line here to limit the angle between the (nearestPoint.parent,nearestPoint) 
  # and (nearestPoint,nextPoint) within maximum turning angle
  nextPoint = nearestPoint + diff
  return nextPoint

# isValidEdge(nearestPoint, nextPoint, map) @Nikhil @Connor <- map function
# what it does: draw a line between nearestPoint & nextPoint. Decide whether the cloest point on line to 
# the boundary is too close to the boundary 
# input: nearestPoint, nextPoint
# output: true / false
def isValidEdge(nearestPoint, nextPoint, map):
  return map.isValidEdge(nearestPoint, nextPoint)

# addStartingLine(startPoint, map)
# what it does: add a wall to the map next to the starting point (this is hardcoded)
# input: startPoint, map
# output: map
def addStartingLine(startPoint, map):
  # TODO: code to get the two ends of the wall, can be hardcoded
  wallEndPoints = np.array([0,0]) # hardcoded!
  map.addStartingLine(wallEndPoints)
  return map

# checkEndingCondition(startPoint, tree, step_size)
# what it does: check the point that is closest to the endPoint on the tree and see whether it's within step_size of the endPoint 
# input: endPoint (pre-picked to be within step-size of startPoint), tree, step_size
# output: true / false
def checkEndingCondition(endPoint, tree, step_size):
  nearestTriple = tree.findNearestPoint(endPoint)
  nearestPoint = nearestTriple[:2]
  return distance(nearestPoint, endPoint) <= step_size * 2

# -- HELPER FUNCTION TO PRUNE WAYPOINTS --
def pathInterp(path,d):
  Interp = path[0,:].reshape(1,2)
  p0 = path[0,:]

  for i in np.arange(1,len(path)):
    p1 = path[i,:]
    dist = norm(p1-p0)
    pointCount = dist/d
    unitvec = (p1-p0)/pointCount
    
    x = np.linspace(p0[0]+unitvec[0],p1[0],int(pointCount))
    y = np.linspace(p0[1]+unitvec[0],p1[1],int(pointCount))
    tempPath = np.hstack((x.reshape(-1,1), y.reshape(-1,1)))
    Interp = np.vstack((Interp, tempPath));
    p0 = p1;

  Interp[-1] = Interp[0]
  return Interp


def dist2line(pt, v1, v2):

  pt = np.hstack((pt, np.zeros((len(pt),1))))
  v1 = np.hstack((v1, np.zeros((1,1))))
  v2 = np.hstack((v2, np.zeros((1,1))))
  a = v1 - v2
  b = pt - v2


  alpha = np.sum(a*b,axis=1)/np.sum(a*a)

  proj = alpha.reshape(-1,1)*a

  check = (norm(proj,axis=1) < norm(a)) & (alpha>0)

  d = norm(np.cross(a,b,axis = 1),axis=1)/norm(a)


  if np.sum(check) == 0:
    return 0
  else:
    return np.min(d[check])


def PruneWayPoints(filePath,res,path,d):
  map = plt.imread(str(filePath))
  (r,c) = np.where(map==0)
  r = res[0] * r 
  c = res[1] * c

  id = 1
  sparsePath = path[0,:]
  lastpts = path[0,:]

  while id < len(path)-1:

    count = 0;
    for j in np.arange(id,len(path),1):

      x = np.append(lastpts[1],path[j,1])*20

      y = np.append(lastpts[0],path[j,0])*20

      num = np.ceil(max(abs(np.diff(x)),abs(np.diff(y)))) 

      xq = np.int32(np.linspace(x[0], x[1],np.int32(num*3)))
      yq = np.int32(np.linspace(y[0], y[1],np.int32(num*3)))

      val = map[yq,xq]

      dist = dist2line(np.array((r,c)).T,lastpts.reshape(1,2),path[j,:].reshape(1,2))
      
      cutswall = ~(np.sum(val == 255) == len(val))
      drivable = ~cutswall and (dist > d)
      
      if drivable:
        id  = j

      elif cutswall:
        break

    lastpts = path[id,:];
    id = id + 1
    print id
    sparsePath = np.vstack((sparsePath, lastpts))

  sparsePath[0,:] = lastpts
  return sparsePath


def TransformPoints(path,im_center):
  fp = np.hstack((path[:,1].reshape(-1,1),path[:,0].reshape(-1,1))) + im_center 
  fp = np.hstack((fp[:,0].reshape(-1,1), -fp[:,1].reshape(-1,1)))
  return fp



# -- MAIN SCRIPT --
if __name__ == '__main__':
  # load map
  map = Map(filePath, buffer, scale, step_size)

  # TODO: get the wall points from matplotlib UI
  # must be vertical
  startPoint = np.array([270, 155]) * scale
  startWall1 = [240 * scale, 155 * scale]
  startWall2 = [300 * scale, 155 * scale]

  # startPoint = np.array([51, 51])
  # startWall1 = [47, 51]
  # startWall2 = [55, 51]

  map.resetMap(step_size)
  map.createStartLine(startWall1, startWall2)
  tree = RRT(startPoint, map, step_size)
  rrt_path = np.vstack([tree.getPathFromLastVertex(), startPoint])
  print(rrt_path)
  # map.displayMapAndPath(rrt_path)
  plt.close("all")


  np.savetxt("rrt_path.csv", rrt_path,delimiter = ",")

  rrt_path  = np.genfromtxt('rrt_path.csv',delimiter=',')

  #prune waypoint -- this part is not working yet --
  res = np.array([0.05,0.05])
  d = 0.6
  sparsePath = PruneWayPoints(filePath, res, rrt_path, d)

  sparsePath[0,:] = sparsePath[-1,:]

  tck,u = interpolate.splprep(sparsePath.T,k=1,s=0)
  unew = np.arange(0, 1.01, 0.01)
  out = interpolate.splev(unew, tck)

  sparsePath = PruneWayPoints(filePath, res, np.hstack((out[0].reshape(-1,1),out[1].reshape(-1,1))), 0.4)

  sparsePath[0,:] = sparsePath[-1,:]

  sparsePath = pathInterp(sparsePath,0.3)

  tck,u = interpolate.splprep(sparsePath.T,k=2,s=1)
  unew = np.arange(0, 1.01, 0.01)
  out = interpolate.splev(unew, tck)

  path = np.hstack((out[0].reshape(-1,1),out[1].reshape(-1,1)))

  im_center = np.array([[-7.969707,-4.438220]])
  
  out_path = TransformPoints(path, im_center)


  rootPath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../'))

  np.savetxt(os.path.abspath(os.path.join(rootPath, "lidart_pure_pursuit/scripts/waypoints/" + name +".csv")), out_path,delimiter = ",")
  np.savetxt(os.path.abspath(os.path.join(rootPath,"occupancy_grid/waypoints/" + name + ".csv")), out_path,delimiter = ",")

  print(path)

  plt.figure()
  plt.imshow(map.image)
  plt.hold(True)
  plt.plot(path[:,1]*20,path[:,0]*20)
  plt.plot(rrt_path[:,1]*20,rrt_path[:,0]*20)
  plt.show()
