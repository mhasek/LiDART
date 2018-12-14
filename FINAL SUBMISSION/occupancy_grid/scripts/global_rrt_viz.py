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


# filepath of .pgm file
filePath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'maps/race-map.pgm'))

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
    mpl.pyplot.figure(figsize=(20,10))
    mpl.pyplot.matshow(pixelValues, fignum=1)
    mpl.pyplot.show()

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

# -- MAIN SCRIPT --
if __name__ == '__main__':
  # load map
  map = Map(filePath, buffer, scale, step_size)