#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
import pdb
from geometry_msgs.msg import Point

pub = rospy.Publisher('scan_match_location', Point, queue_size=10)

# macros
N = 1081

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
is_first_time = True
prev_scan = np.array(N)
curr_scan = np.array(N)
curr_x = 0 #???is the initial location 0,0?? TODO 
curr_y = 0 #???is the initial location 0,0?? TODO 
curr_direction =  0 # initialized to be 0 (pointing in +x) degrees. positive is counter-clockwise
def scan_callback(data):
	global prev_scan
	global curr_scan
	global curr_direction

	curr_scan = process_scan(data)

	if not is_first_time:
		# given 2 lidar scans, get the q that minimizes the error function
		q = iterative_find_q(curr_scan, prev_scan)
		# use the q to calculate current location
		curr_x = curr_x + math.sqrt(q[0]**2 + q[1]**2) * math.cos(math.radians(p[2] + curr_direction))
		curr_y = curr_y + math.sqrt(q[0]**2 + q[1]**2) * math.sin(math.radians(p[2] + curr_direction))
		#publish msg
		msg = Point()
		msg.x = curr_x
		msg.y = curr_y
		msg.z = 0
		pub.publish(msg)
		# update values
		curr_direction = curr_direction + q[2]
		prev_scan = curr_scan

	else: # first time
		is_first_time = False
		prev_scan = curr_scan

		msg = Point()
		msg.x = 0
		msg.y = 0
		msg.z = 0
		pub.publish(msg)

# --------------------- HELPER FUNCTIONS RELATED TO LIDAR SCAN PROCESSING ------------------------ #
# values used for laser scan processing
START_ANGLE = -135
END_ANGLE = 135
ANGLE_SPAN = END_ANGLE - START_ANGLE
THETA_DELTA = (END_ANGLE - START_ANGLE)/N
thetas = np.arange(START_ANGLE,END_ANGLE,THETA_DELTA)
# ACCEPTABLE_DISTANCE = sys.float_info.max

# Process laser scan and save as cartesian
def process_scan(data):
	# read input
	scans = np.array(data.ranges);
	# cap at min and max
	r_min = data.range_min
	r_max = data.range_max
	scans = np.clip(scans, r_min, r_max)
	# converted to cartesian
	scan_x = scans*np.cos(thetas)
	scan_y = scans*np.sin(thetas)
	data_xy = np.hstack((scan_x,scan_y))

	return data_xy

# --------------------- HELPER FUNCTIONS RELATED TO ITERATIVELY FINDING Q ------------------------ #
# Iteratively find the q that minimizes the error function
# input: curr_scan, prev_scan
# output: q = [t_x, t_y, theta]
threshold = 0.1
def iterative_find_q(curr_scan, prev_scan):
	# define an initial guess for q_0
	q = np.array([0,0,0])
	# k is the cnt of iteration
	k = 0
	# initialize error to be something big
	error = N

	# while error(q_(k+1), C_k) is larger than a threshold:
	while (error > threshold){
		# C_(k+1) = search_correspondence(curr_scan, prev_scan, q_(k+1))
		C = search_correspondence(curr_scan, prev_scan, q)
		# q_(k+2) = get_q(curr_scan, prev_scan, C_(k+2)) #TODO
		q = get_q(curr_scan, prev_scan, C)
		# calculate error(q_(k+2), C_(k+1))
		error = calculate_error(curr_scan, prev_scan, C, q)
	}

	# return q
	return q

def calculate_error(curr_scan, prev_scan, C, q):
	project_p2p = np.matmul(rot(q[2]),curr_scan.T).T + np.tile(q[:2], (5,1)) - prev_scan[C,:]
	segments = prev_scan[C,:] - prev_scan[[(C[i] - 1) if (C[i]+1 >= len(C))  else (C[i] + 1) for i in range(5)],:]
	normals = np.array([-segments[:,1], -segments[:,0]]).T
	normals_lengths = np.sqrt(np.sum(normals**2, axis = 1))
	normals = normals / normals_lengths.reshape(5,1)
	project_p2l = np.sum(np.sum(np.multiply(normals,project_p2p), axis=1)**2)
	return project_p2l

# --------------------- HELPER FUNCTIONS RELATED TO SEARCH CORRESPONDENCE ------------------------ #
# Search the closest correspondence between curr_scan and prev_scan
# input: curr_scan, prev_scan: both lidar scans; 
# q: roto-transformation guess from curr_scan to prev_scan [t_x, t_y, theta]
# output: C[N,1]: i-th scan in curr_scan corresponds to C[i]-th scan in prev_scan
def search_correspondence(curr_scan, prev_scan, q):
	# old scan: prev_scan
	old_x = prev_scan[:,0]
	old_y = prev_scan[:,1]
	# world scan: curr_scan projected onto prev_scan after transform q
	world_x = np.matmul(rot(q[2]), curr_scan[:,0]) + q[0]
	world_y = np.matmul(rot(q[2]), curr_scan[:,1]) + q[1]

	last_best = -1

	pairings = np.empty([N, 1])

	# ??? What does this do
	up_out, up_in = getUpArrays(old_x, old_y, N)
	down_out, down_in = getDownArrays(old_x, old_y, N)

	# create pairings
	for i in range(N):
	  	world_angle = i / (N - 1.0) * ANGLE_SPAN + START_ANGLE
	  	world_norm = (world_x[i])**2 + (world_y[i])**2
	  	best = -1
	  	best_dist = sys.float_info.max
	  
	  	start_index = last_best
	  	if (start_index == -1):
	    	start_index = i
	  
	  	up = start_index + 1
	  	down = start_index
	  
	  	last_dist_up = sys.float_info.max
	  	last_dist_down = sys.float_info.max
	  
	  	up_stopped = False
	  	down_stopped = False
		  
		# up stopped becomes true when no angle can become closer
		# down stopped becoems true when no smaller angle can become true
		while not up_stopped or not down_stopped:
		    up = int(up)
		    down = int(down)

		    now_up = not up_stopped and (last_dist_up <= last_dist_down)
		    if (now_up):
		      	if (up >= N):
		        	up_stopped = True
		        	continue
		        
		      	last_dist_up = (old_x[up] - world_x[i])**2 + (old_y[up] - world_y[i])**2
		      	if (last_dist_up < best_dist and last_dist_up < ACCEPTABLE_DISTANCE):
		        	best = up
		        	best_dist = last_dist_up
		        
		      	if (isAngleBigger(world_x[i], world_y[i], old_x[up], old_y[up])):
		        	angle_offset = (up - i) / (N - 1.0) * ANGLE_SPAN + START_ANGLE
		        	min_dist_up = np.sin(np.deg2rad(angle_offset)) * world_norm
		        	if (min_dist_up > best_dist):
		        	  	up_stopped = True
		          		continue
		           
			        if (old_x[up])**2 + old_y[up]**2 > world_norm:
			          	up = up_in[up][0]
			        else:
			          	up = up_out[up][0]
		      	else:
		        	up += 1
		      	
		      	if (up < 0 or up >= N):
		          	up_stopped = True
		            
		    if (not now_up):
		      	if (down < 0):
		        	down_stopped = True
		        	continue
		        
		      	last_dist_down = (old_x[down] - world_x[i])**2 + (old_y[down]- world_y[i])**2
		      	if (last_dist_down < best_dist and last_dist_down < ACCEPTABLE_DISTANCE):
		        	best = down
		        	best_dist = last_dist_down
		        
		      	if (isAngleBigger(old_x[down], old_y[down],world_x[i], world_y[i])):
		        	angle_offset = (i - down) / (N - 1.0) * ANGLE_SPAN + START_ANGLE
		        	min_dist_down = np.sin(np.deg2rad(angle_offset)) * world_norm
		        	if (min_dist_down > best_dist):
		          		down_stopped = True
		          		continue
		            
		        if (old_x[down]**2 + old_y[down]**2 > world_norm):
		          	down = down_in[down][0]
		        else:
		          	down = down_out[down][0]
		      else:
		        down -= 1
		      if (down < 0 or down >= N): down_stopped = True
		          
		pairings[i] = best
		last_best = best

	return pairings



def rot(theta):
	return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def getDownArrays(x, y, N):
 
  downOut = np.empty([N, 1])
  downIn = np.empty([N, 1])
  
  outValueStack = []
  outIndexStack = []
  inValueStack = []
  inIndexStack = []
  for i in range(0, N, 1):
    
    r_array = x[i]**2 + y[i]**2
    r = r_array[0]
    
    lastOutIndex = -1
    lastOutValue = sys.float_info.max
    lastInIndex = -1
    lastInValue = 0
    
    while (len(outIndexStack) > 0):
      lastOutIndex = outIndexStack.pop()
      lastOutValue = outValueStack.pop()
      if (lastOutValue > r):
        break
      
    while (len(inIndexStack) > 0):
      lastInIndex = inIndexStack.pop()
      lastInValue = inValueStack.pop()
      if (lastInValue < r):
        break
      
    downOut[i] = lastOutIndex
    downIn[i] = lastInIndex
    
    if (lastOutValue > r):
      outIndexStack.append(lastOutIndex)
      outValueStack.append(lastOutValue)
    outIndexStack.append(i)
    outValueStack.append(r)
    
    if (lastInValue < r):
      inIndexStack.append(lastInIndex)
      inValueStack.append(lastInValue)
    inIndexStack.append(i)
    inValueStack.append(r)
    
  return downOut, downIn

def getUpArrays(x, y, N):
  upOut = np.empty([N, 1])
  upIn = np.empty([N, 1])
  
  outValueStack = []
  outIndexStack = []
  inValueStack = []
  inIndexStack = []
  for i in range(N-1, -1, -1):
    
    r_array = x[i]**2 + y[i]**2
    r = r_array[0]
    
    lastOutIndex = -1
    lastOutValue = sys.float_info.max
    lastInIndex = -1
    lastInValue = 0
    
    while (len(outIndexStack) > 0):
      lastOutIndex = outIndexStack.pop()
      lastOutValue = outValueStack.pop()
      if (lastOutValue > r):
        break
      
    while (len(inIndexStack) > 0):
      lastInIndex = inIndexStack.pop()
      lastInValue = inValueStack.pop()
      if (lastInValue < r):
        break
      
    upOut[i] = lastOutIndex
    upIn[i] = lastInIndex
    
    if (lastOutValue > r):
      outIndexStack.append(lastOutIndex)
      outValueStack.append(lastOutValue)
    outIndexStack.append(i)
    outValueStack.append(r)
    
    if (lastInValue < r):
      inIndexStack.append(lastInIndex)
      inValueStack.append(lastInValue)
    inIndexStack.append(i)
    inValueStack.append(r)
    
  return upOut, upIn

def dist(new_point):
  return np.sqrt(new_point[0]**2 + new_point[1]**2) 

def isAngleBigger(x1, y1, x2, y2):
  # Returns whether angle 1 is to the right of (further up for indices) angle 2
  if (x1 == 0): return (x1 > x2)
  if (x1 > 0 and x2 < 0): return True
  if (x1 < 0 and x2 > 0): return False
  if (x1 > 0 and x2 > 0):
    if (y1 >= 0 and y2 <= 0): return False
    if (y1 <= 0 and y2 >= 0): return True
    else:
      return (y2/x2 > y1/x1)
  else:
    # Xs negative
    if (y1 <= 0 and y2 >= 0): return False
    if (y1 >= 0 and y2 <= 0): return True
    else:
      return (y1/x1 > y2/x2)


# --------------------- HELPER FUNCTIONS RELATED TO GET Q ------------------------ #



# Boilerplate code to start this ROS node.
if __name__ == '__main__':
  rospy.init_node('scan_matcher', anonymous = True)
  rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.spin()