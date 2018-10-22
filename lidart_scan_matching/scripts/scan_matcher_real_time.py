#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import pdb
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

pub = rospy.Publisher('scan_match_location', Point, queue_size=10)
pub_odom = rospy.Publisher('scan_match_corr_odom_pos', Point, queue_size=10)

# macros
N = 1081

# Get the real time estimated location, based on the most recent pair of odom_pos and scan
is_first_time = True
prev_scan = np.array(N)
curr_scan = np.array(N)
curr_x = 0 #???is the initial location 0,0?? TODO 
curr_y = 0 #???is the initial location 0,0?? TODO 
curr_direction =  0 # initialized to be 0 (pointing in +x) degrees. positive is counter-clockwise
def real_time_estimated_location():
	global is_first_time
	global prev_scan
	global curr_scan
	global curr_direction
	global curr_x
	global curr_y
	global error_list
	global error_2_list

	# lock that will be unlocked after we receive the first scan
	while first_scan:
		print("no first scan")

	while True:
		curr_scan = process_scan(most_curr_scan)
		curr_odom_pos = most_curr_odom_pos

		if not is_first_time:

			# given 2 lidar scans, get the q that minimizes the error function
			error_list = []
			error_2_list = []

			q = iterative_find_q(curr_scan, prev_scan)

			curr_transform = np.matmul(rot(q[2]),curr_scan.T).T + np.tile(q[:2], (N,1))

			# plt.plot(prev_scan[:,0],prev_scan[:,1])
			# plt.plot(curr_scan[:,0],curr_scan[:,1])
			# plt.plot(curr_transform[:,0],curr_transform[:,1])
			# plt.legend(["prev_scan","curr_scan","curr_transform"])
			# plt.show()

			# plt.plot(error_list) # debug
			# plt.plot(error_2_list)
			# plt.legend(["error","error_2"])
			# plt.show()

			# pdb.set_trace()

			#print("found q: %0.2f %0.2f %0.2f" % (q[0], q[1], q[2]))

			# use the q to calculate current location
			curr_x = curr_x + math.sqrt(q[0]**2 + q[1]**2) * math.cos(curr_direction - q[2])
			curr_y = curr_y + math.sqrt(q[0]**2 + q[1]**2) * math.sin(curr_direction - q[2])
			# curr_x = curr_x + math.sqrt(q[0]**2 + q[1]**2) * math.cos(curr_direction + q[2])
			# curr_y = curr_y + math.sqrt(q[0]**2 + q[1]**2) * math.sin(curr_direction + q[2])
			#publish msg
			msg = Point()
			msg.x = curr_x
			msg.y = curr_y
			msg.z = 0
			pub.publish(msg)

			# update values
			curr_direction = curr_direction - q[2]
			# curr_direction = curr_direction + q[2]
			prev_scan = curr_scan

			#publish corresponding odom pos
			pub_odom.publish(curr_odom_pos)

		else: # first time
			is_first_time = False
			prev_scan = curr_scan

			msg = Point()
			msg.x = 0
			msg.y = 0
			msg.z = 0
			pub.publish(msg)

			#publish corresponding odom pos
			pub_odom.publish(curr_odom_pos)

# --------------------- HELPER FUNCTIONS RELATED TO SUBSCRIBER CALLBACK ------------------------ #

# most current odom & scan
most_curr_odom_pos = Point()
most_curr_scan = np.array(N)
first_scan = True

def scan_callback(data):
	global first_scan
	global most_curr_scan

	if first_scan:
		first_scan = False
	
	most_curr_scan = data

def odom_callback(data):
	global most_curr_odom_pos
	# pdb.set_trace() # debug
	most_curr_odom_pos = data.pose.pose.position

# --------------------- HELPER FUNCTIONS RELATED TO LIDAR SCAN PROCESSING ------------------------ #
# Process laser scan and save as cartesian
def process_scan(data):
	# read input
	scans = np.array(data.ranges)
	# cap at min and max (radian)
	theta_min = data.angle_min
	theta_max = data.angle_max
	theta_delta = data.angle_increment
	# pdb.set_trace()

	r_min = data.range_min
	r_max = data.range_max
	scans = np.clip(scans, r_min, r_max)
	thetas = np.arange(theta_min,theta_max,theta_delta)
	# converted to cartesian
	scan_x = scans*np.cos(thetas)
	scan_y = scans*np.sin(thetas)
	data_xy = np.hstack((scan_x.reshape(-1,1),scan_y.reshape(-1,1)))

	return data_xy

# --------------------- HELPER FUNCTIONS RELATED TO ITERATIVELY FINDING Q ------------------------ #
# Iteratively find the q that minimizes the error function
# input: curr_scan, prev_scan
# output: q = [t_x, t_y, theta]
epsilon_xy = 0.3
epsilon_theta = 0.2
max_iteration = 30
prev_q = np.array([0,0,0])
# debug!
error_list = []
error_2_list = []
def iterative_find_q(curr_scan, prev_scan):
	global prev_q
	global error_list
	global error_2_list
	# define an initial guess for q_0
	q = prev_q.copy()
	best_q = q.copy()
	# initialize empty C & q
	C = np.zeros([N,3])
	prev_C = np.zeros([N,3])
	best_C = np.zeros([N,3])
	# k is the cnt of iteration
	k = 0
	# initialize error & diff_C to be something big
	error = N
	diff_C = sys.float_info.max
	diff_q_xy = sys.float_info.max
	diff_q_theta = sys.float_info.max
	diff_error = sys.float_info.max
	prev_error = 0;
	best_error = sys.float_info.max

	# initialize set of previously seen q
	seen_q = set()

	# while:
	# 1. test convergence: diff(C_(k-1),C_(k)) is larger than a threshold 
	# 2. test cycle (previously seen C)
	while (diff_q_xy > epsilon_xy or diff_q_theta > epsilon_theta):
		prev_C = C.copy()
		prev_q = q.copy()
		k += 1
		# print("iteration %d" % k)
		# C_(k+1) = search_correspondence(curr_scan, prev_scan, q_(k+1))
		C = search_correspondence(curr_scan, prev_scan, prev_q)

		# debug
		error_2 = calculate_error(curr_scan, prev_scan, C, prev_q)
		error_2_list.append(error_2)
		# print("iteration %d; error_2: %0.2f" % (k, error_2))

		# diff_C = np.sum((prev_C[:,1] - C[:,1])**2)
		# print("iteration %d; diff(prev_C[:,1] - C[:,1]): %0.2f" % (k, diff_C))

		# q_(k+2) = get_q(curr_scan, prev_scan, C_(k+2))
		q = get_q(curr_scan, prev_scan, C)
		diff_q_xy = np.sqrt(np.sum((prev_q[:2] - q[:2])**2))
		diff_q_theta = abs(q[2] - prev_q[2])
		print("iteration %d; diff_q_xy: %0.2f; diff_q_theta: %0.2f" % (k, diff_q_xy, diff_q_theta))

		# calculate error(q_(k+2), C_(k+1))
		prev_error = error
		error = calculate_error(curr_scan, prev_scan, C, q)
		diff_error = error - prev_error
		if (error < best_error):
			best_error = error
			best_C = C
			best_q = q
		
		# debug
		#print("iteration %d; best_error: %0.2f; error: %0.2f; error diff: %0.2f" % (k, best_error, error, diff_error))
		error_list.append(error) 

		# pdb.set_trace() # debug

		if (k > max_iteration):
			print("maximum iteration exceeded")
			break

		# detect loop
		q_str = np.array2string(q, precision=2)

		#print("iteration %d; q_str:%s" % (k, q_str))
		if q_str in seen_q:
			print("cycle detected")
			break
		else:
			seen_q.add(q_str)

	# return best_q
	return best_q

def calculate_error(curr_scan, prev_scan, C, q):
	project_p2p = np.matmul(rot(q[2]),curr_scan.T).T + np.tile(q[:2], (N,1)) - prev_scan[C[:,1],:]
	segments = prev_scan[C[:,1],:] - prev_scan[C[:,2],:]
	normals = np.array([-segments[:,1], segments[:,0]]).T
	normals_lengths = np.sqrt(np.sum(normals**2, axis = 1))
	normals = normals / normals_lengths.reshape(N,1)
	project_p2l = np.sum(np.sum(np.multiply(normals,project_p2p), axis=1)**2)
	return project_p2l

# --------------------- HELPER FUNCTIONS RELATED TO SEARCH CORRESPONDENCE ------------------------ #
# Search the closest correspondence between curr_scan and prev_scan
# input: curr_scan, prev_scan: both lidar scans; 
# q: roto-transformation guess from curr_scan to prev_scan [t_x, t_y, theta], where theta is in radian
# output: C[N,3]: [i,ji1,ji2] - i-th scan in current scan corresponds to ji1 and ji2 in previous scan
def search_correspondence(curr_scan, prev_scan, q):
	# values used for search correspondence (in angles)
	start_angle = -135
	end_angle = 135
	angle_span = end_angle - start_angle
	acceptable_distance = sys.float_info.max

	# old scan: prev_scan
	old_x = prev_scan[:,0]
	old_y = prev_scan[:,1]
	# world scan: curr_scan transformed onto prev_scan after transform q
	world_xy = np.matmul(rot(q[2]), curr_scan.T) + np.tile((q[:2].reshape(-1,1)),(1,N))
	world_x = world_xy[0,:]
	world_y = world_xy[1,:]

	last_best = -1

	C = np.zeros([N, 3]).astype(int)
	C[:,0] = np.arange(N)

	# ??? What does this do
	up_out, up_in = getUpArrays(old_x, old_y, N)
	down_out, down_in = getDownArrays(old_x, old_y, N)

	# create pairings
	for i in range(N):
	  	world_angle = i / (N - 1.0) * angle_span + start_angle
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
				if (last_dist_up < best_dist and last_dist_up < acceptable_distance):
					best = up
					best_dist = last_dist_up
		        
				if (isAngleBigger(world_x[i], world_y[i], old_x[up], old_y[up])):
					angle_offset = (up - i) / (N - 1.0) * angle_span + start_angle
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
				if (last_dist_down < best_dist and last_dist_down < acceptable_distance):
					best = down
					best_dist = last_dist_down
		        
				if (isAngleBigger(old_x[down], old_y[down],world_x[i], world_y[i])):
					angle_offset = (i - down) / (N - 1.0) * angle_span + start_angle
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

		#best + 1 is closer
		if (best == 0 or (best != (N-1) and ((old_x[best + 1] - world_x[i])**2 + (old_y[best + 1] - world_y[i]) > (old_x[best - 1] - world_x[i])**2 + (old_y[best - 1] - world_y[i])))): 
			C[i,1] = best
			C[i,2] = best + 1
		else:
			C[i,1] = best - 1
			C[i,2] = best
                      
		last_best = best

	return C

# use radian
def rot(rad):
	return np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])

# use theta
def rot_deg(theta):
	rad = np.deg2rad(theta)
	return np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])

def getDownArrays(x, y, N):
 
  downOut = np.empty([N, 1])
  downIn = np.empty([N, 1])
  
  outValueStack = []
  outIndexStack = []
  inValueStack = []
  inIndexStack = []
  for i in range(0, N, 1):
    
    r = x[i]**2 + y[i]**2
    
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
    
    r = x[i]**2 + y[i]**2
    
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
  # x1 = y1_in
  # x2 = y2_in
  # y1 = -x1_in
  # y2 = -x2_in
  # Returns whether angle 1 is to the right of (further up for indices) angle 2
  # if (x1 == 0): return (x1 > x2)
  # if (x1 > 0 and x2 < 0): return True
  # if (x1 < 0 and x2 > 0): return False
  # if (x1 > 0 and x2 > 0):
  #   if (y1 >= 0 and y2 <= 0): return False
  #   if (y1 <= 0 and y2 >= 0): return True
  #   else:
  #     return (y2/x2 > y1/x1)
  # else:
  #   # Xs negative
  #   if (y1 <= 0 and y2 >= 0): return False
  #   if (y1 >= 0 and y2 <= 0): return True
  #   else:
  #     return (y1/x1 > y2/x2)

  if (y1 * y2 < 0):
  	return (y1 > y2)
  else:
  	calc = (x1 - x2) * y2 - x2 * (y1 - y2)
  	return (calc < 0)


# --------------------- HELPER FUNCTIONS RELATED TO GET Q ------------------------ #
def get_q(p1,p2,C):

	M = np.zeros((4,4))
	g = np.zeros((4,1))
	x = np.zeros((4,1))
	W = np.zeros((4,4))

	for i in range(len(C)):

		M_k = np.array([[1 , 0, p1[C[i,0],0], -p1[C[i,0],1] ], \
			[ 0 , 1, p1[C[i,0],1], p1[C[i,0],0] ]])

		n_k = np.matmul(normalize(p2[C[i,1],:] - p2[C[i,2],:]).reshape(1,2),rot(3.14159/2.)).reshape(2,1)

		C_k = np.matmul(n_k,n_k.T)

		M += np.matmul(np.matmul(M_k.T,C_k),M_k)

		Pi_k = p2[C[i,1],:].reshape(2,1)

		g += -2*np.matmul(Pi_k.T , np.matmul(C_k,M_k)).T

	W[[2,3],[2,3]] = 1
	W = np.matrix(W)
	A = np.matrix(2*M[0:2,0:2])
	B = np.matrix(2*M[0:2,2:4])
	D = np.matrix(2*M[2:4,2:4])
	g = np.matrix(g)
	M = np.matrix(M)

	S = D - (B.T*(A.I)*B)

	Sa = S.I * np.linalg.det(S)

	p_lambda = np.array([4., (2.*S[0,0]+2.*S[1,1]), np.linalg.det(S) ]);

	rhs = getRHS(A, B, Sa, g)

	roots = np.roots(rhs - np.convolve(p_lambda,p_lambda))

	lam = np.real(np.max(roots[np.isreal(roots)]))

	fx = lambda l: -((2*M + 2*l*W).I).T*g

	x = fx(lam)

	theta = np.arctan(x[3,0]/x[2,0]) # theta is in radian

	# print "error: ",(g.T*((2*M + 2*lam*W).I)*W*((2*M + 2*lam*W).I).T*g)[0,0] - 1

	return np.array([x[0,0],x[1,0],theta]).T

# normalize vector
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

#TODO: what does this do???
def getRHS(A,B,Sa,g):

	g1 = g[0:2]; g2=g[2:4];

   	p7 = np.array([0,0,( g1.T*((A.I)*B*  4    *B.T*(A.I))*g1 + 2*g1.T*(-(A.I)*B*  4   )*g2  + g2.T*( 4   )*g2)[0,0], \
	      ( g1.T*((A.I)*B*  4*Sa *B.T*(A.I))*g1 + 2*g1.T*(-(A.I)*B*  4*Sa)*g2  + g2.T*( 4*Sa)*g2)[0,0], \
			( g1.T*((A.I)*B* Sa*Sa *B.T*(A.I))*g1 + 2*g1.T*(-(A.I)*B* Sa*Sa)*g2 + g2.T*(Sa*Sa)*g2)])

   	return p7


# Boilerplate code to start this ROS node.
if __name__ == '__main__':
  rospy.init_node('scan_matcher_real_time', anonymous = True)
  rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.Subscriber("vesc/odom", Odometry, odom_callback)
  real_time_estimated_location()
  rospy.spin()