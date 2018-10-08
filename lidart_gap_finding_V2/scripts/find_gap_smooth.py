#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
import numpy as np
import pdb
from scipy import cluster
from lidart_gap_finding.msg import gaps
from lidart_gap_finding.msg import gap
from sklearn.mixture import GaussianMixture as GMM
from matplotlib.patches import Ellipse
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import itertools

# initialize publishers
# pub_dp = rospy.Publisher('/drive_parameters', drive_param, queue_size=1)
pub_gc = rospy.Publisher('/gap_center', Vector3, queue_size=1)
pub_g = rospy.Publisher('/gaps_data', gaps, queue_size=1)
pub_m = rospy.Publisher('/Gaps_Marker', Marker, queue_size=1)
pub_obs = rospy.Publisher('/Obs_Marker', Marker, queue_size=1)
pub_cntrs = rospy.Publisher('/Cntrs_Marker', Marker, queue_size=1)

# variables for smoothing
prev_gap_center = Vector3()
prev_gap_euc_length = 0
min_gap_len = 0.5
min_gap_ang = np.deg2rad(10)
# min_gap_dep = 2
smooth_dist2_th = 0.25
# initiate obstacle avoidance threshold
OAT = 3


## Helper function##
# helper function for polar to cartesian coordinates conversion
def pol2cart(theta,r):
	x = r*np.cos(theta)
	y = r*np.sin(theta)
	return x,y

def getScanRange(theta1,theta2,data):
	scans = np.array(data.ranges)
	theta_min = data.angle_min
	theta_max = data.angle_max
	theta_delta = data.angle_increment
	r_min = data.range_min
	r_max = data.range_max
	thetas = np.arange(theta_min,theta_max,theta_delta)

	angle_index1 = find_nearest(thetas, np.deg2rad(theta1))
	angle_index2 = find_nearest(thetas, np.deg2rad(theta2))
	# cap infinite values at OAT
	range_val = scans[angle_index1:angle_index2]

	return range_val

# This method finds the index of value nearest to array
def find_nearest(array, value):
	array = np.asarray(array)
	idx = (np.abs(array - value)).argmin()
	return idx

# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def scan_callback(data):
	global OAT
	# get scan & angle from received LIDAR data
	scans = np.array(data.ranges)
	theta_min = data.angle_min
	theta_max = data.angle_max
	theta_delta = (theta_max - theta_min)/len(scans)
	r_min = data.range_min
	r_max = data.range_max

	# limit scan values within range
	idx = (scans > r_min) & (scans < r_max)

	# generate a matrix to correspond angles with ranges
	thetas = np.arange(theta_min,theta_max,theta_delta)
	scans = scans[idx].reshape(-1,1)
	thetas = thetas[idx].reshape(-1,1)

	thetas = thetas[scans<=OAT]
	scans = scans[scans<=OAT]

	# only look at obstacles that are in front of you * bit wise and
	ang_min = -3.14/2
	ang_max = 3.14/2
	scans = scans[(thetas <= ang_max) * (thetas >= ang_min)].reshape(-1,1)
	thetas = thetas[(thetas <= ang_max) * (thetas >= ang_min)].reshape(-1,1)

	data_pol = np.hstack((thetas,scans))

	# converted to cartesian as well
	scan_x = scans*np.cos(thetas)
	scan_y = scans*np.sin(thetas)
	data_xy = np.hstack((scan_x,scan_y))

	# use DBSCAN to cluster points
	dbscan = DBSCAN(eps=0.5,min_samples=10).fit(data_xy)
	# get labels and number of clusters
	label = dbscan.labels_

	scans = scans[label!=-1]
	thetas = thetas[label!=-1]
	data_pol = data_pol[label!=-1,:]
	scan_x = scan_x[label!=-1]
	scan_y = scan_y[label!=-1]
	data_xy = data_xy[label!=-1,:]
	label = label[label!=-1]

	n_clusters = len(np.unique(label))

	gap_center = Vector3()
	if n_clusters == 0:
		gap_center.x = OAT
		gap_center.y = 0
		gap_center.z = 0
		pub_gc.publish(gap_center)
		return


	# initiate empty arrays to save the centers of clusters
	cent_p = np.zeros((n_clusters,2))
	cent_p_polar = np.zeros((n_clusters,2))
	# initiate empty arrays to save the bounds of obstacles
	obs_bound = np.zeros((n_clusters,4))
	obs_bound_cart = np.zeros((n_clusters,4))

	# for each label: 
	for i,lab_idx in enumerate(np.unique(label)):
		dat_clus = data_xy[label==lab_idx,:]
		dat_clus_polar = data_pol[label==lab_idx,:]

		# find the center of the cluster through the closest point to the average of all cluster points
		cent = np.mean(dat_clus,axis=0) # vertical average
		cent_idx = np.argmin(np.sum( (dat_clus - cent)**2, axis=1 )) #finding the closest point in the cluster to the centroid.
		cent_p[i,:] = dat_clus[cent_idx,:] #a matrix of on-cluster central points
		cent_p_polar[i,:] = dat_clus_polar[cent_idx,:] # get the on-cluster center polar coordinates

		# limit a valid cluster size to be >= 10
		pot_obs_idx = (scans[label==lab_idx] < OAT).reshape(-1)
		# if np.sum(pot_obs_idx)<10: # if the valid scan datapoints within OAT is smaller than 10, then ignore this cluster
		# 	continue

		# get the obstacle boundaries through the highest and lowest angles in polar coodinates of all cluster points
		pot_obs = dat_clus_polar[pot_obs_idx,:]
		obs_bound[i,:] = pot_obs[(1,-1),:].reshape(-1) # get the first one and the last one polar coordinates of the obstacles and save them in the shape of (1,4)
		
		pot_obs_xy = dat_clus[pot_obs_idx,:]
		obs_bound_cart[i,:] = pot_obs_xy[(1,-1),:].reshape(-1)

	# sort the obs_bound points by angle
	# print "unsorted :\n" ,obs_bound
	obs_bound_cart = obs_bound_cart[obs_bound[:,0].argsort(),:] 
	obs_bound = obs_bound[obs_bound[:,0].argsort(),:]
	# print "sorted :\n" ,obs_bound

	## Prepare the message about gaps_data
	# initialize an empty gap message
	gaps_data = gaps()

	# append LIDAR detection boundaries & obstacle boundaries to the gaps_data message
	g = gap()

	g.theta1 = ang_min
	g.r1 = OAT
	g.x1,g.y1 = pol2cart(ang_min, OAT)

	for obst,obst_xy in itertools.izip(obs_bound,obs_bound_cart):
			g.theta2 = obst[0]
			g.r2 = obst[1]
			g.x2 = obst_xy[0]
			g.y2 = obst_xy[1]
			dx = abs(g.x2 - g.x1)
			dy = abs(g.y2 - g.y1)
			g.delta_angle = abs(g.theta2 - g.theta1)
			g.euc_length = (dx**2+dy**2)**0.5
			g.cx = 0.5*(g.x1 + g.x2)
			g.cy = 0.5*(g.y1 + g.y2)

			# gap_depths = getScanRange(g.theta1, g.theta2, data)
			if (g.euc_length > min_gap_len) and (g.delta_angle > min_gap_ang):
				gaps_data.data.append(g)

			g = gap()
			g.theta1 = obst[2]
			g.r1 = obst[3]
			g.x1 = obst_xy[2]
			g.y1 = obst_xy[3]

	g.theta2 = ang_max
	g.r2 = OAT
	g.x2,g.y2 = pol2cart(ang_max, OAT)
	dx = abs(g.x2 - g.x1)
	dy = abs(g.y2 - g.y1)
	g.delta_angle = abs(g.theta2 - g.theta1)

	g.euc_length = (dx**2+dy**2)**0.5
	g.cx = 0.5*(g.x1 + g.x2)
	g.cy = 0.5*(g.y1 + g.y2)

	# gap_depths = getScanRange(g.theta1, g.theta2, data)
	if (g.euc_length > min_gap_len) and (g.delta_angle > min_gap_ang):
		gaps_data.data.append(g)




	## decide the gap center to use
	gap_center = find_gap_center(gaps_data)

	## set up markers ##
	publish_gaps_marker(gaps_data)
	publish_obs_marker(obs_bound_cart)

	# about the center
	pub_gc.publish(gap_center)
	# publish_cntrs_marker(gap_center)
	publish_cntrs_marker(gaps_data)

	# publish /gaps_data
	pub_g.publish(gaps_data)
	

############ function to decide gap center index ####################
# if the difference between one of the current gap centers and the previous gap center is < 0.5 * prev_gap_euc_length, choose this gap center
# else, just choose the center of the widest gap
# TODO: adjust the threshold for choosing closest gap center
def find_gap_center(gaps_data):
	global prev_gap_center, prev_gap_euc_length, smooth_dist2_th
	max_len = 0.
	min_dist = np.inf
	gc = Vector3()
	for i,g in enumerate(gaps_data.data):
		dist = (prev_gap_center.x - g.cx)**2 + (prev_gap_center.y - g.cy)**2

		if dist < smooth_dist2_th and dist < min_dist:
			min_dist = dist
			gc.x = g.cx
			gc.y = g.cy
			gc.z = 0
		elif g.euc_length > max_len and min_dist != np.inf:
			max_len = g.euc_length
			gc.x = g.cx
			gc.y = g.cy
			gc.z = 0

	if max_len == 0 and min_dist == np.inf:
		gc.x = OAT

	return gc
		

############ functions to set up markers #############
def publish_gaps_marker(gaps_data):
	global pub_m
	
	Gaps_Marker = Marker()

	Gaps_Marker.header.frame_id = "/laser"

	Gaps_Marker.type = Gaps_Marker.LINE_LIST

	Gaps_Marker.scale.x = 0.1
	Gaps_Marker.scale.y = 0.1
	Gaps_Marker.scale.z = 0.1

	Gaps_Marker.color.r = 1
	Gaps_Marker.color.g = 0
	Gaps_Marker.color.b = 0
	Gaps_Marker.color.a = 1

	for g in gaps_data.data:
		p = Point()
		p.x = g.x1
		p.y = g.y1
		p.z = 0
		Gaps_Marker.points.append(p)
		p = Point()
		p.x = g.x2
		p.y = g.y2
		p.z = 0
		Gaps_Marker.points.append(p)

	pub_m.publish(Gaps_Marker)

def publish_obs_marker(obs_bound_cart):
	Obs_Marker = Marker()

	Obs_Marker.header.frame_id = "/laser"

	Obs_Marker.type = Obs_Marker.LINE_LIST

	Obs_Marker.scale.x = 0.1
	Obs_Marker.scale.y = 0.1
	Obs_Marker.scale.z = 0.1

	Obs_Marker.color.r = 1
	Obs_Marker.color.g = 1
	Obs_Marker.color.b = 0
	Obs_Marker.color.a = 1

	for obs in obs_bound_cart:
		p1 = Point()
		p1.x = obs[0]
		p1.y = obs[1]
		p1.z = 0
		p2 = Point()
		p2.x = obs[2]
		p2.y = obs[3]
		p2.z = 0
		Obs_Marker.points.append(p1)
		Obs_Marker.points.append(p2)

	pub_obs.publish(Obs_Marker)


## visualize selected gap center
# def publish_cntrs_marker(gap_center):
# 	marker = Marker()
# 	marker.header.frame_id = "/laser"
# 	marker.pose.position.x = gap_center.x
# 	marker.pose.position.y = gap_center.y
# 	marker.pose.position.z = gap_center.z # or set this to 0

# 	marker.type = marker.SPHERE

# 	marker.scale.x = 0.2 # If marker is too small in Rviz can make it bigger here
# 	marker.scale.y = 0.2
# 	marker.scale.z = 0.2
# 	marker.color.a = 1.0
# 	marker.color.r = 1.0
# 	marker.color.g = 1.0
# 	marker.color.b = 0.0

# 	pub_cntrs.publish(marker)



# visualize all gap centers
def publish_cntrs_marker(gaps_data):
	marker = Marker()
	marker.header.frame_id = "/laser"

	marker.type = marker.POINTS

	for g in gaps_data.data:
		p = Point()
		p.x = g.cx
		p.y = g.cy
		p.z = 0
		marker.points.append(p)

	marker.scale.x = 0.2 # If marker is too small in Rviz can make it bigger here
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0

	pub_cntrs.publish(marker)

## Main Program ##
# Boilerplate code to start this ROS node.
if __name__ == '__main__':
	rospy.init_node('find_gap_node', anonymous=True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()

