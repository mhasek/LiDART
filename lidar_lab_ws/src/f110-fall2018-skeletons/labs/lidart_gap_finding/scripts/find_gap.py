#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import numpy as np
import pdb
from scipy import cluster
import matplotlib
matplotlib.use('GTKAgg')
import matplotlib.pyplot as plt
from lidart_gap_finding.msg import gaps
from sklearn.mixture import GaussianMixture as GMM
from matplotlib.patches import Ellipse
from sklearn.cluster import DBSCAN

## Initialization ##
# initialize plotting variables
plt.ion()
fig = plt.figure()
plt.show()
# initialize publishers
pub_dp = rospy.Publisher('/drive_parameters', drive_param, queue_size=1)
pub_gc = rospy.Publisher('/gap_center', Vector3, queue_size=1)
pub_g = rospy.Publisher('/lidar_gap', gaps, queue_size=1)

## Helper function##
# helper function for polar to cartesian coordinates conversion
def pol2cart(theta,r):
	x = r*np.cos(theta)
	y = r*np.sin(theta)
	return x,y

# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def scan_callback(data):
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
	scans = scans[idx].reshape(sum(idx),1)
	thetas = thetas[idx].reshape(sum(idx),1)
	data = np.hstack((thetas,scans))
	# converted to cartesian as well
	scan_x = scans*np.cos(thetas)
	scan_y = scans*np.sin(thetas)
	data_xy = np.hstack((scan_x,scan_y))

	# use DBSCAN to cluster points
	dbscan = DBSCAN(eps=2).fit(data_xy)
	# get labels and number of clusters
	label = dbscan.labels_
	n_clusters = len(set(label)) - (1 if -1 in label else 0)

	# initiate obstacle avoidance threshold
	OAT = 3
	# initiate empty arrays to save the centers of clusters
	cent = np.zeros((n_clusters,2))
	cent_p = np.zeros((n_clusters,2))
	cent_p_polar = np.zeros((n_clusters,2))
	# initiate empty arrays to save the bounds of obstacles
	obs_bound = np.zeros((n_clusters,4))

	# for each label: 
	for i in range(n_clusters):
		dat_clus = data_xy[label==i,:]
		dat_clus_polar = data[label==i,:]

		# find the center of the cluster through the closest point to the average of all cluster points
		cent[i,:] = np.mean(dat_clus,axis=0) # vertical average
		cent_idx = np.argmin(np.sum( (dat_clus - cent[i,:])**2, axis=1 )) #finding the closest point in the cluster to the centroid.
		cent_p[i,:] = dat_clus[cent_idx,:] #a matrix of on-cluster central points
		cent_p_polar[i,:] = dat_clus_polar[cent_idx,:] # get the on-cluster center polar coordinates

		# limit a valid cluster size to be >= 5
		pot_obs_idx = (scans[label==i] < OAT).reshape(-1)
		if np.sum(pot_obs_idx)<5: # if the valid scan datapoints within OAT is smaller than 5, then ignore this cluster
			continue

		# get the obstacle boundaries through the highest and lowest angles in polar coodinates of all cluster points
		pot_obs = dat_clus_polar[pot_obs_idx,:]
		obs_bound[i,:] = pot_obs[(1,-1),:].reshape(1,4) # get the first one and the last one polar coordinates of the obstacles and save them in the shape of (1,4)

	# sort the obs_bound points by angle
	obs_bound = obs_bound[obs_bound[:,0].argsort(),:] 

	## Prepare the message about gaps_data
	# initialize an empty gap message
	gaps_data = gaps()
	
	# append LIDAR detection boundaries & obstacle boundaries to the gaps_data message
	gaps_data.theta1.append(theta_min)
	gaps_data.r1.append(OAT)

	x,y = pol2cart(theta_min, OAT)
	gaps_data.x1.append(x)
	gaps_data.y1.append(y)

	for obst in obs_bound:
		gaps_data.theta2.append(obst[0])
		gaps_data.r2.append(obst[1])
		gaps_data.theta1.append(obst[2])
		gaps_data.r1.append(obst[3])

		x,y = pol2cart(obst[0], obst[1])
		gaps_data.x2.append(x)
		gaps_data.y2.append(y)

		x,y = pol2cart(obst[2], obst[3])
		gaps_data.x1.append(x)
		gaps_data.y1.append(y)

	gaps_data.theta2.append(theta_max)
	gaps_data.r2.append(OAT)

	x,y = pol2cart(theta_max, OAT)
	gaps_data.x2.append(x)
	gaps_data.y2.append(y)

	# find the delta_angle and euclidean length of each gap, and save this information in gaps_data
	for i in range(n_clusters+1):
		dx = abs(gaps_data.x2[i] - gaps_data.x1[i])
		dy = abs(gaps_data.y2[i] - gaps_data.y1[i])
		gaps_data.delta_angle.append(abs(gaps_data.theta2[i] - gaps_data.theta1[i]))
		gaps_data.euc_length.append((dx**2+dy**2)**0.5)

	g_ang = np.array(gaps_data.delta_angle)
	g_len = np.array(gaps_data.euc_length)

	# find the index of the widest gap. Here we add another constraint: the angle threshold should be > 0.5 rad 
	gap_idx = np.argmax(g_len*(g_ang>0.5))

	## Prepare the message about gap_center
	# initialize gap_center message
	gap_center = Vector3()
	gap_center.x = (gaps_data.x1[gap_idx] + gaps_data.x2[gap_idx])/2
	gap_center.y = (gaps_data.y1[gap_idx] + gaps_data.y2[gap_idx])/2
	gap_center.z = 0

	# # visualization (using matplotlib)
	# plotting clusters
	color = ['r','g','b','k']
	ax = fig.add_subplot(111)
	ax.set_xlim((-10,10))
	ax.set_ylim((10,10))
	for i in range(n_clusters):
		plt.sca(ax)
		plt.plot(scan_x[label==i],scan_y[label==i],color[i]+'o',mew=0.1)
		plt.hold(True)

	# plotting centroids & gap center
	ax.plot(cent_p[:,0],cent_p[:,1],'bx',ms=10, mew=10)
	ax.plot(cent[:,0],cent[:,1],'cx',ms=5, mew=5) # mean centroids
	#ax.plot(gap_center.x,gap_center.y,'kx',ms=10, mew=10)
	plt.sca(ax)
	plt.arrow(0,0,0.5,0,color='red',linewidth=10)
	plt.hold(False)

	plt.show()
	plt.pause(0.001)

	## Publish messages
	# publishing data
	msg = drive_param()
	msg.velocity = 0.05  # TODO: implement PID for velocity
	msg.angle = 0    # TODO: implement PID for steering angle
	pub_dp.publish(msg)
	pub_g.publish(gaps_data)
	pub_gc.publish(gap_center)


## Main Program ##
# Boilerplate code to start this ROS node.
if __name__ == '__main__':
	rospy.init_node('find_gap_node', anonymous=True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()

