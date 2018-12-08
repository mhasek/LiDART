#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import pdb
import matplotlib.pyplot as plt
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

theta_array = []
first_scan = True
counter = 0

OCCUPANCY_GRID_HEIGHT = 7.0 # in meters
OCCUPANCY_GRID_WIDTH = 7.0 # in meters
RESOLUTION = 5.0 # boxes per meter
PIXEL_HEIGHT = int(OCCUPANCY_GRID_HEIGHT*RESOLUTION)
PIXEL_WIDTH = int(OCCUPANCY_GRID_WIDTH*RESOLUTION)

current_occupancy_grid = np.zeros((PIXEL_WIDTH, PIXEL_HEIGHT))
radius_map = np.zeros((PIXEL_WIDTH, PIXEL_HEIGHT))
theta_map = np.zeros((PIXEL_WIDTH, PIXEL_HEIGHT))

def create_theta_array(scan):
    theta_min = scan.angle_min
    theta_max = scan.angle_max
    theta_delta = (theta_max - theta_min)/len(np.array(scan.ranges))
    theta = np.arange(theta_min,theta_max,theta_delta)
    return theta

def create_radius_array():
    global radius_map
    global theta_map
    for row in range(0, PIXEL_WIDTH):
        for col in range(0, PIXEL_HEIGHT):
            x_local = row - PIXEL_WIDTH/2
            radius_map[col, row] = np.sqrt(x_local**2 + col**2)
            theta_map[col, row] = np.arctan2(col, x_local)


def callback(data):
    global counter
    counter += 1
    # updates global occupancy grid so we can have constant publishing
    global current_occupancy_grid
    global theta_array
    global first_scan

    # Only get theta array once
    if first_scan:
        theta_array = create_theta_array(data)
        first_scan = False

    scans = np.array(data.ranges) # SCANS
    angles = theta_array

    # Filter out NaN's and InF's
    filtered_scan = scans[np.isfinite(scans)]
    filtered_angles = angles[np.isfinite(scans)]

    # converts scans to x and y
    x = filtered_scan*np.sin(filtered_angles + np.deg2rad(90))
    y = filtered_scan*np.cos(filtered_angles + np.deg2rad(90))

    # Puts values in given box size for occupancy grid
    box_constraint = (x < OCCUPANCY_GRID_HEIGHT) & (x > 0) & (np.abs(y) < OCCUPANCY_GRID_WIDTH/2.0)

    x = x[box_constraint]
    y = y[box_constraint]
    # plt.axis([0, int(OCCUPANCY_GRID_WIDTH*RESOLUTION), 0, int(OCCUPANCY_GRID_HEIGHT*RESOLUTION)])
    # plt.show()


    # Converts to numpy array centered at top left
    occupancy_from_car = np.zeros((PIXEL_WIDTH, PIXEL_HEIGHT))

    y_np = (PIXEL_HEIGHT - x*RESOLUTION).astype(int)
    # y_np = ((y + (OCCUPANCY_GRID_WIDTH/2.0))*RESOLUTION).astype(int)
    x_np = (y*RESOLUTION + PIXEL_WIDTH/2.0).astype(int)
    # pdb.set_trace()
    occupancy_from_car[y_np, x_np] = 1
    # np.set_printoptions(threshold=np.nan)
    # pdb.set_trace()


    # w = PIXEL_WIDTH
    # pdb.set_trace()


    # Raytraces obstacles
    # current_radius_map = radius_map
    # current_theta_map = theta_map
    # filtered_scan = filtered_scan[box_constraint]
    # filtered_angles = filtered_angles[box_constraint]

    # print(occupancy_from_car)
    # pdb.set_trace()

    ## THIS WILL SHOW YOU
    # plt.axis([0, int(OCCUPANCY_GRID_WIDTH*RESOLUTION), 0, int(OCCUPANCY_GRID_HEIGHT*RESOLUTION)])
    # plt.imshow(occupancy_from_car, cmap='gray_r')
    # plt.show()

    # Puts coordinates in numpy frame
    # x_prime = y + OCCUPANCY_GRID_WIDTH/2.0
    # y_prime = -x + OCCUPANCY_GRID_HEIGHT

    # Convert numpy frame to indices
    # x_index = (x_prime*RESOLUTION).astype(int)
    # y_index = (y_prime*RESOLUTION).astype(int)

    # Inserts obstacles into occupancy array
    # occupancy = np.zeros((int(OCCUPANCY_GRID_WIDTH*RESOLUTION), int(OCCUPANCY_GRID_HEIGHT*RESOLUTION)))
    # occupancy[y_index, x_index] = 1
    # plt.imshow(occupancy, cmap='gray_r')
    # plt.axis([0, int(OCCUPANCY_GRID_WIDTH*RESOLUTION), 0, int(OCCUPANCY_GRID_HEIGHT*RESOLUTION)])
    # plt.show()
    # plt.gcf().clear()
    current_occupancy_grid = occupancy_from_car

if __name__ == '__main__':
    rospy.init_node('generate_occupancy_grid', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/scan", LaserScan, callback)
    pub = rospy.Publisher('grid_path', numpy_msg(Floats), queue_size=10)
    create_radius_array()
    # print(theta_map)
    # print(theta_map)
    # pdb.set_trace()
    # pdb.set_trace()
    # pub = rospy.Publisher('floats', numpy_msg(Floats), queue_size=10)
    while not rospy.is_shutdown():
        # pub.publish(current_occupancy_grid)
        # print("PUBLISHING")
        rate.sleep()
        # pub.publish(current_occupancy_grid)
        # rate.sleep()
