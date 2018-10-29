#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy
import pdb
from std_msgs.msg import ColorRGBA

# We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
publisher = rospy.Publisher('/scan_matcher_visualization', Marker, queue_size="1")
marker = Marker()

publisher_odom = rospy.Publisher('/scan_matcher_odom_visualization', Marker, queue_size="1")
marker_odom = Marker()

# publisher_gap = rospy.Publisher('/visualization_gap_finding_gaps', Marker, queue_size="1000")
# gaps_points = Marker()

# Input data is Vector3 representing center of largest gap
def marker_callback(data):
# Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
    marker.header.frame_id = "/map"
    marker.pose.position.x = data.x
    marker.pose.position.y = data.y
    marker.pose.position.z = data.z # or set this to 0

    marker.type = marker.SPHERE

    marker.scale.x = 0.2 # If marker is too small in Rviz can make it bigger here
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Publish the MarkerArray
    publisher.publish(marker)

def marker_odom_callback(data):
    # Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
    marker_odom.header.frame_id = "/map"
    marker_odom.pose.position.x = data.x
    marker_odom.pose.position.y = data.y
    marker_odom.pose.position.z = data.z # or set this to 0

    marker_odom.type = marker_odom.SPHERE

    marker_odom.scale.x = 0.2 # If marker is too small in Rviz can make it bigger here
    marker_odom.scale.y = 0.2
    marker_odom.scale.z = 0.2
    marker_odom.color.a = 1.0
    marker_odom.color.r = 0.0
    marker_odom.color.g = 0.0
    marker_odom.color.b = 1.0

    # Publish the MarkerArray
    publisher.publish(marker_odom)

if __name__ == '__main__':
    rospy.init_node('scan_matcher_visualization')
    rospy.Subscriber('/scan_match_location', Point, marker_callback)
    rospy.Subscriber('/scan_match_corr_odom_pos', Point, marker_odom_callback)
    rospy.spin()
