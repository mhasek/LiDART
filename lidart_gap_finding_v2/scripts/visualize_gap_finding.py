#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from lidart_gap_finding.msg import gaps
import rospy
import pdb
from std_msgs.msg import ColorRGBA
# We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
publisher = rospy.Publisher('/visualization_gap_finding', Marker, queue_size="1")
marker = Marker()

# publisher_gap = rospy.Publisher('/visualization_gap_finding_gaps', Marker, queue_size="1000")
# gaps_points = Marker()

# Input data is Vector3 representing center of largest gap
def marker_callback(data):
    # while not rospy.is_shutdown():
    #     marker = Marker()

# Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
    marker.header.frame_id = "/laser"
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
    print("Sending marker")
    publisher.publish(marker)



if __name__ == '__main__':
    rospy.init_node('visualize_gap_finding')
    rospy.Subscriber('/gap_center', Vector3, marker_callback)
    # rospy.Subscriber('/gaps', gaps, gap_callback)
    rospy.spin()
