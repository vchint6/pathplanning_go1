#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

def goal_callback(goal_msg):
    global goal_x, goal_y
    goal_x = goal_msg.pose.position.x
    goal_y = goal_msg.pose.position.y
    rospy.loginfo("Received goal: x={}, y={}".format(goal_x, goal_y))

rospy.init_node('goal_marker_publisher')
marker_publisher = rospy.Publisher('goal_marker', Marker, queue_size=1)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

goal_x = 0.0  # Default values
goal_y = 0.0

goal_marker = Marker()
goal_marker.header.frame_id = 'map'  # Adjust the frame_id as per your setup.
goal_marker.type = Marker.SPHERE
goal_marker.action = Marker.ADD
goal_marker.pose.position.x = goal_x
goal_marker.pose.position.y = goal_y
goal_marker.pose.position.z = 0
goal_marker.pose.orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
goal_marker.scale.x = 0.2
goal_marker.scale.y = 0.2
goal_marker.scale.z = 0.2
goal_marker.color.r = 1.0
goal_marker.color.a = 1.0

rate = rospy.Rate(10)  # Adjust the publishing rate as needed

while not rospy.is_shutdown():
    goal_marker.pose.position.x = goal_x  # Update the X coordinate
    goal_marker.pose.position.y = goal_y  # Update the Y coordinate
    marker_publisher.publish(goal_marker)
    rate.sleep()


