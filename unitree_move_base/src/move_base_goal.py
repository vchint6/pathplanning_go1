#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped

# Define the boundaries of your map
MAP_MIN_X = -10.0
MAP_MAX_X = 10.0
MAP_MIN_Y = -10.0
MAP_MAX_Y = 10.0

def send_goal(x, y, z, w):
    # Initialize the ROS node
    rospy.init_node('send_goal_node', anonymous=True)
    
    # Create a publisher for the /move_base_simple/goal topic
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    # Wait for the publisher to establish the connection
    rospy.sleep(1)
    
    # Create the goal message
    goal = PoseStamped()
    goal.header.frame_id = "slamware_map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0  # Assuming flat terrain
    
    # Set the orientation using quaternion values
    goal.pose.orientation.x = 0.0  # Usually, this will be 0 for a 2D navigation
    goal.pose.orientation.y = 0.0  # Usually, this will be 0 for a 2D navigation
    goal.pose.orientation.z = z    # Orientation around the Z axis
    goal.pose.orientation.w = w    # Orientation around the W axis
    
    # Publish the goal
    rospy.loginfo("Sending goal to ({}, {}, {}, {})".format(x, y, z, w))
    goal_pub.publish(goal)
    
    rospy.loginfo("Goal sent successfully!")

def get_user_input():
    while True:
        try:
            x = float(input("Enter target x-coordinate: "))
            y = float(input("Enter target y-coordinate: "))
            angle_degrees = float(input("Enter target orientation in degrees: "))
            
            # Convert angle in degrees to quaternion
            angle_radians = math.radians(angle_degrees)
            z = math.sin(angle_radians / 2.0)
            w = math.cos(angle_radians / 2.0)
            
            return x, y, z, w
        except ValueError:
            print("Invalid input. Please enter numeric values.")

def check_map_bounds(x, y):
    if MAP_MIN_X <= x <= MAP_MAX_X and MAP_MIN_Y <= y <= MAP_MAX_Y:
        return True
    else:
        return False

if __name__ == '__main__':
    try:
        while True:
            # Get user input for target coordinates and orientation
            target_x, target_y, target_z, target_w = get_user_input()
            
            # Check if the target point is within the map boundaries
            if check_map_bounds(target_x, target_y):
                # Send the goal
                send_goal(target_x, target_y, target_z, target_w)
            else:
                print("Target point is out of the map region. Please enter a valid point.")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation goal interrupted.")

