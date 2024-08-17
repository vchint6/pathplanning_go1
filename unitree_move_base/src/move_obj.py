#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Twist
import time

# Initialize ROS node
rospy.init_node('move_obj_node')

# Create a service proxy to set the model state
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# Define the name of the object you want to move
model_name = 'box_obstacle'  # Replace with the actual model name

# Set the initial position
initial_state = ModelState()
initial_state.model_name = model_name
initial_state.pose.position.x = 2.25
initial_state.pose.position.y = -1.5
initial_state.pose.position.z = 0.0
initial_state.pose.orientation.x = 0.0
initial_state.pose.orientation.y = 0.0
initial_state.pose.orientation.z = 0.0
initial_state.pose.orientation.w = 1.0
initial_state.twist = Twist()
initial_state.reference_frame = 'world'  # Adjust if needed

# Define the initial velocity
initial_velocity = Twist()
initial_velocity.linear.x = 0.0
initial_velocity.linear.y = 0.5  # 0.5 m/s in the y-direction
initial_velocity.linear.z = 0.0
initial_velocity.angular.x = 0.0
initial_velocity.angular.y = 0.0
initial_velocity.angular.z = 0.0

# Define a rate for the motion
rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    # Set the object to the initial position with the initial velocity
    initial_state.twist = initial_velocity

    # Call the service to set the model state
    set_model_state(model_state=initial_state)

    # Get the current position of the box
    current_state = get_model_state(model_name, "world")
    current_position = current_state.pose.position
    rospy.loginfo("Current position: x=%f, y=%f, z=%f", current_position.x, current_position.y, current_position.z)

    # Wait for 10 seconds
    time.sleep(12.0)

    # Update the position based on the elapsed time
    initial_state.pose.position.y += initial_velocity.linear.y * 12.0

    # Switch the direction of motion by reversing the y-velocity
    initial_velocity.linear.y *= -1

    # Repeat the motion
    rate.sleep()

# When you shut down the script, the motion will stop

