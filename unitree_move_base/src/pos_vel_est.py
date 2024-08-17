#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

# Function to calculate the magnitude of a 3D vector
def calculate_magnitude(vector):
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)

# Callback function to print the models' state and calculate 'l'
def model_state_callback(msg, params):
    length_go1, breadth_obstacle, length_obstacle, marker_pub = params

    try:
        # Find the index of the 'box_obstacle' in the model states list
        box_obstacle_index = msg.name.index('box_obstacle')
        go1_gazebo_index = msg.name.index('go1_gazebo')

        # Extract the linear velocity vectors for 'box_obstacle'
        velocity_box = msg.twist[box_obstacle_index].linear

        # Extract the linear velocity vectors for 'go1_gazebo'
        velocity_go1 = msg.twist[go1_gazebo_index].linear

        # Calculate the magnitude (resultant velocity) for both models
        magnitude_box = calculate_magnitude(velocity_box)
        magnitude_go1 = calculate_magnitude(velocity_go1)

        # Calculate 'l' using the specified formula or set to 3 if the velocity of 'go1' is 0 or if l > 3
        if magnitude_go1 != 0:
            l = (magnitude_box / magnitude_go1) * (length_go1 + breadth_obstacle)
            l = min(l, 3) + length_obstacle  # Set l to 3 if l is greater than 3
        else:
            l = 3 + length_obstacle

        #print("Parameter 'l': {}".format(l))

        # Draw a vector from the center of the obstacle in the direction of its movement
        marker = Marker()
        marker.header.frame_id = "/map"  # Adjust the frame_id if needed
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Width of the arrow
        marker.scale.y = 0.2  # Length of the arrow
        marker.scale.z = 0.1  # Height of the arrow (for visualization in RViz)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        try:
            # Find the index of the 'box_obstacle' in the model states list
            box_obstacle_index = msg.name.index('box_obstacle')

            # Extract the position and orientation vectors for 'box_obstacle'
            position_box = msg.pose[box_obstacle_index].position
            orientation_box = msg.pose[box_obstacle_index].orientation

            # Set the starting point of the arrow to the center of the 'box_obstacle'
            marker.points.append(Point(x=position_box.x+0.25, y=position_box.y+0.5, z=position_box.z))

            # Calculate the end point based on the direction of movement (using the linear velocity)
            end_point_x = position_box.x+0.25 + l * velocity_box.x
            end_point_y = position_box.y+0.5 + l * velocity_box.y

            # Set the end point of the arrow based on the calculated 'l' in the direction of movement
            marker.points.append(Point(x=end_point_x, y=end_point_y, z=position_box.z))

            # Publish the marker
            marker_pub.publish(marker)

        except ValueError:
            # Handle the case where one or both models are not found in the list
            print("Error: One or both models not found in the model states list.")

    except ValueError:
        # Handle the case where one or both models are not found in the list
        print("Error: One or both models not found in the model states list.")

# Initialize ROS node
rospy.init_node('print_model_states_node')

# Parameters for length and breadth
length_go1 = rospy.get_param('~length_go1', 0.6)  # Default length of 'go1'
breadth_obstacle = rospy.get_param('~breadth_obstacle', 0.4)  # Default breadth of 'obstacle'
length_obstacle = rospy.get_param('~length_obstacle', 0.4)

# Publisher for the marker
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Subscribe to the model states topic with parameters
rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback, (length_go1, breadth_obstacle, length_obstacle, marker_pub))

# Keep the program running
rospy.spin()

