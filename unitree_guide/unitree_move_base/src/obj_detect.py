#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from math import cos, sin
from sklearn.cluster import DBSCAN
import numpy as np

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)

        # Assume a constant distance threshold for object detection
        self.detection_threshold = 2.0  # Modify as needed

        # Variables to store the previous lidar scan data
        self.prev_lidar_data = None

        # Subscribe to LaserScan messages
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        rospy.spin()

    def lidar_callback(self, lidar_msg):
        # Check if there is previous lidar data
        if self.prev_lidar_data is not None:
            # Initialize lists to store x, y coordinates of obstacle points
            obstacle_points_x = []
            obstacle_points_y = []

            # Iterate through each range measurement
            for i, range_value in enumerate(lidar_msg.ranges):
                angle = lidar_msg.angle_min + i * lidar_msg.angle_increment

                # Check if the range is within the detection threshold
                if range_value < self.detection_threshold:
                    # Convert polar coordinates to Cartesian coordinates
                    x = range_value * cos(angle)
                    y = range_value * sin(angle)

                    # Save x, y coordinates of obstacle points
                    obstacle_points_x.append(x)
                    obstacle_points_y.append(y)

            # Cluster the obstacle points using DBSCAN
            clustered_points = self.cluster_obstacle(obstacle_points_x, obstacle_points_y)

            # Log the information
            rospy.loginfo("Number of clusters: {}".format(len(clustered_points)))
            for cluster in clustered_points:
                rospy.loginfo("Cluster: {}".format(cluster))

        # Save the current lidar data for the next iteration
        self.prev_lidar_data = lidar_msg

    def cluster_obstacle(self, obstacle_points_x, obstacle_points_y):
        # Stack the x, y coordinates to create a feature matrix
        features = np.column_stack((obstacle_points_x, obstacle_points_y))

        # Apply DBSCAN clustering algorithm
        dbscan = DBSCAN(eps=0.2, min_samples=5)  # Modify eps and min_samples as needed
        labels = dbscan.fit_predict(features)

        # Organize points into clusters
        clustered_points = {}
        for i, label in enumerate(labels):
            if label not in clustered_points:
                clustered_points[label] = []
            clustered_points[label].append((features[i, 0], features[i, 1]))

        return clustered_points

if __name__ == '__main__':
    ObjectDetectionNode()

