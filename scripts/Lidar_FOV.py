#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarDataExtractor(Node):

    def __init__(self):
        super().__init__('lidar_data_extraction_and_publishing')

        # Publisher to send filtered point cloud data
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_lidar_points', 10)

        # Subscriber to receive PointCloud2 messages
        self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_callback, 10)

    def lidar_callback(self, msg):
        # Define the angle range (in radians) 
        min_angle = np.radians(-30)  # Minimum angle (e.g., -45 degrees)
        max_angle = np.radians(30)   # Maximum angle (e.g., 45 degrees)

        # Create an empty list to store the filtered points
        filtered_points = []

        # Get the fields from the input message
        fields = msg.fields
        num_fields = len(fields)
        point_step = num_fields * 2  # Assuming FLOAT32 fields

        # Iterate through the LiDAR points
        for point in pc2.read_points(msg, field_names=("x", "y", "z")):
            # Extract the x-coordinate of the point as an angle (modify this if your data encodes angles differently)
            point_angle = point[1]

            # Check if the point's angle is within the desired range and other conditions
            if min_angle <= point_angle <= max_angle and 0 <= point[0] <= 2.2:
                filtered_points.append(point)

        # Create a new PointCloud2 message for the filtered points
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = 1
        filtered_msg.width = len(filtered_points)
        filtered_msg.fields = fields  # Use the same fields as the original message
        filtered_msg.is_bigendian = False
        filtered_msg.point_step = point_step
        filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
        filtered_msg.is_dense = True
        filtered_msg.data = np.array(filtered_points, dtype=np.float32).tobytes()

        # Publish the filtered points to a new topic
        self.filtered_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    node = LidarDataExtractor()
    rclpy.spin(node)

    # Shutdown the ROS 2 communication infrastructure
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
