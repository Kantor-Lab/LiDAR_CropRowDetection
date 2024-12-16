#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import time
class LidarDataExtractor(Node):

    def __init__(self):
        super().__init__('lidar_data_extraction_and_publishing')

        # Publisher to send filtered point cloud data
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_lidar_points', 10)

        # Subscriber to receive PointCloud2 messages
        self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_callback, 10)
        

    def lidar_callback(self, msg):
        start = __import__('time').time()
        # Define the angle range (in radians)
        min_angle = np.radians(-30)  # Minimum angle (e.g., -30 degrees)
        max_angle = np.radians(30)   # Maximum angle (e.g., 30 degrees)

        # Create an empty list to store the filtered points
        filtered_points = []

        # Iterate through the LiDAR points
        middle = __import__('time').time()
        pc_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        print("len", len(pc_data))
        print("time", middle - start)
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            # Extract the coordinates and fields
            x, y, z= point

            # Calculate the angle from the point's coordinates (using y for angle in this case)
            # point_angle = np.arctan2(y, x)  # Use arctan2 to get the angle

            # Check if the point's angle is within the desired range and other conditions
            if min_angle <= y <= max_angle and 0 <= x <= 2.2:
                filtered_points.append([x, y, z, 0.0, np.int16(0), 0.0])  # Store points as lists
                # filtered_points.append([np.float32(x), np.float32(y), np.float32(z), np.float32(0), np.int16(0), np.float32(0)])
        # Convert filtered_points list into a numpy array with float32 type
        # filtered_points_np = np.array(filtered_points)
        self.get_logger().info(f"Filtered points count: {len(filtered_points)}")
        middle = __import__('time').time()
        print("middle time:", middle - start)
        filtered_points_bytes = bytearray()

        for point in filtered_points:
            # Pack each point manually, ensuring correct types for each field
            try:
                x, y, z, intensity, ring, time = point
                x = np.float32(x)
                y = np.float32(y)
                z = np.float32(z)
                intensity = np.float32(intensity)
                
                ring = np.int16(ring)  # Convert ring to INT16 (as a NumPy int16)
                time = np.float32(time)
                # Pack the point with correct data types (FLOAT32 for x, y, z, intensity, time and INT16 for ring)
                filtered_points_bytes.extend(struct.pack('=ffffhf', x, y, z, intensity, ring, time))
            except Exception as e:
                self.get_logger().error(f"Error packing point data: {e}")
                continue
        print("len",len(filtered_points_bytes))
                # Check if filtered_points is empty
        # if filtered_points_np.size == 0:
        #     self.get_logger().warn("No filtered points found.")
        #     return

        # Create a new PointCloud2 message for the filtered points
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = 1
        filtered_msg.width = len(filtered_points)
        # Define the fields based on your specific LiDAR format
        filtered_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.INT16, count=1),  # Corrected to INT16
            PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1)
        ]

        filtered_msg.is_bigendian = False
        filtered_msg.point_step = 22  # 3 * FLOAT32 + INT16 + 1 * FLOAT32 = 22 bytes
        filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
        
        # print("orignial data", msg.width, msg.point_step, msg.row_step, len(msg.data))
        filtered_msg.is_dense = True

        # Convert filtered points to bytes and set the data
        # filtered_msg.data = filtered_points_np.tobytes()
        filtered_msg.data = bytes(filtered_points_bytes)
        # print("new data", filtered_msg.width, filtered_msg.point_step, filtered_msg.row_step, len(filtered_msg.data), len(filtered_msg.data)/filtered_msg.width)
        # Verify if the data size matches expected size
        expected_size = filtered_msg.width * filtered_msg.height * filtered_msg.point_step
        actual_size = len(filtered_msg.data)
        if actual_size != expected_size:
            self.get_logger().warn(f"Data size mismatch: expected {expected_size} but got {actual_size}")

        # Publish the filtered points to a new topic
        end = __import__('time').time()
        print("take time:", end - start)
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

# class LidarDataExtractor(Node):
#     def __init__(self):
#         super().__init__('lidar_data_extraction_and_publishing')
        
#         # Publisher to send filtered point cloud data
#         self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_lidar_points', 10)

#         # Subscriber to receive PointCloud2 messages
#         self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_callback, 10)

#     def lidar_callback(self, msg):
#         # Extract the field names from the incoming PointCloud2 message
#         field_names = [field.name for field in msg.fields]
        
#         # Read all points, including intensity, ring, and time if available
#         points = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))

#         # Create a list to store filtered points
#         filtered_points = []

#         # Define angle range in radians
#         min_angle = np.radians(-30)  # Min angle (e.g., -30 degrees)
#         max_angle = np.radians(30)   # Max angle (e.g., 30 degrees)

#         # Filter points based on conditions
#         for point in points:
#             x, y, z = point[0], point[1], point[2]  # Coordinates
#             intensity = point[3] if "intensity" in field_names else 0
#             ring = point[4] if "ring" in field_names else 0
#             time = point[5] if "time" in field_names else 0

#             # Compute angle from the y-coordinate (assuming y represents angle here)
#             point_angle = y  # This is an assumption based on your setup

#             # Filter based on angle and x-coordinate conditions
#             if min_angle <= point_angle <= max_angle and 0 <= x <= 2.2:
#                 filtered_points.append([x, y, z, intensity, ring, time])

#         # Convert filtered points to a numpy array
#         filtered_points_np = np.array(filtered_points, dtype=np.float32)

#         # Create and publish the filtered point cloud
#         self.publish_filtered_pointcloud(filtered_points_np, msg.header, msg.fields)

#     def publish_filtered_pointcloud(self, points_np, header, original_fields):
#         # Create a new PointCloud2 message
#         filtered_msg = PointCloud2()
#         filtered_msg.header = header
#         filtered_msg.height = 1
#         filtered_msg.width = len(points_np)
#         filtered_msg.fields = original_fields  # Use the original fields
#         filtered_msg.is_bigendian = False

#         # Set the point_step to match the size of each point (x, y, z, intensity, ring, time)
#         filtered_msg.point_step = sum([field.count * 4 for field in original_fields])  # 4 bytes per float32 field
#         filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
#         filtered_msg.is_dense = True

#         # Convert filtered points to bytes
#         filtered_msg.data = points_np.tobytes()

#         # Publish the filtered message
#         self.filtered_pub.publish(filtered_msg)

# def main(args=None):
#     rclpy.init(args=args)

#     # Create and spin the node
#     node = LidarDataExtractor()
#     rclpy.spin(node)

#     # Shutdown the ROS 2 communication infrastructure
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
