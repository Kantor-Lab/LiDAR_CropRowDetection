#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct
import time
class LidarPlaneFittingNode(Node):
    def __init__(self):
        super().__init__('lidar_plane_fitting')

        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/plane_marker', 10)  # Increased queue size
        self.filtered_pub = self.create_publisher(PointCloud2, '/points_above_plane', 10)

        # Subscriber
        self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_callback, 10)

    def lidar_callback(self, msg):
        # Direct access to PointCloud2 data (no conversion to numpy until necessary)
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Process points and extract x, y, z directly
        # start = __import__('time').time()
        x, y, z = np.array([point[0] for point in points]), np.array([point[1] for point in points]), np.array([point[2] for point in points])
        # end = __import__('time').time()
        # print("time:", end - start)
        above_plane_threshold = 0.06  # Threshold for filtering points above the plane

        # Create a Marker message to visualize the plane (still necessary for visualization)
        plane_marker = self.create_plane_marker(msg, x, y, z)
        self.marker_pub.publish(plane_marker)

        # Efficient point filtering
        points_above_plane, points_above_plane_bytes = self.filter_points_above_plane(msg, plane_marker, x, y, z, above_plane_threshold)
        # print("points above plane:", len(points_above_plane))
        # Publish the filtered points
        self.publish_filtered_points(msg, points_above_plane, points_above_plane_bytes)

    def create_plane_marker(self, msg, x, y, z):
        # Create and configure a Marker message to visualize the plane
        plane_marker = Marker()
        plane_marker.header = msg.header
        plane_marker.type = Marker.CUBE
        plane_marker.action = Marker.ADD
        plane_marker.pose.position.x = float(np.mean(x))
        plane_marker.pose.position.y = float(np.mean(y))
        plane_marker.pose.position.z = float(np.mean(z) + 0.02)  # Plane offset

        # Set orientation using a predefined rotation
        q1 = R.from_euler('xyz', [0, -0.7, 0]).as_quat()
        plane_marker.pose.orientation.x = q1[0]
        plane_marker.pose.orientation.y = q1[1]
        plane_marker.pose.orientation.z = q1[2]
        plane_marker.pose.orientation.w = q1[3]

        # Set scale and color properties
        plane_marker.scale.x = 10.0  # Scale of the plane
        plane_marker.scale.y = 10.0
        plane_marker.scale.z = 0.001
        plane_marker.color.a = 0.5  # Transparency
        plane_marker.color.r = 0.0
        plane_marker.color.g = 1.0
        plane_marker.color.b = 0.0

        return plane_marker

    def filter_points_above_plane(self, msg, plane_marker, x, y, z, above_plane_threshold):
        # Calculate the rotation matrix from the quaternion
        rotation_matrix = self.quaternion_to_rotation_matrix(plane_marker.pose.orientation)

        # Determine the plane's normal vector
        normal_vector = np.dot(rotation_matrix, np.array([0, 0, 1]))
        normal_vector /= np.linalg.norm(normal_vector)  # Normalize the vector

        # Calculate the offset 'd' for the plane equation
        d = -np.dot(normal_vector, np.array([np.mean(x), np.mean(y), np.mean(z)]))

        # Filter points more efficiently
        filtered_points = [
            point for point in zip(x, y, z)  # Process points as tuples
            if self.is_point_above_plane(point, normal_vector, d, above_plane_threshold)
            and self.is_point_within_range(point)
        ]

        # Pack filtered points
        points_above_plane_bytes = bytearray()
        for point in filtered_points:
            x, y, z = point
            points_above_plane_bytes.extend(struct.pack('=fff', x, y, z))  # Assuming simple 3D point packing
        return filtered_points, points_above_plane_bytes

    def is_point_above_plane(self, point, normal_vector, d, above_plane_threshold):
        # Check if the point is above the plane
        a, b, c = normal_vector
        return a * point[0] + b * point[1] + c * point[2] + d > above_plane_threshold

    def is_point_within_range(self, point):
        # Simplify the filter to just range conditions
        x, y = point[0], point[1]
        robot_width = 1.524
        Lidar_range = 2.2
        min_dis = -robot_width/2  # Example: Min angle (e.g., -30 degrees)
        max_dis = robot_width/2   # Example: Max angle (e.g., 30 degrees)

        return min_dis <= y <= max_dis and 0 <= x <= Lidar_range

    def publish_filtered_points(self, msg, filtered_points, points_above_plane_bytes):
        # Create and configure a new PointCloud2 message for the filtered points
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = 1
        filtered_msg.width = len(filtered_points)
        filtered_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        filtered_msg.is_bigendian = False
        filtered_msg.point_step = 12  # 3 floats for x, y, z
        filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
        filtered_msg.is_dense = True
        filtered_msg.data = bytes(points_above_plane_bytes)

        # Publish the filtered points
        self.filtered_pub.publish(filtered_msg)

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion):
        # Convert a quaternion to a rotation matrix
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return np.array([
            [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2]
        ])


def main(args=None):
    rclpy.init(args=args)
    node = LidarPlaneFittingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
# import rclpy
# from rclpy.node import Node
# from sensor_msgs_py import point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2, PointField
# from visualization_msgs.msg import Marker
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs_py.point_cloud2 import read_points
# import struct
# import time
# class LidarPlaneFittingNode(Node):
#     def __init__(self):
#         super().__init__('lidar_plane_fitting')

#         # Publishers
#         self.marker_pub = self.create_publisher(Marker, '/plane_marker', 1)
#         self.filtered_pub = self.create_publisher(PointCloud2, '/points_above_plane', 10)

#         # Subscriber
#         self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_callback, 1)

#     def lidar_callback(self, msg):
#         # Convert the PointCloud2 message to a NumPy array
#         filtered_points = []
#         for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring", "time"), skip_nans=True):
#             x, y, z, intensity, ring, time = point
#             filtered_points.append([np.float32(x), np.float32(y), np.float32(z), np.float32(intensity), np.int16(ring), np.float32(time)])
#         # pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#         # pc_array = np.array(list(pc_data))
#         pc_array = np.array(filtered_points)
#         # print(pc_array[:2])
#         x, y, z = pc_array[:, 0], pc_array[:, 1], pc_array[:, 2]

#         above_plane_threshold = 0.02  # Threshold for filtering points above the plane

#         # Create a Marker message to visualize the plane
#         plane_marker = self.create_plane_marker(msg, x, y, z)

#         # Publish the Marker message
#         self.marker_pub.publish(plane_marker)

#         # Filter the points based on their position relative to the plane and additional filters
#         points_above_plane, points_above_plane_bytes = self.filter_points_above_plane(msg,plane_marker, x, y, z, above_plane_threshold)

#         # Publish the filtered PointCloud2 message
#         self.publish_filtered_points(msg, points_above_plane, points_above_plane_bytes)

#     def create_plane_marker(self, msg, x, y, z):
#         # Create and configure a Marker message to visualize the plane
#         plane_marker = Marker()
#         plane_marker.header = msg.header
#         plane_marker.type = Marker.CUBE
#         plane_marker.action = Marker.ADD
#         plane_marker.pose.position.x = float(np.mean(x))
#         plane_marker.pose.position.y = float(np.mean(y))
#         plane_marker.pose.position.z = float(np.mean(z) + 0.02)  # Plane offset

#         # Set orientation using a predefined rotation
#         q1 = R.from_euler('xyz', [0, -0.7, 0]).as_quat()
#         plane_marker.pose.orientation.x = q1[0]
#         plane_marker.pose.orientation.y = q1[1]
#         plane_marker.pose.orientation.z = q1[2]
#         plane_marker.pose.orientation.w = q1[3]

#         # Set scale and color properties
#         plane_marker.scale.x = 10.0  # Scale of the plane
#         plane_marker.scale.y = 10.0
#         plane_marker.scale.z = 0.001
#         plane_marker.color.a = 0.5  # Transparency
#         plane_marker.color.r = 0.0
#         plane_marker.color.g = 1.0
#         plane_marker.color.b = 0.0

#         return plane_marker

#     def filter_points_above_plane(self, msg, plane_marker, x, y, z, above_plane_threshold):
#         # Calculate the rotation matrix from the quaternion
#         rotation_matrix = self.quaternion_to_rotation_matrix(plane_marker.pose.orientation)

#         # Determine the plane's normal vector
#         normal_vector = np.dot(rotation_matrix, np.array([0, 0, 1]))
#         normal_vector /= np.linalg.norm(normal_vector)  # Normalize the vector

#         # Calculate the offset 'd' for the plane equation
#         d = -np.dot(normal_vector, np.array([np.mean(x), np.mean(y), np.mean(z)]))

#         # Filter points based on whether they are above the plane and the angle & x conditions
#         points_above_plane = [
#             point for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring", "time"))
#             if self.is_point_above_plane(point, normal_vector, d, above_plane_threshold)
#             and self.is_point_within_range(point)
#         ]
#         print("len",len(points_above_plane),points_above_plane[:1])
#         points_above_plane_bytes = bytearray()
#         for point in points_above_plane:
#             # Pack each point manually, ensuring correct types for each field
#             try:
#                 x, y, z, intensity, ring, time = point
#                 x = np.float32(x)
#                 y = np.float32(y)
#                 z = np.float32(z)
#                 intensity = np.float32(intensity)
                
#                 ring = np.int16(ring)  # Convert ring to INT16 (as a NumPy int16)
#                 time = np.float32(time)
#                 # Pack the point with correct data types (FLOAT32 for x, y, z, intensity, time and INT16 for ring)
#                 points_above_plane_bytes.extend(struct.pack('=ffffhf', x, y, z, intensity, ring, time))
#             except Exception as e:
#                 self.get_logger().error(f"Error packing point data: {e}")
#                 continue
#         print("len", len(points_above_plane_bytes))
#         return points_above_plane, points_above_plane_bytes

#     def is_point_above_plane(self, point, normal_vector, d, above_plane_threshold):
#         # Check if the point is above the plane
#         a, b, c = normal_vector
#         return a * point[0] + b * point[1] + c * point[2] + d > above_plane_threshold

#     def is_point_within_range(self, point):
#         # Filter based on angle and x-coordinate conditions
#         x, y = point[0], point[1]
#         min_angle = np.radians(-30)  # Example: Min angle (e.g., -30 degrees)
#         max_angle = np.radians(30)   # Example: Max angle (e.g., 30 degrees)

#         return min_angle <= y <= max_angle and 0 <= x <= 2.2

#     def publish_filtered_points(self, msg, points_above_plane, points_above_plane_bytes):
#         # Create and configure a new PointCloud2 message for the filtered points
#         filtered_msg = PointCloud2()
#         filtered_msg.header = msg.header
#         filtered_msg.height = 1
#         filtered_msg.width = len(points_above_plane)
#         filtered_msg.fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#             PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
#             PointField(name='ring', offset=16, datatype=PointField.INT16, count=1),  # Corrected to INT16
#             PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1)
#         ]
#         filtered_msg.is_bigendian = False
#         filtered_msg.point_step = 22 #len(msg.fields) * 4  # Assuming 32-bit float for each field
#         filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
#         filtered_msg.is_dense = True
#         # filtered_msg.data = np.array(points_above_plane, dtype=np.float32).tobytes()
#         filtered_msg.data = bytes(points_above_plane_bytes)
#         # Publish the filtered points
#         print("new data", filtered_msg.width, filtered_msg.point_step, filtered_msg.row_step, len(filtered_msg.data), len(filtered_msg.data)/filtered_msg.width)
#         self.filtered_pub.publish(filtered_msg)

#     @staticmethod
#     def quaternion_to_rotation_matrix(quaternion):
#         # Convert a quaternion to a rotation matrix
#         x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

#         return np.array([
#             [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
#             [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
#             [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2]
#         ])


# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarPlaneFittingNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
