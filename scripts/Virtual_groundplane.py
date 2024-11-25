#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sklearn.linear_model import RANSACRegressor
from visualization_msgs.msg import Marker
import numpy as np
# from tf_transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R
from numpy.linalg import svd


class LidarPlaneFittingNode(Node):
    def __init__(self):
        super().__init__('lidar_plane_fitting')
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/plane_marker', 1)
        self.filtered_pub = self.create_publisher(PointCloud2, '/points_above_plane', 1)

        # Subscriber
        self.create_subscription(PointCloud2, '/filtered_lidar_points', self.lidar_callback, 1)

    def lidar_callback(self, msg):
        # Convert the PointCloud2 message to a NumPy array
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_array = np.array(list(pc_data))

        x = pc_array[:, 0]
        y = pc_array[:, 1]
        z = pc_array[:, 2]

        above_plane = 0.02  # Adjust as needed

        # Create a Marker message for the plane
        plane_marker = Marker()
        plane_marker.header = msg.header
        plane_marker.type = Marker.CUBE
        plane_marker.action = Marker.ADD
        plane_marker.pose.position.x = np.mean(x)
        plane_marker.pose.position.y = np.mean(y)
        plane_marker.pose.position.z = np.mean(z) + above_plane

        q1 = R.from_euler('xyz', [0, -0.7, 0]).as_quat()  # Modify angles as needed

        plane_marker.pose.orientation.x = q1[0]
        plane_marker.pose.orientation.y = q1[1]
        plane_marker.pose.orientation.z = q1[2]
        plane_marker.pose.orientation.w = q1[3]
        plane_marker.scale.x = 10.0  # Adjust the scale as needed
        plane_marker.scale.y = 10.0
        plane_marker.scale.z = 0.001
        plane_marker.color.a = 0.5  # Adjust the transparency
        plane_marker.color.r = 0.0
        plane_marker.color.g = 1.0
        plane_marker.color.b = 0.0

        # Publish the Marker message
        self.marker_pub.publish(plane_marker)

        min_angle = np.radians(-90)
        max_angle = np.radians(90)
        points_above_plane = []
        fields = msg.fields
        num_fields = len(fields)
        point_step = num_fields * 2  # Assuming FLOAT32 fields

        rotation_matrix = self.quaternion_to_rotation_matrix(plane_marker.pose.orientation)

        # Determine the normal vector of the plane using the rotation matrix
        normal_vector = np.dot(rotation_matrix, np.array([0, 0, 1]))
        normal_vector /= np.linalg.norm(normal_vector)  # Normalize
        d = -np.dot(normal_vector, np.array([np.mean(x), np.mean(y), np.mean(z)]))
        a, b, c = normal_vector

        # Iterate through the LiDAR points
        for point in pc2.read_points(msg, field_names=("x", "y", "z")):
            point_angle = point[1]

            # Check if the point's angle is within the desired range
            if min_angle <= point_angle <= max_angle and point[0] <= 20:
                if a * point[0] + b * point[1] + c * point[2] + d > above_plane:
                    points_above_plane.append(point)

        # Create a new PointCloud2 message for the filtered points
        self.get_logger().info(f"Points above plane: {len(points_above_plane)}")
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = 1
        filtered_msg.width = len(points_above_plane)
        filtered_msg.fields = fields  # Use the same fields as the original message
        filtered_msg.is_bigendian = False
        filtered_msg.point_step = point_step
        filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
        filtered_msg.is_dense = True
        filtered_msg.data = np.array(points_above_plane, dtype=np.float32).tobytes()

        # Publish the filtered points to a new topic
        self.filtered_pub.publish(filtered_msg)

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        rotation_matrix = np.array([
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]
        ])
        return rotation_matrix


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
