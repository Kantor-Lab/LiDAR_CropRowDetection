#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
import numpy as np
import csv
import os


class GlobalToLocalOdometryNode(Node):
    def __init__(self):
        super().__init__('global_to_local_odometry_node')
        self.global_offset = None
        self.initial_position = None
        self.markers = []
        self.robot_location = []
        self.total_distmae = []
        self.i = 1

        # Subscriber and Publisher
        self.global_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_position_callback,
            10
        )
        self.marker_pub = self.create_publisher(Marker, "/robot_position", 10)

    def save_list_to_csv(self, file_name, data, directory=None):
        if directory:
            os.makedirs(directory, exist_ok=True)
            file_path = os.path.join(directory, file_name)
        else:
            file_path = file_name

        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            if isinstance(data, (np.float64, float, int)):
                writer.writerow([data])
            elif isinstance(data, (list, np.ndarray)):
                for item in data:
                    writer.writerow([item])
        self.get_logger().info(f"List saved to {file_path}")

    def odom_position_callback(self, msg):
        cluster_marker = Marker()
        cluster_marker.header = Header()
        cluster_marker.header.stamp = self.get_clock().now().to_msg()
        cluster_marker.header.frame_id = "velodyne"
        cluster_marker.type = Marker.POINTS
        cluster_marker.action = Marker.ADD
        cluster_marker.scale.x = 0.2
        cluster_marker.scale.y = 0.2
        cluster_marker.scale.z = 0.2
        cluster_marker.color.a = 1.0
        cluster_marker.color.r = 0.0
        cluster_marker.color.g = 1.0
        cluster_marker.color.b = 0.0
        new_centroids = []

        if self.i == 1:
            self.initial_position = msg.pose.pose.position
            self.i += 1
        else:
            quaternion = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            _, _, yaw = euler_from_quaternion(quaternion)
            self.robot_location.append([msg.pose.pose.position.y, msg.pose.pose.position.x, yaw])
            self.total_distmae.append(np.abs(msg.pose.pose.position.y - 0.381))

            p1 = Point()
            p1.x = msg.pose.pose.position.x
            p1.y = msg.pose.pose.position.y
            p1.z = 0.0
            new_centroids.append(p1)

            p2 = Point()
            p2.x = msg.pose.pose.position.x
            p2.y = msg.pose.pose.position.y
            p2.z = 0.0
            new_centroids.append(p2)

            self.markers.append(new_centroids)
            for centroids in self.markers:
                for point in centroids:
                    cluster_marker.points.append(point)

            self.marker_pub.publish(cluster_marker)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalToLocalOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
