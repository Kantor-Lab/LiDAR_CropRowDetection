#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int32, Header
from geometry_msgs.msg import Point, Twist
from sklearn.linear_model import RANSACRegressor
import numpy as np
# from cuml.cluster import KMeans
# from LiDAR_CropRowDetection.msg import Line2Pts, LineList
from robot_interfaces.msg import Line2Pts, LineList
from scipy.cluster.hierarchy import linkage, fcluster
# import tf_transformations
import math
from rclpy.time import Time
# from change_odom import save_list_to_csv  # Ensure this is ROS 2 compatible



def quaternion_to_rotation_matrix(quaternion):
    # Converts a Quaternion to a 3x3 rotation matrix
    x, y, z, w = quaternion
    rotation_matrix = np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]
    ])
    return rotation_matrix


class LineFittingNode(Node):
    def __init__(self):
        super().__init__('ransac_line_fitting')

        # ROS 2 publishers
        self.line_pub = self.create_publisher(Marker, '/line_marker', 10)
        self.lines_pub = self.create_publisher(LineList, '/lines', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ROS 2 subscribers
        # self.create_subscription(Float32MultiArray, "/marker_list", self.list_callback, 1)
        self.create_subscription(Float32MultiArray, "/marker_list", self.publish_lines_callback, 1)
        self.create_subscription(Int32, "/modes", self.mode_callback, 1)
        self.create_subscription(Int32, "/line_function", self.line_function_callback, 1)
        self.create_subscription(Int32, "/swiped_lines", self.swiped_lines_callback, 1)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 1)

        # Timer for periodic callbacks
        # self.timer = self.create_timer(0.2, self.publish_lines_callback)

        # State variables
        self.angle_error = []
        self.fitting_centroids = None

        self.global_robot_position = None
        self.i = 1
        self.initial_position = None
        self.initial_orientation = None
        self.robot_orientation = None
        self.mode = None
        self.line_fitting = None
        self.swiped_lines = None
        self.total_mae = []
        self.total_std = []
        self.total_rmse = []
        self.max_value = 0

    def odom_callback(self, msg):
        self.get_logger().info("In the odom callback")
        """Process Odometry messages to determine robot's current position and orientation."""
        if self.i == 1:
            self.initial_position = msg.pose.pose.position
            self.initial_orientation = msg.pose.pose.orientation
            self.i += 1
        else:
        # Extract the robot's position from the odometry message
            self.global_robot_position = msg.pose.pose.position
            # global_robot_position.x += 1
            self.robot_orientation = msg.pose.pose.orientation

    def mode_callback(self, msg):
        """Handle mode changes."""
        self.mode = msg.data
    def line_function_callback(self, msg):
        """Handle line fitting mode switching."""
        self.line_fitting = msg.data

    def swiped_lines_callback(self, msg):
        """Handle swiped lines."""
        self.swiped_lines = msg.data

    def list_callback(self, msg):
        """Process the detected centroid list."""
        self.fitting_centroids = []
        data = msg.data
        for i in range(0, len(data), 2):
            point = (data[i], data[i + 1], 0.0)  # Append a 0.0 value to match expected centroid format
            self.fitting_centroids.append(point)

    def publish_lines_callback(self, msg):
        """Logic for line processing, robot behavior, and visualization."""
        # Initialize visualization marker
        self.get_logger().info("In the publish lines callback")
        line_marker = Marker()
        line_marker.header.frame_id = 'velodyne'
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0

        self.fitting_centroids = []
        data = msg.data
        for i in range(0, len(data), 2):
            point = (data[i], data[i + 1], 0.0)  # Append a 0.0 value to match expected centroid format
            self.fitting_centroids.append(point)
        
        # print("here in callback", self.initial_orientation, self.robot_orientation, self.mode)
        # Ensure robot positioning and sensor data is ready
        if not self.global_robot_position or not self.fitting_centroids:
            return

        if not self.initial_orientation or not self.robot_orientation or self.mode is None:
            return
        
        # Mode-dependent behavior
        if self.mode == 1 or self.mode == 2:
            pass
        elif self.mode == 0 and self.line_fitting == 0:
            # Handle initial line-following mode
            robot_position = self.global_robot_position

            # Convert the centroids into numpy array
            fitting_centroids_np = np.array(self.fitting_centroids)
            now_rotation = quaternion_to_rotation_matrix([
                self.robot_orientation.x,
                self.robot_orientation.y,
                self.robot_orientation.z,
                self.robot_orientation.w,
            ])
            T = np.array([robot_position.x, robot_position.y, robot_position.z])

            # Handle swiped_lines logic
            # if self.swiped_lines and self.swiped_lines % 2 == 1:
            left_x1 = self.global_to_local(self.fitting_centroids[-2:][1], T, now_rotation)[0]
            left_y1 = self.fitting_centroids[-2:][1][1] - robot_position.y
            left_x2 = 0.0
            left_y2 = self.fitting_centroids[-2:][1][1] - robot_position.y

            right_x1 = self.global_to_local(self.fitting_centroids[-2:][0], T, now_rotation)[0]
            right_x2 = 0.0
            right_y1 = self.fitting_centroids[-2:][0][1] - robot_position.y
            right_y2 = self.fitting_centroids[-2:][0][1] - robot_position.y

            # Determine minimal line index checks
            min_left = 0.1
            min_right = 0.1
            min_left_index = None
            min_right_index = None
            for i in range(0, len(self.fitting_centroids) - 1, 2):
                local_left_x = self.global_to_local(self.fitting_centroids[i + 1], T, now_rotation)[0]
                local_right_x = self.global_to_local(self.fitting_centroids[i], T, now_rotation)[0]

                if 0.0 < local_left_x < min_left:
                    min_left_index = i + 1
                    min_left = local_left_x
                    left_x2 = local_left_x

                if 0.0 < local_right_x < min_right:
                    min_right_index = i
                    min_right = local_right_x
                    right_x2 = local_right_x
            if not min_left_index and not min_right_index:
                self.get_logger().info("Moving straight...")
                vel_msg = Twist()
                vel_msg.linear.x = 0.2
                self.cmd_vel_pub.publish(vel_msg)

            if min_left_index:
                self.get_logger().info("Left side detected.")
                left_line_centroids = [self.fitting_centroids[idx] for idx in range(min_left_index, len(self.fitting_centroids), 2)]
                left_line_centroids = sorted(left_line_centroids, key=lambda x: x[0])
                if len(left_line_centroids) > 5:
                    reg_left, m_left, b_left = self.Ransac_line_fit(left_line_centroids)
                    self.publish_fitting_line(left_line_centroids, reg_left, line_marker)
                left_x1 = np.mean([self.global_to_local(point, T, now_rotation)[0] for point in left_line_centroids[-20:]], axis=0)
                left_y1 = left_line_centroids[-1][1] - robot_position.y
                left_y2 = left_line_centroids[0][1] - robot_position.y
                # left_y1 = reg_left.predict(np.array(left_x1).reshape(-1,1))[0]- robot_position.y
                # left_y2 = reg_left.predict(np.array(left_x2).reshape(-1,1))[0]- robot_position.y
            if min_right_index:
                self.get_logger().info("Right side detected.")
                right_line_centroids = [self.fitting_centroids[idx] for idx in range(min_right_index, len(self.fitting_centroids), 2)]
                right_line_centroids = sorted(right_line_centroids, key=lambda x: x[0])

                if len(right_line_centroids) > 5:
                    reg_right, m_right, b_right = self.Ransac_line_fit(right_line_centroids)
                    self.publish_fitting_line(right_line_centroids, reg_right, line_marker)
                right_x1 = np.mean([self.global_to_local(point, T, now_rotation)[0] for point in right_line_centroids[-20:]], axis=0)
                right_y1 = right_line_centroids[-1][1] - robot_position.y
                right_y2 = right_line_centroids[0][1] - robot_position.y
                # right_y1 = reg_right.predict(np.array(right_x1).reshape(-1,1))[0]- robot_position.y
                # right_y2 = reg_right.predict(np.array(right_x2).reshape(-1,1))[0]- robot_position.y

            # Publish visualization to RViz
            self.line_pub.publish(line_marker)

            if min_left_index or min_right_index:
                # Create line visualization data
                msg = LineList()
                line_1 = Line2Pts(x1=right_x1, y1=right_y1,
                                        x2=right_x2 , y2=right_y2)
                line_2 = Line2Pts(x1=left_x1, y1=left_y1,
                                        x2=left_x2, y2=left_y2)
                msg.lines = [line_1, line_2]
                msg.num_lines = 2
                msg.header = Header(stamp=Time().to_msg(), frame_id="Lidar_detect")
                self.lines_pub.publish(msg)
        self.get_logger().info("Processed line fitting and movement logic.")


    def Ransac_line_fit(self,centroids):
        X = []
        Y = []
        for row in centroids:
            X.append(row[0])
            Y.append(row[1])
        X= np.array(X).reshape(-1,1)
        Y = np.array(Y)
        reg = RANSACRegressor(min_samples=2, random_state=0,max_trials=1000).fit(X,Y)
        slope = reg.estimator_.coef_[0]
        bias = reg.estimator_.intercept_
        return reg, slope, bias
    def global_to_local(self,global_point, local_origin, local_orientation):
        translated_point = global_point - local_origin
        
        # Rotate the point
        local_point = np.dot(local_orientation.T, translated_point)
        
        return local_point
    def publish_fitting_line(self,centroids, reg, line_marker):
        
        if len(centroids) > 4:
                    
                # Define two points to form the line
            p1 = Point()
            p1_x = np.array(centroids[0][0]).reshape(-1,1)
            p1.x = float(p1_x)
            p1.y = float(reg.predict(p1_x)[0])
            p1.z = 0.2  # Assuming a 2D line

            p2 = Point()
            p2_x = np.array(centroids[-1][0]).reshape(-1,1)  # Adjust the ending x-coordinate as needed
            p2.x = float(p2_x)
            p2.y = float(reg.predict(p2_x)[0])
            p2.z = 0.2
        
            line_marker.points.append(p1)
            line_marker.points.append(p2)
    def spin(self):
        """Main loop for ROS 2 node."""
        self.get_logger().info("Starting main spin loop...")
        # while rclpy.ok():
        #     print("it is ok")
        #     self.publish_lines_callback()
        rclpy.spin(self)
            # time.sleep(0.1)


def main():
    # Initialize ROS 2
    rclpy.init()

    # Instantiate and run the node
    node = LineFittingNode()
    try:
        while rclpy.ok():
            node.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()