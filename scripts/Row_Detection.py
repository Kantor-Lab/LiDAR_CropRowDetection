#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32MultiArray, Int32

import numpy as np
import open3d as o3d
from collections import deque
from scipy.spatial.transform import Rotation as R
# from lidar_rowdetect.srv import PointTurn
import sensor_msgs_py.point_cloud2 as pc2
import time
# from change_odom import save_list_to_csv
from cuml.cluster import KMeans

# Initialize global variables
time_to_stop = 0
robot_position = None
robot_orientation = None
markers = []
fitting_points = []
all_centroids = []
predicted_centroids = []
ground_truth = []
i = 1
q = 1
initial_orientation = None
initial_position = None
mode = 0
switched_line = 0
swiped_lines = 1
number_of_rows = 1
line_fitting = 0
j = 0
orientation_history = deque(maxlen=10)
last_centroids = []
left_centroids = []
right_centroids = []
left_turn = False


class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker_centroids", 1)
        self.LIST_pub = self.create_publisher(Float32MultiArray, "/marker_list", 1)
        self.value_pub = self.create_publisher(Int32, "/number_of_centroids", 1)
        self.mode_pub = self.create_publisher(Int32, "/modes", 1)
        self.line_fitting_function_pub = self.create_publisher(Int32, "/line_function", 1)
        self.swiped_lines_publisher = self.create_publisher(Int32, "/swiped_lines", 1)
        self.change_lane_publisher = self.create_publisher(Int32, "/switch_lines", 1)
        self.endvel_publish = self.create_publisher(Twist, "/cmd_vel", 1)
        
        # Subscribers
        self.create_subscription(Odometry, "/odometry/filtered", self.odometry_callback, 1)
        self.create_subscription(PointCloud2, "/points_above_plane", self.lidar_callback, 1)
        print("here")
        self.declare_parameter('tilt_angle', 0.7)
        
        # Service Client
        self.service_client = self.create_client(PointTurn, "point_turn")

    def send_service_request(self, left_turn):
        # Create a service request object
        request = PointTurn.Request()
        request.left = left_turn

        # Call the service and handle the response asynchronously
        future = self.service_client.call_async(request)
        future.add_done_callback(self.service_response_callback)
        
    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response}")
            else:
                self.get_logger().info("Service call failed")
        except Exception as e:
            self.get_logger().error(f"Service call failed with exception: {e}")
            
    def odometry_callback(self, msg):
        print("here in odom")
        global initial_orientation, initial_position, robot_position, robot_orientation, orientation_history, j
        orientation_history.append(msg.pose.pose.orientation)
        if initial_orientation is None:
            initial_orientation = msg.pose.pose.orientation
            self.get_logger().info("Getting initial orientation")
        elif j == 0:
            initial_position = msg.pose.pose.position
            j = 1
        else:
            robot_position = msg.pose.pose.position
            robot_orientation = msg.pose.pose.orientation
    def lidar_callback(self, msg):
        
        global robot_position, robot_orientation, initial_orientation, initial_position, time_to_stop, mode, switched_line
        global swiped_lines, left_turn, line_fitting

        if robot_position is None or robot_orientation is None or initial_orientation is None or initial_position is None:
            return
        print("here in lidar")
        temp_robot_position = robot_position
        temp_robot_orientation = robot_orientation
        temp_initial_orientation = initial_orientation
        initial_rotation_matrix = self.quaternion_to_rotation_matrix([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
        self.mode_pub.publish(Int32(data=mode))
        ranges = [(0, 2.4)]

        # pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)))
        # pc_array = np.zeros((len(pc_data), 3))
        # for i, point in enumerate(pc_data):
        #     pc_array[i][0] = pc_data[i][0]
        #     pc_array[i][1] = pc_data[i][1]
        #     pc_array[i][2] = pc_data[i][2]
        # pc_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)), dtype=np.float32)
        pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)))
        pc_array = np.vstack([pc_data['x'], pc_data['y'], pc_data['z']]).T.astype(np.float32)

        # print(pc_array.shape)
        # print("First 5 rows:", pc_array[:5])

        # Read raw points from the message

        if mode == 0 and len(pc_array) < 200:
            if time_to_stop <= 20:
                time_to_stop += 1
                line_fitting
                pass
            else:
                mode = 1
                pass
        elif mode == 1 or mode == 2:
            # Create a service request object
            request = PointTurn.Request()
            request.left = left_turn
    
            # Call the service and handle the response asynchronously
            future = self.service_client.call_async(request)
            future.add_done_callback(self.service_response_callback)
            if service_response:
                mode = 0
                switched_line = 1
                swiped_lines += 1
                left_turn = not left_turn

        if mode == 0 and len(pc_array) > 200:
            line_fitting = 0
            time_to_stop = 0
            classified_points = {f"Between {r[0]} and {r[1]}": pc_array[(pc_array[:, 0] >= r[0]) & (pc_array[:, 0] < r[1])] for r in ranges}
            start = time.time()
            for key, value in classified_points.items():
                print(key)
                print(len(value))
                if len(value) < 100:
                    pass
                else:
                    cluster_marker, list_message, num_clusters = self.calculate_kmeans(msg, value, temp_robot_position, temp_robot_orientation, temp_initial_orientation, initial_rotation_matrix)
                end = time.time()
                print("take time:", end - start)
                if cluster_marker:
                    self.marker_pub.publish(cluster_marker)
            self.LIST_pub.publish(list_message)
            # number_of_cluster = Int32()
            # number_of_cluster.data = num_clusters
            self.value_pub.publish(Int32(data=num_clusters))

        self.line_fitting_function_pub.publish(Int32(data=line_fitting))
        self.swiped_lines_publisher.publish(Int32(data=swiped_lines))
        self.change_lane_publisher.publish(Int32(data=switched_line))

    def calculate_kmeans(self, msg, pc_array,robot_position, robot_orientation, initial_orientation, initial_rotation_matrix):
        num_clusters =4  # Adjust the number of clusters as needed
        kmeans = KMeans(n_clusters=num_clusters, n_init= 10, tol = 1e-4, max_iter = 1000, random_state=0).fit(pc_array)
        
        # Get cluster labels for each point
        cluster_labels = kmeans.predict(pc_array)
        centroids = kmeans.cluster_centers_
        centroids = sorted(centroids, key=lambda x: x[1])
        centroids = self.kmeans_filter(centroids=centroids, threshold= 0.3)
        
        # Create markers for each cluster centroid
        global predicted_centroids
        predicted_centroids.append(len(centroids))
        # print(centroids)
        if len(predicted_centroids) < 9:
            num_clusters = max(set(predicted_centroids), key = predicted_centroids.count)
        else:
            num_clusters = max(set(predicted_centroids[-9:]), key = predicted_centroids.count)
        print("most common", num_clusters)
        print("centroids", centroids)
        cluster_marker = Marker()
        cluster_marker.header = msg.header
        cluster_marker.type = Marker.POINTS
        cluster_marker.action = Marker.ADD
        cluster_marker.scale.x = 0.2  # Adjust the marker size as needed
        cluster_marker.scale.y = 0.2
        cluster_marker.scale.z = 0.2
        cluster_marker.color.a = 1.0  # Fully opaque
        cluster_marker.color.r = 1.0  # Red
        cluster_marker.color.g = 0.0
        cluster_marker.color.b = 0.0

        new_centroids = []
        points = []
        
        global i
        global orientation_history
        now_rotation = self.quaternion_to_rotation_matrix([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
        rotation_matrix = np.dot(now_rotation, np.linalg.inv(initial_rotation_matrix))

        ## Accomodate wheel slipping
        # if i == 1:
        #     rotation_matrix = np.dot(now_rotation, np.linalg.inv(initial_rotation_matrix))
        #     i = 0
        # elif orientation_history is not None:
        #     previous_orientation = quaternion_to_rotation_matrix([orientation_history[0].x, orientation_history[0].y, orientation_history[0].z, orientation_history[0].w ])
        #     rotation_matrix = np.dot(now_rotation, np.linalg.inv(previous_orientation))
        global swiped_lines
        T = np.array([robot_position.x, robot_position.y, robot_position.z])
        for point in centroids:
            point[0] = point[0] * np.cos(0.7) + 1 #0.7 is lidar tilted angle
            point[2] = 0 
            P = np.array([point[0], point[1], point[2]])
            new_point = now_rotation @P + T
            p = Point()
            p.x = new_point[0]
            p.y = new_point[1] 
            p.z = new_point[2] 
            new_centroids.append(p)
            points.append(new_point)
            
        global markers
        markers.append(new_centroids)
        global fitting_points
        global switched_line

        if switched_line == 1:
            fitting_points = []
            # i = 1
            initial_orientation = None
            switched_line = 0
        
        fitting_points.append(points)
        global left_centroids
        global left_centroids_abs
        global right_centroids
        global right_centroids_abs
        # global drone_list
        detection_error = []

        for centroids in markers:
            for point in centroids:
                cluster_marker.points.append(point)
        #         drone_list.append([point.x, point.y, point.z])

        #### Save detected centroids location       
        # print("drone list", len(drone_list))
        # print("last element", drone_list[-1])
        # file_path = "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/drone_map_txt/sim_curvecorn_test.txt"
        # if robot_position.y > 270:
        # if robot_position.y < 4651800: #4651667:
        # if robot_position.x > 50:
        #     write_points_to_file(drone_list, file_path)
        
        fitting_centroids = []
        for row in fitting_points:
            # sort the detected centroids with y value from small to large
            global last_centroids
            distances = [(sublist, (sublist[1]-robot_position.y)) for sublist in row if len(sublist) > 1]
            sorted_distances = sorted(distances, key=lambda x: abs(x[1]))

            if len(sorted_distances) >= 2 and len(sorted_distances) % 2 == 1: #pass if the number of detected centroids is odd for now
                pass
            elif len(sorted_distances) >= 2 and len(sorted_distances) % 2 == 0:
                closest_elements = [sorted_distances[0][0], sorted_distances[1][0]]
            elif len(sorted_distances) == 1:
                difference_1 = abs(last_centroids[0][1] - sorted_distances[0][0][1])
                difference_2 = abs(last_centroids[1][1] - sorted_distances[0][0][1])
                if difference_1 > difference_2:
                    closest_elements = [sorted_distances[0][0], last_centroids[0]]
                else:
                    closest_elements = [sorted_distances[0][0], last_centroids[1]]
            elif len(sorted_distances) == 0:
                closest_elements = last_centroids
            closest_elements = sorted(closest_elements, key=lambda x:x[1])
            
            if closest_elements:
                # for i in range(len(closest_elements)):
                for element in closest_elements:
                    # element[1] += robot_position.y
                    fitting_centroids.append(element[:2])
            last_centroids = closest_elements
                
        flat_data = [item for sublist in fitting_centroids for item in sublist]
        list_message = Float32MultiArray(data=flat_data)
        return cluster_marker, list_message, num_clusters
    def quaternion_to_rotation_matrix(self,quaternion):
        # Convert a Quaternion to a 3x3 rotation matrix
        x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

        rotation_matrix = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

        return rotation_matrix
    def write_points_to_file(self,points, filename):
        """Write each point's coordinates to a text file, one per line."""
        with open(filename, 'w') as file:
            for point in points:
                # Write the x, y, z coordinates to the file, separated by commas
                file.write(f"{point[0]}, {point[1]}, {point[2]}\n")
                # file.write(f"{point}\n")
            print(f"Points have been written to {filename}")
    def kmeans_filter(self, centroids, threshold):
        merge = True
        while merge:
            new_centroids = []
            delete_centroids = []
            # print("centroids", centroids)
            for i in range(len(centroids)-1):
                distance = np.abs(centroids[i][1] - centroids[i+1][1])
                # print(distance)
                if distance > threshold:
                    new_centroids.append(centroids[i])
                    if np.abs(centroids[i][1] - centroids[i-1][1]) < threshold:
                        new_centroids.pop()
                        if i >= 1 and i == len(centroids) -2:
                            new_centroids.append(centroids[i+1])
                    elif i >= 1 and i == len(centroids) -2 and np.abs(centroids[i][1] - centroids[i-1][1]) >= threshold: 
                        new_centroids.append(centroids[i+1])
                elif distance <= threshold:
                    new_centroids.append(np.mean([np.array(centroids[i]),np.array(centroids[i+1])], axis = 0))
                    delete_centroids.append(1)
            if delete_centroids:
                print("distance between centroids too large;Continue")
                centroids = new_centroids
            else:
                merge = False
                print("quit")
        
        return centroids
def main():
    rclpy.init()
    node = LidarProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
