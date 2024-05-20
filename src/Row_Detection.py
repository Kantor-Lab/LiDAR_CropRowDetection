#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sklearn.cluster import OPTICS
import numpy as np
from geometry_msgs.msg import Point
from scipy.cluster.hierarchy import linkage, fcluster
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
# import torch
import time
from cuml.cluster import KMeans
import open3d as o3d
import tf
from sklearn.metrics import r2_score
from geometry_msgs.msg import Twist
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
previous_seq = 0
now_seq = 0
def odometry_callback(msg):
    global i
    global j
    global initial_orientation
    global initial_position
    global initial_rotation_matrix
    if initial_orientation is None:
        initial_orientation = msg.pose.pose.orientation
        print("getting initial orientation")
        # i = 0
    elif j == 0:
        initial_position = msg.pose.pose.position
        j = 1
    else:
        global robot_position
    # Extract the robot's position from the odometry message
        robot_position = msg.pose.pose.position
        global robot_orientation
        robot_orientation = msg.pose.pose.orientation
def odometry_callback1(msg):
    global q  
    global initial_orientation
    if q == 1:
        initial_orientation = msg.pose.pose.orientation
        q += 1
    else:
        global robot_orientation
        robot_orientation = msg.pose.pose.orientation

def lidar_callback(msg, marker_pub):
    global robot_position
    if robot_position is None:
        return
    global robot_orientation
    if robot_orientation is None:
        return
    global initial_orientation
    if initial_orientation is None:
        return
    global initial_position
    if initial_position is None:
        return
    temp_robot_position = robot_position
    temp_robot_orientation = robot_orientation
    temp_initial_orientation = initial_orientation
    initial_rotation_matrix = quaternion_to_rotation_matrix([temp_initial_orientation.x, temp_initial_orientation.y, temp_initial_orientation.z, temp_initial_orientation.w ])
    global time_to_stop
    global mode
    global switched_line
    global line_fitting
    mode_pub.publish(mode)
    ranges = [(0, 2.2)]#, (2.3, 3), (3, 3.5)]#(1.8, 2.3), (2.3, 2.8), (2.8, 3.3)] #[(0, 1.3),(1.3, 1.7),(1.7, 2.2), (2.2, 2.6)]
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_data))
    if mode == 0 and len((pc_array)) < 200:
        if time_to_stop <= 80:
            time_to_stop += 1
            line_fitting = 1
            pass
        else:
            mode = 1
            pass
    if mode == 1 or mode == 2:
        print(mode)
        print("switching rows")
        turning(initial_orientation, robot_orientation, initial_position, robot_position)
        
    if mode == 0 and len((pc_array)) > 200:
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
                cluster_marker, list_message, num_clusters = calculate_kmeans(msg, value, temp_robot_position, temp_robot_orientation, temp_initial_orientation, initial_rotation_matrix)
        end = time.time()
        print("take time:", end - start)
        marker_pub.publish(cluster_marker)
        LIST_pub.publish(list_message)
        value_pub.publish(num_clusters)
        ### calculate MAE or RMSE
        # if temp_robot_position.x > 40:
        #     crop_location = []
        #     num_rows=4 
        #     bushes_per_row=400
        #     bush_spacing=0.1
        #     row_spacing=0.762
        #     for row in range(num_rows):
        #         # y = row * row_spacing + 0.133716
        #         y = row * row_spacing 
        #         # y = row * row_spacing - 1.6502027
        #         z = 0
        #         for bush in range(40,bushes_per_row):
        #             # x = bush * bush_spacing + 0.510412
        #             x = bush * bush_spacing 
        #             # x = bush * bush_spacing + 1.873750
        #             temp = []
        #             temp.append(x)
        #             temp.append(y)
        #             temp.append(z)
        #             crop_location.append(temp)
            
        #     std, mae = calculate_mae(crop_location, fitting_points)
        #     print("standard deviation:", std)
        #     print("MAE is:", mae ) 
            # rmse = calculate_rmse(crop_location, fitting_points)
            # print("RMSE is:", rmse)
    swiped_lines_publisher.publish(swiped_lines)
    change_lane_publisher.publish(switched_line)    
    line_fitting_function.publish(line_fitting)
    
def calculate_kmeans(msg, pc_array,robot_position, robot_orientation, initial_orientation, initial_rotation_matrix):
    
    num_clusters =4  # Adjust the number of clusters as needed
    kmeans = KMeans(n_clusters=num_clusters, n_init= 10, tol = 1e-4, max_iter = 1000, random_state=0).fit(pc_array)
    
    # Get cluster labels for each point
    cluster_labels = kmeans.predict(pc_array)
    centroids = kmeans.cluster_centers_
    centroids = sorted(centroids, key=lambda x: x[1])
    centroids = kmeans_filter(centroids=centroids, threshold= 0.3)
     
    # Create markers for each cluster centroid
    global predicted_centroids
    predicted_centroids.append(len(centroids))
    # print(centroids)
    if len(predicted_centroids) < 9:
        num_clusters = max(set(predicted_centroids), key = predicted_centroids.count)
    else:
        # print(predicted_centroids[-9])
        num_clusters = max(set(predicted_centroids[-9:]), key = predicted_centroids.count)
    print("most common", num_clusters)
    
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
    
    now_rotation = quaternion_to_rotation_matrix([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
    rotation_matrix = np.dot(now_rotation, np.linalg.inv(initial_rotation_matrix))

    global swiped_lines
    T = np.array([robot_position.x, robot_position.y, robot_position.z])
    for point in centroids:
        point[0] = point[0] * np.cos(0.7) + 1 
        point[2] = 0 
        P = np.array([point[0], point[1], point[2]])
        new_point = rotation_matrix @ P + T
        points.append(new_point)
        p = Point()
        p.x = new_point[0] #+robot_position.y
        p.y = new_point[1] #+robot_position.x
        p.z = new_point[2] 
        new_centroids.append(p)
    global markers
    markers.append(new_centroids)
    global fitting_points
    global switched_line
    global i
    if switched_line == 1:
        fitting_points = []
        # i = 1
        initial_orientation = None
        switched_line = 0
    fitting_points.append(points)
    for centroids in markers:
        for point in centroids:
            cluster_marker.points.append(point)
    # marker_pub.publish(cluster_marker)
    fitting_centroids = []
    for row in fitting_points:
        if swiped_lines % 2 == 1:
            row = [arr - [0, robot_position.y, 0] for arr in row]
            neg_num = min((n[1] for n in row if n[1] < 0), key=lambda x: abs(x))
        
            pos_num = next(n[1] for n in row if n[1] > 0)
            for i in range(len(row)):
                if row[i][1] == neg_num or row[i][1] == pos_num:
                    temp = []
                    row[i][1] += robot_position.y
                    for element in row[i][:2]:
                        temp.append(element)
                    fitting_centroids.append(temp)
        else:
            row = [[arr[0], robot_position.y - arr[1], arr[2]] for arr in row]
        # print(row)
            neg_num = min((n[1] for n in row if n[1] < 0), key=lambda x: abs(x))
        # print("neg", neg_num)
            pos_num = next(n[1] for n in row if n[1] > 0)
            for i in range(len(row)):
                if row[i][1] == neg_num or row[i][1] == pos_num:
                    temp = []
                    row[i][1] = -(row[i][1] - robot_position.y)
                    for element in row[i][:2]:
                        temp.append(element)
                    fitting_centroids.append(temp)
        

    flat_data = [item for sublist in fitting_centroids for item in sublist]
    list_message = Float32MultiArray(data=flat_data)
    return cluster_marker, list_message, num_clusters
    

def kmeans_filter(centroids, threshold):
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
def calculate_mae(ground_truth, fitting_points):
    total_distance = 0
    max_value = 0
    error = []
    fitting_centroids = []
    for row in fitting_points:
        for i in range(len(row)):
            temp = []
            for element in row[i][:2]:
                temp.append(element)
                fitting_centroids.append(temp)
    for i in range(len(ground_truth)):
        item = ground_truth[i][:2]
        distances = []
        for centroids in fitting_centroids:
            distance = np.linalg.norm(np.array(item)-np.array(centroids))
            distances.append(distance)
        min_distance = min(distances)
        if min_distance > max_value:
            max_value = min_distance
        error.append(min_distance)
        total_distance += min_distance
    error = np.array(error)
    mae = np.mean(error)
    std = np.std(error)
    print("max deviation:", max_value)
    return std, mae
def calculate_rmse(ground_truth, fitting_points):
    total_distance = 0
    fitting_centroids = []
    for row in fitting_points:
        # print("neg & pos numbers:", neg_num, pos_num)
        for i in range(len(row)):
            temp = []
            for element in row[i][:2]:
                temp.append(element)
                fitting_centroids.append(temp)
    # print("length of fitting centroids:", len(fitting_centroids))
    for i in range(len(ground_truth)):
        item = ground_truth[i][:2]
        distances = []
        for centroids in fitting_centroids:
            distance = np.linalg.norm(np.array(item)-np.array(centroids))
            distances.append(distance)
        min_distance = min(distances)
        total_distance += min_distance ** 2
    return np.sqrt(total_distance / len(ground_truth))
def turning(initial_orientation, robot_orientation, initial_position, robot_position):
    initial_euler = tf.transformations.euler_from_quaternion([initial_orientation.x, initial_orientation.y, initial_orientation.z, initial_orientation.w ])
    now_euler = tf.transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
    turn_degree = now_euler[2]-initial_euler[2]
    print("turn degree", turn_degree)
    right_turn_msg = Twist()
    right_turn_msg.angular.z = 0.5  # Angular velocity for turning
    left_turn_msg = Twist()
    left_turn_msg.angular.z = -0.5

    straight_msg = Twist()
    straight_msg.linear.x = 0.2
    straight_distance = 1.524
    global mode
    global j
    global i
    global swiped_lines
    global switched_line
    global number_of_rows
    if number_of_rows == 1:
        print("turning right")
        if mode == 1 and -1.571 <= turn_degree <= 0.1:
            rospy.loginfo("Turning")
                # 20 iterations for turning
            endvel_publish.publish(right_turn_msg)
        # print(mode)
        if mode == 1 and -3.12 <= turn_degree <= -1.571:
            rospy.loginfo("Straight")
            endvel_publish.publish(straight_msg)
            print("robot y", robot_position.y)
            print("initial y", initial_position.y)
            if np.abs(robot_position.y - initial_position.y) >= straight_distance:
                mode = 2
            print(mode)
        if mode == 2 and -3.12 <= turn_degree <= -1.571:
            rospy.loginfo("Turning again")
            endvel_publish.publish(right_turn_msg)
        if  turn_degree < -3.12 or turn_degree > 3.12:    
            stop_msg = Twist()
            stop_msg.linear.x = 0
            mode = 0
            swiped_lines += 1
            switched_line = 1
            number_of_rows = 0
            j = 0
            endvel_publish.publish(stop_msg)
    else:
        print("turning left")
        if mode == 1 and 1.571 <= turn_degree <= 3.14 or -3.14 <= turn_degree <= -1.571:
            rospy.loginfo("Turning")
                # 20 iterations for turning
            endvel_publish.publish(left_turn_msg)
        if mode == 1 and -1.571 <= turn_degree <= -0.02:
            rospy.loginfo("Straight")
            endvel_publish.publish(straight_msg)
            if np.abs(robot_position.y - initial_position.y) >= straight_distance:
                mode = 2
            print(mode)
        if mode == 2 and -1.571 <= turn_degree <= -0.02:
            rospy.loginfo("Turning again")
            endvel_publish.publish(left_turn_msg)
        if -0.02 < turn_degree < 0.1: #or turn_degree  -3.1:    
            stop_msg = Twist()
            stop_msg.linear.x = 0
            mode = 0
            swiped_lines += 1
            switched_line = 1
            number_of_rows = 1
            # i = 1
            j = 0
            endvel_publish.publish(stop_msg)
        # rospy.signal_shutdown('End of lane Detected')
def quaternion_to_rotation_matrix(quaternion):
    # Convert a Quaternion to a 3x3 rotation matrix
    x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

    rotation_matrix = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

    return rotation_matrix
if __name__ == "__main__":
    rospy.init_node("lidar_meanshift_clustering", anonymous=True)
    marker_pub = rospy.Publisher("/visualization_marker_centroids", Marker, queue_size=1)
    LIST_pub = rospy.Publisher('/marker_list', Float32MultiArray, queue_size=1)
    value_pub = rospy.Publisher('/number_of_centroids', Int32, queue_size=1)
    mode_pub = rospy.Publisher('/modes', Int32, queue_size=1)
    line_fitting_function = rospy.Publisher('/line_function', Int32, queue_size=1)
    swiped_lines_publisher = rospy.Publisher('/swiped_lines', Int32, queue_size=1)
    change_lane_publisher = rospy.Publisher('/switch_lines', Int32, queue_size=1)
    endvel_publish = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback,queue_size= 1)
    rospy.Subscriber("/points_above_plane", PointCloud2, lidar_callback, marker_pub, queue_size= 1)
    
    
    rospy.spin()
