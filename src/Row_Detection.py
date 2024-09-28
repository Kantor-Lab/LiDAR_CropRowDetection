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
from collections import deque
import matplotlib.pyplot as plt
from change_odom import save_list_to_csv
from Lidar_RowDetect.srv import PointTurn, PointTurnResponse
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
orientation_history = deque(maxlen=10)
last_centroids = []
left_centroids = []
left_centroids_abs = []
right_centroids = []
right_centroids_abs = []
drone_list = []
left_turn = False
def odometry_callback(msg):#get robot filtered odometry
    global i
    global j
    global initial_orientation
    global initial_position
    global orientation_history
    orientation_history.append(msg.pose.pose.orientation)
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
    global left_turn
    global swiped_lines
    mode_pub.publish(mode)
    ranges = [(0, 2.4)]#, (2.3, 3), (3, 3.5)]#(1.8, 2.3), (2.3, 2.8), (2.8, 3.3)] #[(0, 1.3),(1.3, 1.7),(1.7, 2.2), (2.2, 2.6)]
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
    pc_array = np.array(list(pc_data))
    print("length", len(pc_array))
    if mode == 0 and len((pc_array)) < 200:
        if time_to_stop <= 20:
            time_to_stop += 1
            line_fitting = 1
            pass
        else:
            mode = 1
            pass
    if mode == 1 or mode == 2:
        print(mode)
        print("switching rows")
        service_response = service_client(left_turn)
        rospy.loginfo(f"Service called with left={left_turn}. Response: {service_response}")
        if service_response:
            mode = 0
            switched_line = 1
            swiped_lines += 1
            left_turn = not left_turn
            
        
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
        if cluster_marker:
            marker_pub.publish(cluster_marker)
        LIST_pub.publish(list_message)
        value_pub.publish(num_clusters)

    swiped_lines_publisher.publish(swiped_lines)
    change_lane_publisher.publish(switched_line)    
    line_fitting_function.publish(line_fitting)
    
def calculate_kmeans(msg, pc_array,robot_position, robot_orientation, initial_orientation, initial_rotation_matrix):
    
    num_clusters =10  # Adjust the number of clusters as needed
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
    
    global i
    global orientation_history
    now_rotation = quaternion_to_rotation_matrix([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
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
    global drone_list
    detection_error = []

    for centroids in markers:
        for point in centroids:
            cluster_marker.points.append(point)
            drone_list.append([point.x, point.y, point.z])

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
    
def write_points_to_file(points, filename):
    """Write each point's coordinates to a text file, one per line."""
    with open(filename, 'w') as file:
        for point in points:
            # Write the x, y, z coordinates to the file, separated by commas
            file.write(f"{point[0]}, {point[1]}, {point[2]}\n")
            # file.write(f"{point}\n")
        print(f"Points have been written to {filename}")
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
    service_client = rospy.ServiceProxy('point_turn', PointTurn)
    # rospy.Subscriber("/odometry/filtered_zeroed", Odometry, odometry_callback,queue_size= 1)
    # rospy.Subscriber("/odom", Odometry, odometry_callback,queue_size= 1)
    rospy.Subscriber("/points_above_plane", PointCloud2, lidar_callback, marker_pub, queue_size= 1)
    
    rospy.spin()
