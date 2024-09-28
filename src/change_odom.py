#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
import csv
import os
from tf.transformations import euler_from_quaternion
i = 1
initial_position = None
markers = []
robot_position = None
total_distmae = []
total_std = []
robot_location = []
def global_to_local_odometry(global_odom):
    local_odom = Odometry()

    local_odom.header = global_odom.header
    local_odom.pose = global_odom.pose
    local_odom.twist = global_odom.twist
    print(global_offset.pose.pose.position.y)
    if global_offset is not None:
        local_odom.pose.pose.position.x = global_offset.pose.pose.position.x - local_odom.pose.pose.position.x
        local_odom.pose.pose.position.y = global_offset.pose.pose.position.y - local_odom.pose.pose.position.y

    return local_odom
def save_list_to_csv(file_name, data, directory = None):
    if directory:
        os.makedirs(directory, exist_ok=True)
        file_path = os.path.join(directory, file_name)
    else:
        file_path = file_name

    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Check if data is a single value or a list of values
        if isinstance(data, (np.float64, float, int)):
            writer.writerow([data])
        elif isinstance(data, (list, np.ndarray)):
            # Ensure each element is written as a separate row
            for item in data:
                writer.writerow([item])
    print(f"List saved to {file_path}")

def odom_position(msg):
    global i 
    global initial_position
    cluster_marker = Marker()
    header = Header()
    header.seq = 0  # Assign a proper sequence value here
    header.stamp = rospy.Time.now()  # Add the appropriate timestamp
    header.frame_id = "velodyne" 
    cluster_marker.header = header
    cluster_marker.type = Marker.POINTS
    cluster_marker.action = Marker.ADD
    cluster_marker.scale.x = 0.2  # Adjust the marker size as needed
    cluster_marker.scale.y = 0.2
    cluster_marker.scale.z = 0.2
    cluster_marker.color.a = 1.0  # Fully opaque
    cluster_marker.color.r = 0.0  # Red
    cluster_marker.color.g = 1.0
    cluster_marker.color.b = 0.0
    new_centroids = []

    if i == 1:
        initial_position = msg.pose.pose.position
        i += 1
    else:
        # global robot_position
    # Extract the robot's position from the odometry message
        # robot_position.x += 1
        # robot_position.y -= 0.8
        # robot_position.x = -robot_position.x + initial_position.x
        # robot_position.y = -robot_position.y + initial_position.y
        # robot_position.z = -robot_position.z + initial_position.z
        global total_distmae
        global robot_location
        
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        # print("yaw", yaw)
        robot_location.append([msg.pose.pose.position.y, msg.pose.pose.position.x, yaw])
        location_gt = (0.762*3)/2
        total_distmae.append(np.abs(msg.pose.pose.position.y - 0.381))
        # total_distmae.append(np.abs(msg.pose.pose.position.x + 48.931))
        # total_anglemae.append(np)
        p1 = Point()
        p1.x = msg.pose.pose.position.x #+ 0.762
        p1.y = msg.pose.pose.position.y #+ 0.762
        p1.z = 0 
        new_centroids.append(p1)
        p2 = Point()
        p2.x = msg.pose.pose.position.x #- 0.762
        p2.y = msg.pose.pose.position.y #- 0.762
        p2.z = 0 
        new_centroids.append(p2)
        global markers
        markers.append(new_centroids)
        for centroids in markers:
            for point in centroids:
                cluster_marker.points.append(point)
        # if msg.pose.pose.position.y > 222:
        # if msg.pose.pose.position.x > 20:
        # # if msg.pose.pose.position.y < 4651645:
        # if msg.pose.pose.position.y < 6.3 and msg.pose.pose.position.x < 3.5:
        # if msg.pose.pose.position.y < 6.3 and msg.pose.pose.position.x < 3.0:
        # if msg.pose.pose.position.x > 100:
        # print("odometry MAE is:", np.min(total_distmae),np.mean(total_distmae))
        # print("odometry std is:", np.std(total_distmae))
        #     save_list_to_csv('MPC_maturesoybean_0cm.csv', robot_location, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/icra_results")
            # save_list_to_csv('MPC_maturesoybean_30cm.csv', robot_orientation, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/icra_results")
            # save_list_to_csv('soybean4_odom.csv', robot_location, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/drone_map_txt")
            # save_list_to_csv('PP_deviate_-10cm.csv', np.random.random(10), directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results")
            # save_list_to_csv('0.2_covariance_corn_old.csv', robot_location, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results")
            # save_list_to_csv('PP_deviate_0cm_corn.csv', robot_location, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results")
            # save_list_to_csv('yc_visual_odom.csv', robot_location, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/icra_results")
            # save_list_to_csv('curve_corn_odom.csv', robot_location, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/icra_results")
            # save_list_to_csv('ms_switch_odom.csv', robot_location, directory= "/home/ruijiliu/vision_ws/src/Lidar_RowDetect/mae_results/icra_results")
        marker_pub.publish(cluster_marker)

if __name__ == '__main__':
    rospy.init_node('global_to_local_odometry_node')

    global_offset = None

    # global_odom_sub = rospy.Subscriber('/odom', Odometry, odom_position)
    global_odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_position)
    marker_pub = rospy.Publisher("/robot_position", Marker, queue_size=1)
    rospy.spin()