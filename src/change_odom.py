#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
i = 1
initial_position = None
markers = []
robot_position = None
total_mae = []
total_std = []
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
        global total_mae
        gt = (0.765*3)/2
        total_mae.append(np.abs(msg.pose.pose.position.y - gt))
        p1 = Point()
        p1.x = msg.pose.pose.position.x
        p1.y = msg.pose.pose.position.y + 0.762
        p1.z = 0 
        new_centroids.append(p1)
        p2 = Point()
        p2.x = msg.pose.pose.position.x
        p2.y = msg.pose.pose.position.y - 0.762
        p2.z = 0 
        new_centroids.append(p2)
        global markers
        markers.append(new_centroids)
        for centroids in markers:
            for point in centroids:
                cluster_marker.points.append(point)
        if msg.pose.pose.position.x > 20:
            print("odometry MAE is:", np.mean(total_mae))
            print("odometry std is:", np.std(total_mae))
        marker_pub.publish(cluster_marker)

if __name__ == '__main__':
    rospy.init_node('global_to_local_odometry_node')

    global_offset = None

    # global_odom_sub = rospy.Subscriber('/odom', Odometry, odom_position)
    global_odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_position)
    marker_pub = rospy.Publisher("/robot_position", Marker, queue_size=1)
    rospy.spin()