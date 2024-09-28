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
# from sklearn.cluster import KMeans
from sklearn.linear_model import RANSACRegressor
import cv2
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from cuml.cluster import KMeans
from dummy_vision.msg import line_2pts, line_list
from std_msgs.msg import Header
import time
from geometry_msgs.msg import Twist
from sklearn.metrics import r2_score
import tf
import math
from change_odom import save_list_to_csv
angle_error = []
fitting_centroids = None

global_robot_position = None
i = 1
initial_position = None
initial_orientation = None
robot_orientation = None
mode = None
line_fitting= None
swiped_lines = None
total_mae = []
total_std = []
total_rmse = []
max_value = 0
def odom_position(msg): #get robot update position and orientation
    global i 
    global initial_position
    global initial_orientation
    if i == 1:
        initial_position = msg.pose.pose.position
        initial_orientation = msg.pose.pose.orientation
        i += 1
    else:
        global global_robot_position
    # Extract the robot's position from the odometry message
        global_robot_position = msg.pose.pose.position
        # global_robot_position.x += 1
        global robot_orientation
        robot_orientation = msg.pose.pose.orientation
def mode(msg): #mode for lane-switching
    global mode
    mode = msg.data
def line_function(msg):
    global line_fitting
    line_fitting = msg.data
def swiped_lines_function(msg):
    global swiped_lines
    swiped_lines = msg.data
def list_callback(msg): #receiving detected centroids list
    temp = []
    data = msg.data
    
    for i in range(0, len(data), 2):
        row = list(data[i:i + 2])  # Convert tuple to list
        row.append(0.0)  # Add a 0 to each row
        temp.append(tuple(row))
    global fitting_centroids
    fitting_centroids = temp
def publish_lines_callback():
    line_marker = Marker()
    line_marker.header.frame_id = 'velodyne'
    line_marker.type = Marker.LINE_LIST
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.05  # Line width
    line_marker.color.r = 0.0  # Red
    line_marker.color.g = 1.0
    line_marker.color.b = 0.0
    line_marker.color.a = 1.0  # Fully opaque 
    line_marker.pose.orientation.x = 0.0
    line_marker.pose.orientation.y = 0.0
    line_marker.pose.orientation.z = 0.0
    line_marker.pose.orientation.w = 1.0

    global global_robot_position
    if global_robot_position is None:
        return
    global fitting_centroids
    # print(fitting_centroids)
    if fitting_centroids is None:
        return
    global initial_orientation
    if initial_orientation is None:
        return
    global robot_orientation
    if robot_orientation is None:
        return
    global mode
    if mode is None:
        return
    global line_fitting
    if line_fitting is None:
        return
    global swiped_lines
    if swiped_lines is None:
        return
    global angle_error
    global max_value
    if mode == 1 or mode == 2:
        pass
    if mode == 0 and line_fitting == 0:
        robot_position = global_robot_position

        fitting_centroids = np.array(fitting_centroids)
        print("robot.position1",robot_position.x)
        now_rotation = quaternion_to_rotation_matrix([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
        initial_euler = tf.transformations.euler_from_quaternion([initial_orientation.x, initial_orientation.y, initial_orientation.z, initial_orientation.w ])
        now_euler = tf.transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
        # print("now euler", now_euler)
        line_marker = Marker()
        line_marker.header.frame_id = 'velodyne'
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.r = 0.0  # Red
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0  # Fully opaque 
        line_marker.pose.orientation.x = 0.0
        line_marker.pose.orientation.y = 0.0
        line_marker.pose.orientation.z = 0.0
        line_marker.pose.orientation.w = 1.0 
        T = np.array([robot_position.x, robot_position.y, robot_position.z])
        if swiped_lines % 2 == 1:
            left_x1 = global_to_local(fitting_centroids[-2:][1], T, now_rotation)[0] 
            left_y1 = fitting_centroids[-2:][1][1] - robot_position.y 
            left_x2 = 0
            left_y2 = fitting_centroids[-2:][1][1] - robot_position.y 
            right_x1 = global_to_local(fitting_centroids[-2:][0], T, now_rotation)[0]
            right_x2 = 0
            right_y1 = fitting_centroids[-2:][0][1] - robot_position.y 
            right_y2 = fitting_centroids[-2:][0][1] - robot_position.y 
            line_end = 1.1 #determine when do you want to start fitting a line (e.g. between furthest detected and line end parameter)
            min_left = line_end
            min_right = line_end
            min_left_index = None
            min_right_index = None
            for i in range(0, len(fitting_centroids) - 1 , 2):
                    local_left_x = global_to_local(fitting_centroids[i + 1], T, now_rotation)[0] 
                    local_right_x = global_to_local(fitting_centroids[i], T, now_rotation)[0] 
                    if 0.0 < local_left_x < min_left:
                        min_left_index = i + 1
                        min_left = local_left_x
                        left_x2 = local_left_x
                    if 0.0 < local_right_x < min_right: #
                        min_right_index = i
                        min_right = local_right_x
                        right_x2 = local_right_x
            if min_left_index is None and min_right_index is None:
                print("moving straight")
                vel_msg = Twist()
                vel_msg.linear.x = 0.2
                initialvel_publish.publish(vel_msg)
            if min_left_index is not None:
                print("left")
                left_line_centroids = [fitting_centroids[index]for index in range(min_left_index, len(fitting_centroids), 2)]
                left_line_centroids = sorted(left_line_centroids, key=lambda x: x[0])
                if len(left_line_centroids) > 5:
                    reg_left, m_left, b_left = Ransac_line_fit(left_line_centroids)
                    publish_fitting_line(left_line_centroids, reg_left, line_marker)
                left_x1 = np.mean([global_to_local(point, T, now_rotation)[0] for point in left_line_centroids[-20:]], axis=0)
                
                # ransac fitting 
                left_y1 = reg_left.predict(np.array(left_x1).reshape(-1,1))[0]- robot_position.y
                left_y2 = reg_left.predict(np.array(left_x2).reshape(-1,1))[0]- robot_position.y
                #left_y1 = np.mean([elem[1] for elem in left_line_centroids[-20:]])- robot_position.y 
                #left_y2 = left_line_centroids[0][1] - robot_position.y
            if min_right_index is not None:
                print("right")   
                right_line_centroids = [fitting_centroids[index]for index in range(min_right_index, len(fitting_centroids), 2)]
                right_line_centroids = sorted(right_line_centroids, key=lambda x: x[0])
                if len(right_line_centroids) > 5:
                    reg_right, m_right, b_right = Ransac_line_fit(right_line_centroids)
                    publish_fitting_line(right_line_centroids, reg_right, line_marker)
            
                right_x1 = np.mean([global_to_local(point, T, now_rotation)[0] for point in right_line_centroids[-20:]], axis=0)
                
                # ransac line fitting
                right_y1 = reg_right.predict(np.array(right_x1).reshape(-1,1))[0] - robot_position.y
                right_y2 = reg_right.predict(np.array(right_x2).reshape(-1,1))[0] - robot_position.y
                #right_y1 = np.mean([elem[1] for elem in right_line_centroids[-20:]])- robot_position.y 
                #right_y2 = right_line_centroids[0][1] - robot_position.y

        else: # robot has switched line and going back
            print("second line")
            left_x1 = -fitting_centroids[-2:][1][0] + robot_position.x
            left_y1 = -fitting_centroids[-2:][1][1] + robot_position.y
            left_x2 = 0
            left_y2 = -fitting_centroids[-2:][1][1] + robot_position.y
            right_x1 = -fitting_centroids[-2:][0][0] + robot_position.x
            right_y1 = -fitting_centroids[-2:][0][1] + robot_position.y
            right_x2 = 0
            right_y2 = -fitting_centroids[-2:][0][1] + robot_position.y
            min_left = 0.1
            min_right = 0.1
            min_left_index = None
            min_right_index = None
            for i in range(0, len(fitting_centroids) - 1 , 2):
                local_left_x = -fitting_centroids[i + 1][0] + robot_position.x
                local_right_x = -fitting_centroids[i][0] + robot_position.x
                if 0.0 < local_left_x < min_left:
                    min_left_index = i + 1
                    min_left = local_left_x
                    left_x2 = local_left_x
                if 0.0 < local_right_x < min_right:
                    min_right_index = i
                    min_right = local_right_x
                    right_x2 = local_right_x
            if min_left_index is None and min_right_index is None:
                print("moving straight")
                vel_msg = Twist()
                vel_msg.linear.x = 0.4
                initialvel_publish.publish(vel_msg)
            if min_left_index is not None:
                print("left")
                left_line_centroids = [fitting_centroids[index]for index in range(min_left_index, len(fitting_centroids), 2)]
                left_line_centroids = sorted(left_line_centroids, key=lambda x: x[0])
                reg_left, m_left, b_left = Ransac_line_fit(left_line_centroids)
                left_x1 = -left_line_centroids[0][0] + robot_position.x + 1
                #left_y1 = -left_line_centroids[-1][1] + robot_position.y
                #left_y2 = -left_line_centroids[0][1] + robot_position.y
                left_y1 = -reg_left.predict(np.array(left_x1).reshape(-1,1))[0] + robot_position.y
                left_y2 = -reg_left.predict(np.array(left_x2).reshape(-1,1))[0] + robot_position.y
                publish_fitting_line(left_line_centroids, reg_left, line_marker)

            if min_right_index is not None:
                print("right")   
                right_line_centroids = [fitting_centroids[index]for index in range(min_right_index, len(fitting_centroids), 2)]
                right_line_centroids = sorted(right_line_centroids, key=lambda x: x[0])
                reg_right, m_right, b_right = Ransac_line_fit(right_line_centroids)
                right_x1 = -right_line_centroids[0][0] + robot_position.x + 1
                #right_y1 = -right_line_centroids[-1][1] + robot_position.y 
                #right_y2 = -right_line_centroids[0][1] + robot_position.y 
                right_y1 = -reg_right.predict(np.array(right_x1).reshape(-1,1)) + robot_position.y
                right_y2 = -reg_right.predict(np.array(right_x2).reshape(-1,1)) + robot_position.y
                publish_fitting_line(right_line_centroids, reg_right, line_marker)
        LINE_pub.publish(line_marker)

    
        if min_left_index is not None or min_right_index is not None:
            msg = line_list()
            line_1 = line_2pts(x1=right_x1, y1=right_y1,
                                        x2=right_x2 , y2=right_y2)
            line_2 = line_2pts(x1=left_x1, y1=left_y1,
                                        x2=left_x2, y2=left_y2)
            msg.lines = [line_1, line_2]
            
            msg.num_lines = 2

            msg.header = Header(stamp=rospy.Time.now(), frame_id="Lidar_detect")
            lines_publish.publish(msg)
            
def Ransac_line_fit(centroids):
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
    
def quaternion_to_rotation_matrix(quaternion):
    # Convert a Quaternion to a 3x3 rotation matrix
    x, y, z, w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

    rotation_matrix = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

    return rotation_matrix
def global_to_local(global_point, local_origin, local_orientation):
    translated_point = global_point - local_origin
    
    # Rotate the point
    local_point = np.dot(local_orientation.T, translated_point)
    
    return local_point
def publish_fitting_line(centroids, reg, line_marker):
    
    if len(centroids) > 4:
                
            # Define two points to form the line
        p1 = Point()
        p1.x = np.array(centroids[0][0])
        p1.x = p1.x.reshape(-1,1)
        p1.y = reg.predict(p1.x)[0]
        p1.z = 0.2  # Assuming a 2D line

        p2 = Point()
        p2.x = np.array(centroids[-1][0])  # Adjust the ending x-coordinate as needed
        p2.x = p2.x.reshape(-1,1)
        p2.y = reg.predict(p2.x)[0]
        p2.z = 0.2
    
        line_marker.points.append(p1)
        line_marker.points.append(p2)

if __name__ == "__main__":
    
    rospy.init_node("ransac_line_fitting")

    LINE_pub = rospy.Publisher("/line_marker", Marker, queue_size=1)
    lines_publish = rospy.Publisher("/lines", line_list, queue_size=10)
    initialvel_publish = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/marker_list", Float32MultiArray, list_callback, queue_size= 1)
    rospy.Subscriber("/modes", Int32, mode, queue_size= 1)
    rospy.Subscriber("/line_function", Int32, line_function, queue_size= 1)
    rospy.Subscriber("/swiped_lines", Int32, swiped_lines_function, queue_size= 1)
    rospy.Subscriber("/odometry/filtered", Odometry, odom_position,queue_size= 1)
    # rospy.Subscriber("/odometry/filtered_zeroed", Odometry, odom_position,queue_size= 1)
    # rospy.Subscriber("/odom", Odometry, odom_position,queue_size= 1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        publish_lines_callback()
        rate.sleep()
    rospy.spin()

#     
