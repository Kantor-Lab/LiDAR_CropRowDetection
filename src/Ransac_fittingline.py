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
fitting_centroids = None
num_clusters = None
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
def odom_position(msg):
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
        global_robot_position.x += 1
        global robot_orientation
        robot_orientation = msg.pose.pose.orientation
        # robot_position.y -= 3
        # robot_position.x -= 1
        # global_robot_position.x = global_robot_position.x - initial_position.x
        # global_robot_position.y = global_robot_position.y - initial_position.y
        # robot_position.z = -robot_position.z + initial_position.z
def mode(msg):
    global mode
    mode = msg.data
def line_function(msg):
    global line_fitting
    line_fitting = msg.data
def swiped_lines_function(msg):
    global swiped_lines
    swiped_lines = msg.data
def list_callback(msg):
    temp = []
    data = msg.data
    for i in range(0, len(data), 2):
        row = data[i:i + 2]
        temp.append(row)
    global fitting_centroids
    fitting_centroids = temp
    # print("centroids:", temp)
    
def centroids_num(msg):
    global num_clusters
    num_clusters = msg.data

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
#     # rospy.init_node('line_marker_publisher', anonymous=True)

#     # Create a ROS Publisher for the Marker messages
#     global fitting_points
#     fitting_centroids = []
#     for row in fitting_points:
#         for i in range(len(row)):
#             temp = []
#             for element in row[i][:2]:
#                 temp.append(element)
#             fitting_centroids.append(temp)
    global initial_orientation
    if initial_orientation is None:
        return
    global robot_orientation
    if robot_orientation is None:
        return
    global num_clusters
    if num_clusters is None:
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
    # num_clusters = 4
    print("modes:", mode)
    if mode == 1 or mode == 2:
        pass
    if mode == 0 and line_fitting == 0:
        robot_position = global_robot_position

        fitting_centroids = np.array(fitting_centroids)
        print("length of fitting centroids", len(fitting_centroids))
        # fitting_centroids = sorted(fitting_centroids, key=lambda x: x[0])
        
        # print(min(fitting_centroids, key=lambda x: x[0])[0])
        print(robot_position.x)

        # local_left_x = []
        # local_right_x = []
        # for i in range(0, len(fitting_centroids) - 1 , 2):
        #     left_x = fitting_centroids[i + 1][0] - robot_position.x
        #     right_x = fitting_centroids[i][0] - robot_position.x
        #     if left_x < 0:
        #         left_x2 = left_x

        #     if right_x < 0:
        #         local_right_x.append(right_x)
        # if local_left_x and local_right_x:
        #     left_x2 = min(local_left_x, key=abs)
        #     print("left", left_x2)
        #     right_x2 = min(local_right_x, key=abs)
        #     print("right", right_x2)
        #     min_left_index = min(range(len(local_left_x)), key=lambda i: abs(local_left_x[i]))
        #     min_right_index = min(range(len(local_right_x)), key=lambda i: abs(local_right_x[i]))
        # print("min_left_index_first", min_left_index)
        # print("min_right_index_first", min_right_index)
        initial_euler = tf.transformations.euler_from_quaternion([initial_orientation.x, initial_orientation.y, initial_orientation.z, initial_orientation.w ])
        now_euler = tf.transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
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
        if swiped_lines % 2 == 1:
            left_x1 = fitting_centroids[-2:][1][0] - robot_position.x
            left_y1 = fitting_centroids[-2:][1][1] - robot_position.y
            left_x2 = 0
            left_y2 = fitting_centroids[-2:][1][1] - robot_position.y
            right_x1 = fitting_centroids[-2:][0][0] - robot_position.x
            right_y1 = fitting_centroids[-2:][0][1] - robot_position.y
            right_x2 = 0
            right_y2 = fitting_centroids[-2:][0][1] - robot_position.y
            min_left = 0.1
            min_right = 0.1
            min_left_index = None
            min_right_index = None
            for i in range(0, len(fitting_centroids) - 1 , 2):
                # if -0.1 < now_euler[2]-initial_euler[2] < 0.1:
                    local_left_x = fitting_centroids[i + 1][0] - robot_position.x
                    local_right_x = fitting_centroids[i][0] - robot_position.x
                    # print("local_left_x", local_left_x)
                    # print("local_right_x", local_right_x)
                    if -1 < local_left_x < min_left:
                        min_left_index = i + 1
                        min_left = local_left_x
                        left_x2 = local_left_x
                    if -1 < local_right_x < min_right:
                        min_right_index = i
                        min_right = local_right_x
                        right_x2 = local_right_x
                # else:
                # print("backward")
                # local_left_x = -fitting_centroids[i + 1][0] + robot_position.x
                # local_right_x = -fitting_centroids[i][0] + robot_position.x
                # # print("local_left_x", local_left_x)
                # # print("local_right_x", local_right_x)
                # if -1 < local_left_x < min_left:
                #     min_left_index = i + 1
                #     min_left = local_left_x
                #     left_x2 = local_left_x
                # if -1 < local_right_x < min_right:
                #     min_right_index = i
                #     min_right = local_right_x
                #     right_x2 = local_right_x
            # print("min_left_index_last", min_left_index)
            # print("min_right_index_last", min_right_index)
            if min_left_index is None and min_right_index is None:
                print("moving straight")
                vel_msg = Twist()
                vel_msg.linear.x = 0.2
                initialvel_publish.publish(vel_msg)
            if min_left_index is not None:
                print("left")
                left_line_centroids = [fitting_centroids[index]for index in range(min_left_index, len(fitting_centroids), 2)]
                # left_line_centroids = [fitting_centroids[index]for index in range(0, min_left_index, 2)]
                left_line_centroids = sorted(left_line_centroids, key=lambda x: x[0])
                reg_left = Ransac_line_fit(left_line_centroids)
                left_x1 = left_line_centroids[-1][0] - robot_position.x 
                left_y1 = left_line_centroids[-1][1] - robot_position.y
                left_y2 = left_line_centroids[0][1] - robot_position.y
                # left_x1 = -left_line_centroids[0][0] + robot_position.x 
                # left_y1 = -left_line_centroids[-1][1] + robot_position.y
                # left_y2 = -left_line_centroids[0][1] + robot_position.y
                # left_y1 = reg_left.predict(np.array(left_x1).reshape(-1,1))- robot_position.y
                # left_y2 = reg_left.predict(np.array(left_x2).reshape(-1,1))- robot_position.y
                print("left x1", left_x1)
                # left_p1, left_p2 = publish_fitting_line(left_line_centroids, reg_left)
                publish_fitting_line(left_line_centroids, reg_left, line_marker)
                # left_p1.x = left_x2 + robot_position.x
                # line_marker.points.append(left_p1)
                # line_marker.points.append(left_p2)

                predicted = [(left_x1 + robot_position.x , left_y1 + robot_position.y), (left_x2 + robot_position.x, left_y2 +robot_position.y)]
                # print("left:", left_y1+robot_position.y)
                ground_truth = [(left_x1+robot_position.x, 1.133716), ( left_x2+ robot_position.x, 1.133716)]
                
                
                # calculate_line_mae(ground_truth, predicted)
                # calculate_line_rmse(ground_truth, predicted)

            if min_right_index is not None:
                print("right")   
                right_line_centroids = [fitting_centroids[index]for index in range(min_right_index, len(fitting_centroids), 2)]
                # right_line_centroids = [fitting_centroids[index]for index in range(0, min_right_index,2)]
                right_line_centroids = sorted(right_line_centroids, key=lambda x: x[0])
                # print(len(right_line_centroids))
                reg_right = Ransac_line_fit(right_line_centroids)
                right_x1 = right_line_centroids[-1][0] - robot_position.x 
                right_y1 = right_line_centroids[-1][1] - robot_position.y 
                right_y2 = right_line_centroids[0][1] - robot_position.y 
                # right_x1 = -right_line_centroids[0][0] + robot_position.x 
                # right_y1 = -right_line_centroids[-1][1] + robot_position.y 
                # right_y2 = -right_line_centroids[0][1] + robot_position.y 
                # right_y1 = reg_right.predict(np.array(right_x1).reshape(-1,1)) - robot_position.y
                # right_y2 = reg_right.predict(np.array(right_x2).reshape(-1,1)) - robot_position.y
                # print("right_y1", right_y1)
                # right_p1, right_p2 = publish_fitting_line(right_line_centroids, reg_right)
                publish_fitting_line(right_line_centroids, reg_right, line_marker)
                # print("right p1", right_p1)
                # print("right p2", right_p2)
                # right_p1.x = right_x2 + robot_position.x
                # line_marker.points.append(right_p1)
                # line_marker.points.append(right_p2)

                predicted = [(right_x1 + robot_position.x , right_y1 + robot_position.y), (right_x2 + robot_position.x, right_y2 +robot_position.y)]
                # print("left:", left_y1+robot_position.y)
                ground_truth = [(right_x1+robot_position.x, 0.633716), ( right_x2+ robot_position.x, 0.633716)]
        else:
            print("second line")
            left_x1 = -fitting_centroids[-2:][1][0] + robot_position.x
            print("left_x1", fitting_centroids[-2:][1][0])
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
            
            # else:
            # print("backward")
                local_left_x = -fitting_centroids[i + 1][0] + robot_position.x
                local_right_x = -fitting_centroids[i][0] + robot_position.x
                # print("local_left_x", local_left_x)
                # print("local_right_x", local_right_x)
                if -1 < local_left_x < min_left:
                    min_left_index = i + 1
                    min_left = local_left_x
                    left_x2 = local_left_x
                if -1 < local_right_x < min_right:
                    min_right_index = i
                    min_right = local_right_x
                    right_x2 = local_right_x
        # print("min_left_index_last", min_left_index)
        # print("min_right_index_last", min_right_index)
            if min_left_index is None and min_right_index is None:
                print("moving straight")
                vel_msg = Twist()
                vel_msg.linear.x = 0.4
                initialvel_publish.publish(vel_msg)
            if min_left_index is not None:
                print("left")
                left_line_centroids = [fitting_centroids[index]for index in range(min_left_index, len(fitting_centroids), 2)]
                # left_line_centroids = [fitting_centroids[index]for index in range(0, min_left_index, 2)]
                left_line_centroids = sorted(left_line_centroids, key=lambda x: x[0])
                reg_left = Ransac_line_fit(left_line_centroids)
                # left_x1 = left_line_centroids[-1][0] - robot_position.x 
                # left_y1 = left_line_centroids[-1][1] - robot_position.y
                # left_y2 = left_line_centroids[0][1] - robot_position.y
                left_x1 = -left_line_centroids[0][0] + robot_position.x + 1
                print("left line centroids:", left_line_centroids[0][0])
                print("left x1", left_x1)
                left_y1 = -left_line_centroids[-1][1] + robot_position.y
                left_y2 = -left_line_centroids[0][1] + robot_position.y
                # left_y1 = reg_left.predict(np.array(left_x1).reshape(-1,1))- robot_position.y
                # left_y2 = reg_left.predict(np.array(left_x2).reshape(-1,1))- robot_position.y
                
                # left_p1, left_p2 = publish_fitting_line(left_line_centroids, reg_left)
                publish_fitting_line(left_line_centroids, reg_left, line_marker)
                # left_p1.x = left_x2 + robot_position.x
                # line_marker.points.append(left_p1)
                # line_marker.points.append(left_p2)

                predicted = [(left_x1 + robot_position.x , left_y1 + robot_position.y), (left_x2 + robot_position.x, left_y2 +robot_position.y)]
                # print("left:", left_y1+robot_position.y)
                ground_truth = [(left_x1+robot_position.x, 1.133716), ( left_x2+ robot_position.x, 1.133716)]
                
                
                # calculate_line_mae(ground_truth, predicted)
                # calculate_line_rmse(ground_truth, predicted)

            if min_right_index is not None:
                print("right")   
                right_line_centroids = [fitting_centroids[index]for index in range(min_right_index, len(fitting_centroids), 2)]
                # right_line_centroids = [fitting_centroids[index]for index in range(0, min_right_index,2)]
                right_line_centroids = sorted(right_line_centroids, key=lambda x: x[0])
                # print(len(right_line_centroids))
                reg_right = Ransac_line_fit(right_line_centroids)
                # right_x1 = right_line_centroids[-1][0] - robot_position.x 
                # right_y1 = right_line_centroids[-1][1] - robot_position.y 
                # right_y2 = right_line_centroids[0][1] - robot_position.y 
                right_x1 = -right_line_centroids[0][0] + robot_position.x + 1
                right_y1 = -right_line_centroids[-1][1] + robot_position.y 
                right_y2 = -right_line_centroids[0][1] + robot_position.y 
                # right_y1 = reg_right.predict(np.array(right_x1).reshape(-1,1)) - robot_position.y
                # right_y2 = reg_right.predict(np.array(right_x2).reshape(-1,1)) - robot_position.y
                # print("right_y1", right_y1)
                # right_p1, right_p2 = publish_fitting_line(right_line_centroids, reg_right)
                publish_fitting_line(right_line_centroids, reg_right, line_marker)

                # print("right p1", right_p1)
                # print("right p2", right_p2)
                # right_p1.x = right_x2 + robot_position.x
                # line_marker.points.append(right_p1)
                # line_marker.points.append(right_p2)

                predicted = [(right_x1 + robot_position.x , right_y1 + robot_position.y), (right_x2 + robot_position.x, right_y2 +robot_position.y)]
                # print("left:", left_y1+robot_position.y)
                ground_truth = [(right_x1+robot_position.x, 0.633716), ( right_x2+ robot_position.x, 0.633716)]   
                # calculate_line_mae(ground_truth, predicted)
                # calculate_line_rmse(ground_truth, predicted)
        LINE_pub.publish(line_marker)

        # msg = line_list()
            
        # line_1 = line_2pts(x1=3.177, y1=0.7,
        #                             x2=0.0188, y2=0.7)
        # line_2 = line_2pts(x1=3.213, y1= -0.3,
        #                             x2=0.0686, y2=-0.3)
        # msg.lines = [line_1, line_2]
        # msg.num_lines = 2

        # msg.header = Header(stamp=rospy.Time.now(), frame_id="Lidar_detect")
        # lines_publish.publish(msg)
    
        if min_left_index is not None or min_right_index is not None:
            msg = line_list()
            line_1 = line_2pts(x1=right_x1, y1=right_y1,
                                        x2=right_x2 , y2=right_y2)
            line_2 = line_2pts(x1=left_x1, y1=left_y1,
                                        x2=left_x2, y2=left_y2)
            # line_1 = line_2pts(x1=left_x1, y1=left_y1,
            #                             x2=left_x2 , y2=left_y2)
            # line_2 = line_2pts(x1=right_x1, y1=right_y1,
            #                             x2=right_x2, y2=right_y2)    
            # line_1 = line_2pts(x1=3.177, y1=-0.7,
            #                             x2=0.0188, y2=-0.7)
            # line_2 = line_2pts(x1=3.213, y1= -0.2,
            #
            #                              x2=0.0686, y2=-0.2)
            
            msg.lines = [line_1, line_2]
            
            msg.num_lines = 2

            msg.header = Header(stamp=rospy.Time.now(), frame_id="Lidar_detect")
            lines_publish.publish(msg)
            
            global total_mae
            global total_std
            global max_value
        # print("robot_position:", robot_position.x)
        # if 0 < robot_position.x < 18:
        #     print("overall mae between lines:", np.mean(total_mae))
        #     print("overall std between lines:", np.mean(total_std))
        #     print("max deviation from ground truth:", max_value)
        #     print("overall rmse between lines:", np.mean(total_rmse))

def calculate_line_mae(ground_truth, predicted):
    global total_mae
    global total_std
    global max_value
    error = []
    num_points = 20
    x_gt, y_gt = zip(*ground_truth)
    x_pred, y_pred = zip(*predicted)

    x_interp_gt = np.linspace(min(x_gt), max(x_gt), num_points)
    y_interp_gt = np.interp(x_interp_gt, x_gt, y_gt)
        
    x_interp_pred = np.linspace(min(x_pred), max(x_pred), num_points)
    y_interp_pred = np.interp(x_interp_pred, x_pred, y_pred)

    total_distance = 0
    for i in range(len(y_interp_pred)):
        ab_dis = np.abs(y_interp_pred[i]-y_interp_gt[i])
        # total_distance += ab_dis
        error.append(ab_dis)
        if ab_dis > max_value:
            max_value = ab_dis
    
    mae = np.mean(error)
    std = np.std(error)

    total_mae.append(mae)
    total_std.append(std) 
def calculate_line_rmse(ground_truth, predicted):
    global total_rmse
    error = []
    num_points = 20
    x_gt, y_gt = zip(*ground_truth)
    x_pred, y_pred = zip(*predicted)

    x_interp_gt = np.linspace(min(x_gt), max(x_gt), num_points)
    y_interp_gt = np.interp(x_interp_gt, x_gt, y_gt)
        
    x_interp_pred = np.linspace(min(x_pred), max(x_pred), num_points)
    y_interp_pred = np.interp(x_interp_pred, x_pred, y_pred)

    total_distance = 0
    for i in range(len(y_interp_pred)):
        ab_dis = np.abs(y_interp_pred[i]-y_interp_gt[i])
        # total_distance += ab_dis
        error.append(ab_dis ** 2)
    mse = np.mean(error)
    total_rmse.append(np.sqrt(mse))
def Ransac_line_fit(centroids):
    X = []
    Y = []
    for row in centroids:
        X.append(row[0])
        Y.append(row[1])
    X= np.array(X).reshape(-1,1)
    Y = np.array(Y)

    if len(X) > 4:
        reg = RANSACRegressor(min_samples=5, random_state=0).fit(X,Y)

    return reg
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
        
        # print("p2 - p1", p2.x - p1.x)
                # Add the points to the marker
        line_marker.points.append(p1)
        line_marker.points.append(p2)
        # LINE_pub.publish(line_marker)
    # return p1, p2
if __name__ == "__main__":
    
    rospy.init_node("ransac_line_fitting")
    # # while not rospy.is_shutdown():
    # #     publish_lines(fitting_points=fitting_points)
    LINE_pub = rospy.Publisher("/line_marker", Marker, queue_size=1)
    lines_publish = rospy.Publisher("/lines", line_list, queue_size=10)
    initialvel_publish = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # # rospy.Subscriber("/points_above_plane", PointCloud2, lidar_callback, marker_pub)
    rospy.Subscriber("/marker_list", Float32MultiArray, list_callback, queue_size= 1)
    rospy.Subscriber("/modes", Int32, mode, queue_size= 1)
    rospy.Subscriber("/line_function", Int32, line_function, queue_size= 1)
    rospy.Subscriber("/swiped_lines", Int32, swiped_lines_function, queue_size= 1)
    rospy.Subscriber("/number_of_centroids", Int32, centroids_num, queue_size= 1)
    rospy.Subscriber("/odometry/filtered", Odometry, odom_position,queue_size= 1)
    # rospy.Subscriber("/odom", Odometry, odom_position,queue_size= 1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        publish_lines_callback()
        rate.sleep()
        # publish_3d_points()
    rospy.spin()
    
    
    # def publish_3d_points():
    # global fitting_centroids
    # if fitting_centroids is None:
    #     return
    # centroid = fitting_centroids[-2:]
    # # print(centroid[0][1])
    # global robot_position
    # if robot_position is None:
    #     return
    # min_left = 0.1
    # min_right = 0.1
    # origin_left_x = 0
    # origin_right_x = 0
    # origin_left_y = centroid[0][1]
    # origin_right_y = centroid[1][1]
    # for i in range(0, len(fitting_centroids), 2):
    #     if 0 < fitting_centroids[i][0] - robot_position.x < min_left:
    #         min_left = fitting_centroids[i][0] - robot_position.x
    #         origin_left_x = min_left
    #         origin_left_y = fitting_centroids[i][1]
    #     if 0 < fitting_centroids[i+1][0] - robot_position.x < min_right:
    #         min_right = fitting_centroids[i+1][0] - robot_position.x
    #         origin_right_x = min_right
    #         origin_right_y = fitting_centroids[i][1]
    

    # msg = line_list()
    # line_1 = line_2pts(x1=centroid[0][0]-robot_position.x, y1=centroid[0][1]- robot_position.y,
    #                        x2=origin_left_x, y2=origin_left_y)
        
    # line_2 = line_2pts(x1=centroid[1][0]-robot_position.x, y1=centroid[1][1]-robot_position.y,
    #                        x2=origin_right_x, y2=origin_right_y)
    # msg.lines = [line_1, line_2]
    # msg.num_lines = 2

    # msg.header = Header(stamp=rospy.Time.now(), frame_id="Lidar_detect")
    # lines_publish.publish(msg)
    # except rospy.ROSInterruptException:
    #     pass
# def publish_fitting_line(centroids, reg):
#     line_marker = Marker()
#     line_marker.header.frame_id = 'velodyne'
#     line_marker.type = Marker.LINE_LIST
#     line_marker.action = Marker.ADD
#     line_marker.scale.x = 0.05  # Line width
#     line_marker.color.r = 0.0  # Red
#     line_marker.color.g = 1.0
#     line_marker.color.b = 0.0
#     line_marker.color.a = 1.0  # Fully opaque 
#     line_marker.pose.orientation.x = 0.0
#     line_marker.pose.orientation.y = 0.0
#     line_marker.pose.orientation.z = 0.0
#     line_marker.pose.orientation.w = 1.0
#     # print("centroids location:", fitting_centroids[-2:][0][0])
#     # print("robot x", robot_position.x)
    
#     # for centroid in line_centroids:
#     #     centroid = centroid[0]
#     #     X = []
#     #     Y = []
#     #     for row in fitting_centroids:
#     #         if centroid - 0.3 < row[1] < centroid + 0.3:
#     #             X.append(row[0])
#     #             Y.append(row[1])
#     #     X= np.array(X).reshape(-1,1)
#     #     Y = np.array(Y)  
#     if len(X) > 4:
                
#             # Define two points to form the line
#         p1 = Point()
            
#             # if len(X) <= number_of_centroids:
#             #     p1.x = X[1]  # Adjust the starting x-coordinate as needed
#             # else:
#             #     p1.x = X[-number_of_centroids]
#         p1.x = centroids[0][0]
#         p1.x = p1.x.reshape(-1,1)
#         p1.y = reg.predict(p1.x)[0]
#         p1.z = 0.01  # Assuming a 2D line

#         p2 = Point()
#         p2.x = centroids[-1][0]  # Adjust the ending x-coordinate as needed
#         p2.x = p2.x.reshape(-1,1)
#         p2.y = reg.predict(p2.x)[0]
#         p2.z = 0.01
        
#                 # Add the points to the marker
#         line_marker.points.append(p1)
#         line_marker.points.append(p2)
#     LINE_pub.publish(line_marker)
#             min_left = 0.1
#             min_right = 0.1
#             min_value = min(fitting_centroids, key=lambda x: x[0])[0]
#             for i in range(0, len(fitting_centroids), 2):
#                 if 0 < fitting_centroids[i][0] - robot_position.x -1 < min_left:
#                     min_left = fitting_centroids[i][0] - robot_position.x -1
#                     left_x2 = min_left
#                     left_y2 = reg.predict(np.array(fitting_centroids[i][0]).reshape(-1,1))
#                 if 0 < fitting_centroids[i+1][0] - robot_position.x - 1< min_right:
#                     min_right = fitting_centroids[i+1][0] - robot_position.x -1
#                     right_x2 = min_right
#                     right_y2 = reg.predict(np.array(fitting_centroids[i+1][0]).reshape(-1,1))
#         # Publish the marker
#     # rate = rospy.Rate(10)
#     # while not rospy.is_shutdown():

#     