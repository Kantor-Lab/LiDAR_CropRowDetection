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
def odometry_callback(msg):
    global i
    global initial_orientation
    global initial_position
    if i == 1:
        initial_orientation = msg.pose.pose.orientation
        initial_position = msg.pose.pose.position
        i += 1
    else:
        global robot_position
    # Extract the robot's position from the odometry message
        robot_position = msg.pose.pose.position
        robot_position.x += 1
        # robot_position.y -= 3
        # robot_position.x = -robot_position.x + initial_position.x
        # robot_position.y = -robot_position.y + initial_position.y
        # robot_position.z = robot_position.z + initial_position.z
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
    # Preprocess the point cloud data if needed
    global robot_position
    if robot_position is None:
        return
    global robot_orientation
    if robot_orientation is None:
        return
    global initial_orientation
    if initial_orientation is None:
        return
    
    # global ground_truth
    # truth_odom = [robot_position.x, robot_position.y, robot_position.z]
    # ground_truth.append(truth_odom)
    # point_cloud = o3d.geometry.PointCloud()
    # point_cloud.points = o3d.utility.Vector3dVector(np.array(ground_truth))

# Save the PointCloud to a PCD file
    # o3d.io.write_point_cloud("odom.pcd", point_cloud)
    # Convert the PointCloud2 message to a NumPy array
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_data))
    global time_to_stop
    # if len((pc_array)) < 80:
    #     if time_to_stop <= 40:
    #         time_to_stop += 1
    #         pass
    #     else:
    #         print("End of Lane Detected")
    #         vel_msg = Twist()
    #         vel_msg.linear.x = 0.0
    #         endvel_publish.publish(vel_msg)
    #         rospy.signal_shutdown('End of lane Detected')

    num_clusters =5  # Adjust the number of clusters as needed
    
    start = time.time()
    kmeans = KMeans(n_clusters=num_clusters, n_init= 10, tol = 1e-4, max_iter = 1000, random_state=0).fit(pc_array)
    end = time.time()
    print("take time:", end - start)
    # Get cluster labels for each point
    cluster_labels = kmeans.predict(pc_array)
    centroids = kmeans.cluster_centers_

    # z_mean = np.mean(pc_array[:,2], axis = 0)
    # for row in centroids:
    #     row[-1] = z_mean
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
    
    initial_euler = tf.transformations.euler_from_quaternion([initial_orientation.x, initial_orientation.y, initial_orientation.z, initial_orientation.w ])
    now_euler = tf.transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
    true_orientation = tf.transformations.quaternion_from_euler(now_euler[0]-initial_euler[0], now_euler[1]-initial_euler[1], now_euler[2]-initial_euler[2])
    rotation_matrix = quaternion_to_rotation_matrix(true_orientation)

    # global initial_position
    # if initial_position is None:
    #     return
    # robot_position.x = robot_position.x - initial_position.x
    # robot_position.y = robot_position.y - initial_position.y
    # robot_position.z = robot_position.z - initial_position.z
    # print(robot_position.y)
    T = np.array([robot_position.x, robot_position.y, robot_position.z])
    for point in centroids:
        # old_pointx = point[0]
        # point[0] = -point[1] # Y axis in lidar and x in global is opposite
        # point[1] = old_pointx * np.cos(np.radians(30))
        point[0] = point[0] * np.cos(0.5)
        # point[1] = old_pointx
        point[2] = 0 
        P = np.array([point[0], point[1], point[2]])
        new_point = rotation_matrix @ P + T
        # print(new_point)
        # new_point = point
        points.append(new_point)
        p = Point()
        p.x = new_point[0] #+robot_position.y
        p.y = new_point[1] #+robot_position.x
        p.z = new_point[2] 
        new_centroids.append(p)
    # print("true orientation:", true_orientation)
    global markers
    markers.append(new_centroids)
    global fitting_points
    fitting_points.append(points)
    for centroids in markers:
        for point in centroids:
            cluster_marker.points.append(point)
    # marker_pub.publish(cluster_marker)
    fitting_centroids = []
    for row in fitting_points:
        # neg_num = min((n[1] for n in row if n[1] < 0), key=lambda x: abs(x))
        # # print("neg", neg_num)
        # pos_num = next(n[1] for n in row if n[1] > 0)
        # print("pos", pos_num)
        if len(row) % 2 == 0:
            mid = len(row) // 2
            neg_num = row[mid-1][1]
            pos_num = row[mid][1]
        else: pass
        # print("neg & pos numbers:", neg_num, pos_num)
        for i in range(len(row)):
            if row[i][1] == neg_num or row[i][1] == pos_num:
                temp = []
                for element in row[i][:2]:
                    temp.append(element)
                fitting_centroids.append(temp)

    # number_of_centroids = 100
#     point_cloud = o3d.geometry.PointCloud()
#     point_cloud.points = o3d.utility.Vector3dVector(np.array(fitting_centroids))

# # Save the PointCloud to a PCD file
#     o3d.io.write_point_cloud("output.pcd", point_cloud)
    # average_distance = calculate_distance(ground_truth, fitting_centroids)
    # print("average distance error is:", average_distance)

    # if len(fitting_centroids) > number_of_centroids * num_clusters:
    #     fitting_centroids = fitting_centroids[-number_of_centroids*num_clusters:]
    flat_data = [item for sublist in fitting_centroids for item in sublist]
    # print("length of data",len(flat_data))
    list_message = Float32MultiArray(data=flat_data)
    marker_pub.publish(cluster_marker)
    LIST_pub.publish(list_message)
    value_pub.publish(num_clusters)

    # if robot_position.x > 9.5:
    #     crop_location = []
    #     num_rows=4
    #     bushes_per_row=100
    #     bush_spacing=0.1
    #     row_spacing=0.5
    #     for row in range(num_rows):
    #         y = row * row_spacing + 0.133716
    #         z = 0
    #         for bush in range(bushes_per_row):
    #             x = bush * bush_spacing + 0.510412
    #             temp = []
    #             temp.append(x)
    #             temp.append(y)
    #             temp.append(z)
    #             crop_location.append(temp)
        
    #     std, mae = calculate_mae(crop_location, fitting_points)
    #     print("standard deviation:", std)
    #     print("MAE is:", mae ) 
        # print("RMSE is:", rmse )

def kmeans_filter(centroids, threshold):
    merge = True
    while merge:
        new_centroids = []
        delete_centroids = []
        print("centroids", centroids)
        for i in range(len(centroids)-1):
        # i = 0
        # while i < len(centroids)-1:
            distance = np.abs(centroids[i][1] - centroids[i+1][1])
            # distance = np.linalg.norm(np.array(centroids[i])-np.array(centroids[i+1]))
            print(distance)
            if distance > threshold:
                new_centroids.append(centroids[i])
                if np.abs(centroids[i][1] - centroids[i-1][1]) < threshold:
                    new_centroids.pop()
                    if i >= 1 and i == len(centroids) -2:
                        new_centroids.append(centroids[i+1])
                elif i >= 1 and i == len(centroids) -2 and np.abs(centroids[i][1] - centroids[i-1][1]) >= threshold: 
                    new_centroids.append(centroids[i+1])
                # i+=1
            elif distance <= threshold:
                new_centroids.append(np.mean([np.array(centroids[i]),np.array(centroids[i+1])], axis = 0))
                delete_centroids.append(1)
                # i+=2
        if delete_centroids:
            print("continue")
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
    # # while not rospy.is_shutdown():
    # #     publish_lines(fitting_points=fitting_points)
    marker_pub = rospy.Publisher("/visualization_marker_centroids", Marker, queue_size=1)
    LIST_pub = rospy.Publisher('/marker_list', Float32MultiArray, queue_size=1)
    value_pub = rospy.Publisher('/number_of_centroids', Int32, queue_size=1)
    endvel_publish = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # # LINE_pub = rospy.Publisher("/line_marker", Marker, queue_size=1)
    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback,queue_size= 1)
    # rospy.Subscriber("/odom", Odometry, odometry_callback1,queue_size= 1)
    rospy.Subscriber("/points_above_plane", PointCloud2, lidar_callback, marker_pub, queue_size= 1)
    # rospy.Subscriber("/points_above_plane", PointCloud2, publish_lines_callback, LINE_pub)
    
    
    rospy.spin()

# sklearn kmeans
# start = time.time()
    # kmeans = KMeans(n_clusters=num_clusters, n_init= 1, max_iter = 500, random_state=0).fit(pc_array)
    # end = time.time()
    # print("take time:", end - start)
    # # Get cluster labels for each point
    # cluster_labels = kmeans.labels_

    # centroids = kmeans.cluster_centers_
    # X = pc_array.astype(np.float32)

# hierachical method 
# z_mean = np.mean(pc_array[:,2], axis = 0)
    # pc_array = pc_array[:,:2]
    # linkage_matrix = linkage(pc_array, method="ward")  # You can choose a different method
    # threshold = 10  # Adjust the threshold as needed
    # cluster_labels = fcluster(linkage_matrix, threshold, criterion="distance")

    # # Extract cluster centroids
    # unique_labels = np.unique(cluster_labels)
    # centroids = []
    # for label in unique_labels:
    #     cluster_points = pc_array[cluster_labels == label]
    #     centroid = np.mean(cluster_points, axis=0)
    #     centroid = np.insert(centroid, 2, z_mean, axis=0)
    #     centroids.append(centroid)

# def publish_lines_callback(msg,LINE_pub):
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
    
#     number_of_centroids = 30
#     num_clusters = 4
#     fitting_centroids = np.array(fitting_centroids)
#     if fitting_centroids.shape[0] > number_of_centroids * num_clusters:
#         fitting_centroids = fitting_centroids[:,-number_of_centroids*num_clusters:]
    
    

#       # Adjust the number of clusters as needed
#     lines_kmeans = KMeans(n_clusters=num_clusters, n_init= 1, max_iter = 10, random_state=0).fit(fitting_centroids[:,1].reshape(-1,1)) 
#     line_centroids = lines_kmeans.cluster_centers_ 


#     # y_data = fitting_centroids[:,1].reshape(-1,1)
#     # linkage_matrix = linkage(y_data, method="ward")  # You can choose a different method
#     # threshold = 7  # Adjust the threshold as needed
#     # cluster_labels = fcluster(linkage_matrix, threshold, criterion="distance")

#     # # Extract cluster centroids
#     # unique_labels = np.unique(cluster_labels)
#     # line_centroids = []
#     # for label in unique_labels:
#     #     cluster_points = y_data[cluster_labels == label]
#     #     centroid = np.mean(cluster_points, axis=0)
#     #     line_centroids.append(centroid)
#     # print(len(line_centroids))
    
#     line_marker = Marker()
#     line_marker.header = msg.header
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
#     for centroid in line_centroids:
#         centroid = centroid[0]
#         X = []
#         Y = []
#         for row in fitting_centroids:
#             if centroid - 0.3 < row[1] < centroid + 0.3:
#                 X.append(row[0])
#                 Y.append(row[1])
#         X= np.array(X).reshape(-1,1)
#         Y = np.array(Y)
        
#         if len(X) > 4:
#             reg = RANSACRegressor(min_samples=5, random_state=0).fit(X,Y)
                
#             # Define two points to form the line
#             p1 = Point()
            
#             if len(X) <= number_of_centroids:
#                 p1.x = X[1]  # Adjust the starting x-coordinate as needed
#             else:
#                 p1.x = X[-number_of_centroids]
#             p1.x = p1.x.reshape(-1,1)
#             p1.y = reg.predict(p1.x)[0]
#             p1.z = 0.5  # Assuming a 2D line

#             p2 = Point()
#             p2.x = X[-1]  # Adjust the ending x-coordinate as needed
#             p2.x = p2.x.reshape(-1,1)
#             p2.y = reg.predict(p2.x)[0]
#             p2.z = 0.5
        
#                 # Add the points to the marker
#             line_marker.points.append(p1)
#             line_marker.points.append(p2)
#         # Publish the marker
#     LINE_pub.publish(line_marker)
    # fitting_centroids = [[row[i][:2] for i in range(len(row))] for row in fitting_points]
    # fitting_centroids = [[[element.item() for element in row[i][:2]] for i in range(len(row))] for row in fitting_points]
    # print(fitting_centroids)
    
    
    # if lines is not None:
    #     # Create a Marker message to visualize the detected lines
    #     line_marker = Marker()
    #     line_marker.header = msg.header
    #     line_marker.type = Marker.LINE_LIST
    #     line_marker.action = Marker.ADD
    #     line_marker.scale.x = 0.05  # Line width
    #     line_marker.color.r = 0.0  # Red
    #     line_marker.color.g = 1.0
    #     line_marker.color.b = 0.0
    #     line_marker.color.a = 1.0  # Fully opaque

    #     # Iterate over detected lines and add them to the Marker message
    #     for line in lines:
    #         rho, theta = line[0]
    #         a = np.cos(theta)
    #         b = np.sin(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         x1 = int(x0 + 25 * (-b))
    #         y1 = int(y0 + 25 * (a))
    #         x2 = int(x0 - 25 * (-b))
    #         y2 = int(y0 - 25 * (a))

    #         p1 = Point()
    #         p1.x = x1
    #         p1.y = y1
    #         p1.z = 1.0
    #         p2 = Point()
    #         p2.x = x2
    #         p2.y = y2
    #         p2.z = 1.0
    #         line_marker.points.append(p1)
    #         line_marker.points.append(p2)

    #     # Publish the Marker message
    #     LINE_pub.publish(line_marker)
    
    # print(lines)
    # Publish the single marker containing all clusters

    # Publish the markers
    # print(line1)
    # lines = {}
    # lines['line1'] = line1
    # lines['line2'] = line2
    # lines['line3'] = line3
    # lines['line4'] = line4
    

    # marker = Marker()
    # marker.header = msg.header # Replace with your frame ID
    # marker.type = Marker.LINE_STRIP
    # marker.action = Marker.ADD
    # marker.pose.orientation.w = 1.0  # Default orientation

    # X = [row[0] for row in lines["line1"]]
    # X = np.array(X).reshape(-1, 1)


    # Y = [row[1] for row in lines["line1"]]
    # Y = np.array(Y)


    
    # # Set the color, you can adjust these values as needed
    # if len(X) > 4:
    #     reg = RANSACRegressor(min_samples=5, random_state=0).fit(X,Y)
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0  # Fully opaque

    #     # Set the scale of the line
    #     marker.scale.x = 0.1  # Adjust the line width as needed

    #     # Define two points to form the line
    #     p1 = Point()
    #     number_of_centroids = 50
    #     if len(X) <= number_of_centroids:
    #         p1.x = X[1]  # Adjust the starting x-coordinate as needed
    #     else:
    #         p1.x = X[-number_of_centroids]
    #     p1.x = p1.x.reshape(-1,1)
    #     p1.y = reg.predict(p1.x)[0]
    #     p1.z = 1  # Assuming a 2D line

    #     p2 = Point()
    #     p2.x = X[-1]  # Adjust the ending x-coordinate as needed
    #     p2.x = p2.x.reshape(-1,1)
    #     p2.y = reg.predict(p2.x)[0]
    #     p2.z = 1
    
    #         # Add the points to the marker
    #     marker.points.append(p1)
    #     marker.points.append(p2)

    #         # Publish the marker
    #     LINE_pub.publish(marker)