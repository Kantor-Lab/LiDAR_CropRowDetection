#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
# from sklearn.cluster import KMeans
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from scipy.cluster.hierarchy import linkage, fcluster
import time
from cuml.cluster import KMeans
# import torch
# from kmeans_pytorch import kmeans, kmeans_predict
def lidar_callback(msg, marker_pub):
    # Convert the PointCloud2 message to a NumPy array
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_data))

    # Perform K-means clustering
    num_clusters = 4  # Adjust the number of clusters as needed
    kmeans = KMeans(n_clusters=num_clusters, n_init= 1, tol = 1e-2, max_iter= 10, random_state=0).fit(pc_array)

    # Get cluster labels for each point
    cluster_labels = kmeans.labels_

    cluster_centroids_points = kmeans.cluster_centers_
    
    
    # Create markers for each cluster

    cluster_marker = Marker()
    cluster_marker.header = msg.header
    # cluster_marker.header.frame_id = "/camera_init"
    cluster_marker.type = Marker.POINTS
    cluster_marker.action = Marker.ADD
    cluster_marker.scale.x = 0.1  # Adjust the marker size as needed
    cluster_marker.scale.y = 0.1
    cluster_marker.color.a = 1.0  # Fully opaque
    cluster_marker.color.r = 1.0 # Red
    cluster_marker.color.g = 0.0
    cluster_marker.color.b = 0.0

    for label in range(num_clusters):
        cluster_points = pc_array[cluster_labels == label]

        for point in cluster_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            cluster_marker.points.append(p)

    # Publish the single marker containing all clusters
    marker_pub.publish(cluster_marker)
def lidar_callback1(msg, marker_pub1):
    # Convert the PointCloud2 message to a NumPy array
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_data))

    # linkage_matrix = linkage(pc_array, method="ward")  # You can choose a different method
    # threshold = 10  # Adjust the threshold as needed
    # cluster_labels = fcluster(linkage_matrix, threshold, criterion="distance")

    # # Extract cluster centroids
    # unique_labels = np.unique(cluster_labels)
    # cluster_centroids_points = []
    # for label in unique_labels:
    #     cluster_points = pc_array[cluster_labels == label]
    #     centroid = np.mean(cluster_points, axis=0)
    #     cluster_centroids_points.append(centroid)
    # Perform K-means clustering
    num_clusters = 5  # Adjust the number of clusters as needed
    # kmeans = KMeans(n_clusters=num_clusters, n_init= 'auto', max_iter = 500, random_state=0).fit(pc_array)

    # # Get cluster labels for each point
    # cluster_labels = kmeans.labels_

    # cluster_centroids_points = kmeans.cluster_centers_
    start = time.time()
    kmeans = KMeans(n_clusters=num_clusters, n_init= 10, tol = 1e-5, max_iter = 1000, random_state=0).fit(pc_array)
    end = time.time()
    print("take time:", end - start)
    # Get cluster labels for each point
    cluster_labels = kmeans.predict(pc_array)
    cluster_centroids_points = kmeans.cluster_centers_
    # X = torch.tensor(pc_array, dtype=torch.float32)
    # cluster_labels, cluster_centroids_points = kmeans(X=X, num_clusters=num_clusters, distance='euclidean', device=torch.device('cuda:0'))

    cluster_centroids_points = sorted(cluster_centroids_points, key=lambda x: x[1])
    cluster_centroids_points = kmeans_filter(centroids=cluster_centroids_points, num_clusters=num_clusters, threshold= 0.5)
    # print(len(cluster_centroids_points))
    # print(cluster_centroids_points)
    cluster_centroids = Marker()
    cluster_centroids.header = msg.header
    # cluster_centroids.header.frame_id = "/camera_init"
    cluster_centroids.type = Marker.POINTS
    cluster_centroids.action = Marker.ADD
    cluster_centroids.scale.x = 0.1  # Adjust the marker size as needed
    cluster_centroids.scale.y = 0.1
    cluster_centroids.color.a = 1.0  # Fully opaque
    cluster_centroids.color.r = 1.0 # Red
    cluster_centroids.color.g = 0.0
    cluster_centroids.color.b = 0.0

    
    
    for point in cluster_centroids_points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2]
        cluster_centroids.points.append(p)

    # Publish the single marker containing all clusters
    marker_pub1.publish(cluster_centroids)
def kmeans_filter(centroids, num_clusters, threshold):
    merge = True
    while merge:
        new_centroids = []
        delete_centroids = []
        print("centroids", centroids)
        # for i in range(len(centroids)-1):
        i = 0
        while i < len(centroids) - 1:
            distance = np.abs(centroids[i][1] - centroids[i+1][1])
            # distance = np.linalg.norm(np.array(centroids[i])-np.array(centroids[i+1]))
            print(distance)
            if distance > threshold:
                new_centroids.append(centroids[i])
                if i == len(centroids) -2:
                    new_centroids.append(centroids[i+1])
                i+=1
            # elif np.all(centroids[i] == [0, 0, 0]):
            #     continue
            elif distance <= threshold:
                # print("first centroids", centroids[i])
                # print("sencond centroids", centroids[i+1])
                # print("mean",np.mean([np.array(centroids[i]),np.array(centroids[i+1])], axis = 0))
                new_centroids.append(np.mean([np.array(centroids[i]),np.array(centroids[i+1])], axis = 0))

                delete_centroids.append(1)
                i+=2
                # centroids[i] = np.mean([np.array(centroids[i]),np.array(centroids[i+1])], axis = 0)
                # centroids[i+1] = np.array([0, 0, 0])
        # print(new_centroids)
        if delete_centroids:
            print("continue")
            centroids = new_centroids
            # print(centroids)
        else:
            merge = False
            print("quit")
        # for item in centroids:
        #     if (item == np.array([0, 0, 0])).all():
        #         centroids.remove
        # centroids = new_centroids
        
        # print(centroids)
        # centroids = [item for item in centroids if not np.all(item == [0, 0, 0])]
        
        # centroids = [arr.tolist() for arr in centroids]
        # print(centroids[0])
        # Calculate pairwise Euclidean distances between centroids
    
    return centroids
def contains_zeros(lst):
    for item in lst:
        if (item == np.array([0, 0, 0])).all():
            return True
    return False
if __name__ == "__main__":
    rospy.init_node("lidar_kmeans_clustering")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    marker_pub1 = rospy.Publisher("/visualization_marker1", Marker, queue_size=10)

    # rospy.Subscriber("/points_above_plane", PointCloud2, lidar_callback, marker_pub)
    rospy.Subscriber("/points_above_plane", PointCloud2, lidar_callback1, marker_pub1)
    rospy.spin()
