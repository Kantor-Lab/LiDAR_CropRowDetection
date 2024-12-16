#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
import time
from cuml.cluster import KMeans
from scipy.cluster.hierarchy import linkage, fcluster
from sensor_msgs_py.point_cloud2 import read_points

class LidarKMeansClustering(Node):

    def __init__(self):
        super().__init__('lidar_kmeans_clustering')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.marker_pub1 = self.create_publisher(Marker, '/visualization_marker1', 10)
        self.create_subscription(sensor_msgs.PointCloud2, '/points_above_plane', self.lidar_callback1, 10)
        
    def lidar_callback1(self, msg):
        # Convert the PointCloud2 message to a NumPy array
        filtered_points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z= point
            filtered_points.append([np.float32(x), np.float32(y), np.float32(z)])
        # pc_data = self.read_points(msg)
        # pc_array = np.array(list(pc_data))
        pc_array = np.array(filtered_points, dtype=np.float32)

        num_clusters = 4  # Adjust the number of clusters as needed
        start = time.time()
        kmeans = KMeans(n_clusters=num_clusters, n_init=10, tol=1e-5, max_iter=1000, random_state=0).fit(pc_array)
        end = time.time()
        self.get_logger().info(f"Time taken: {end - start}")

        # Get cluster labels for each point
        cluster_labels = kmeans.predict(pc_array)
        cluster_centroids_points = kmeans.cluster_centers_

        cluster_centroids_points = sorted(cluster_centroids_points, key=lambda x: x[1])
        cluster_centroids_points = self.kmeans_filter(centroids=cluster_centroids_points, num_clusters=num_clusters, threshold=0.3)

        # Create Marker for cluster centroids
        cluster_centroids = Marker()
        cluster_centroids.header = msg.header
        cluster_centroids.type = Marker.POINTS
        cluster_centroids.action = Marker.ADD
        cluster_centroids.scale.x = 0.1  # Adjust the marker size as needed
        cluster_centroids.scale.y = 0.1
        cluster_centroids.color.a = 1.0  # Fully opaque
        cluster_centroids.color.r = 0.0  # Red
        cluster_centroids.color.g = 1.0 
        for point in cluster_centroids_points:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(point[2])
            cluster_centroids.points.append(p)

        # Publish the cluster centroid markers
        self.marker_pub1.publish(cluster_centroids)

    def read_points(self, msg):
        pc_data = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        return pc_data

    def kmeans_filter(self, centroids, num_clusters, threshold):
        merge = True
        while merge:
            new_centroids = []
            delete_centroids = []
            i = 0
            while i < len(centroids) - 1:
                distance = np.abs(centroids[i][1] - centroids[i+1][1])
                if distance > threshold:
                    new_centroids.append(centroids[i])
                    if i == len(centroids) - 2:
                        new_centroids.append(centroids[i + 1])
                    i += 1
                elif distance <= threshold:
                    new_centroids.append(np.mean([np.array(centroids[i]), np.array(centroids[i + 1])], axis=0))
                    delete_centroids.append(1)
                    i += 2
            if delete_centroids:
                centroids = new_centroids
            else:
                merge = False
        return centroids

def main():
    rclpy.init()
    node = LidarKMeansClustering()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
