#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sklearn.linear_model import RANSACRegressor
from visualization_msgs.msg import Marker
import numpy as np
import tf
from numpy.linalg import svd
def lidar_callback(msg, marker_pub):
    # Convert the PointCloud2 message to a NumPy array
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_data))

    x = pc_array[:, 0]
    y = pc_array[:, 1]
    z = pc_array[:, 2]
    
    above_plane = 0.06
    # Create a Marker message for the plane
    plane_marker = Marker()
    plane_marker.header = msg.header
    plane_marker.type = Marker.CUBE
    plane_marker.action = Marker.ADD
    plane_marker.pose.position.x = np.mean(x)
    plane_marker.pose.position.y = np.mean(y)
    plane_marker.pose.position.z = np.mean(z) + above_plane

    q1 = tf.transformations.quaternion_from_euler(0, -0.7, 0)
    # q1 = tf.transformations.quaternion_from_euler(0, -0.6, -0.08)

    plane_marker.pose.orientation.x = q1[0]
    plane_marker.pose.orientation.y = q1[1]
    plane_marker.pose.orientation.z = q1[2]
    plane_marker.pose.orientation.w = q1[3]
    plane_marker.scale.x = 5.0  # Adjust the scale as needed
    plane_marker.scale.y = 5.0
    plane_marker.scale.z = 0.001
    plane_marker.color.a = 0.5  # Adjust the transparency
    plane_marker.color.r = 0.0
    plane_marker.color.g = 1.0
    plane_marker.color.b = 0.0

    # Publish the Marker message
    marker_pub.publish(plane_marker)

    # min_angle = np.radians(-130)  # Minimum angle (e.g., -45 degrees)
    min_angle = np.radians(-130)
    max_angle = np.radians(130)   # Maximum angle (e.g., 45 degrees)
    points_above_plane = []
    fields = msg.fields
    num_fields = len(fields)
    point_step = num_fields * 2 # Assuming FLOAT32 fields

    rotation_matrix = quaternion_to_rotation_matrix(plane_marker.pose.orientation)

    # Determine the normal vector of the plane using the rotation matrix
    normal_vector = np.dot(rotation_matrix, np.array([0, 0, 1]))

    # Normalize the normal vector to have a unit length
    normal_vector /= np.linalg.norm(normal_vector)
    # Calculate the "d" coefficient using the dot product between the normal vector and plane's position
    d = -np.dot(normal_vector, np.array([np.mean(x), np.mean(y), np.mean(z)]))
    # Extract a, b, c coefficients from the normal vector
    a, b, c = normal_vector

    # Iterate through the LiDAR points
    for point in pc2.read_points(msg, field_names=("x", "y", "z")):
        point_angle = point[1]

        # Check if the point's angle is within the desired range
        if min_angle <= point_angle <= max_angle and point[0] <= 20: #and point[2] <= 0.4:
        # Extract the x-coordinate of the point as an angle (modify this if your data encodes angles differently)
            if a * point[0] + b * point[1] + c * point[2] + d > above_plane:
                points_above_plane.append(point)
            
                # points_above_plane.append(point)
    # Create a new PointCloud2 message for the filtered points
    print("point_above_plane", len(points_above_plane))
    filtered_msg = PointCloud2()
    filtered_msg.header = msg.header
    filtered_msg.height = 1
    filtered_msg.width = len(points_above_plane)
    filtered_msg.fields = fields  # Use the same fields as the original message
    filtered_msg.is_bigendian = False
    filtered_msg.point_step = point_step
    filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
    filtered_msg.is_dense = True
    filtered_msg.data = np.array(points_above_plane, dtype=np.float32).tobytes()

    # Publish the filtered points to a new topic
    filtered_pub.publish(filtered_msg)

def quaternion_to_rotation_matrix(quaternion):
    # Convert a Quaternion to a 3x3 rotation matrix
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    rotation_matrix = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

    return rotation_matrix
if __name__ == "__main__":
    rospy.init_node("lidar_plane_fitting")
    marker_pub = rospy.Publisher("/plane_marker", Marker, queue_size=1)
    filtered_pub = rospy.Publisher("/points_above_plane", PointCloud2, queue_size=1)
    rospy.Subscriber("/filtered_lidar_points", PointCloud2, lidar_callback, marker_pub)
    # rospy.Subscriber("/velodyne_points", PointCloud2, lidar_callback, marker_pub)
    rospy.spin()
