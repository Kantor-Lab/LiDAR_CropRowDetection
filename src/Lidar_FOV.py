#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

def lidar_callback(msg):
    # Define the angle range (in radians) 
    min_angle = np.radians(-30)  # Minimum angle (e.g., -45 degrees)
    max_angle = np.radians(30)   # Maximum angle (e.g., 45 degrees)

    # Create an empty list to store the filtered points
    filtered_points = []

    # Get the fields from the input message
    fields = msg.fields
    num_fields = len(fields)
    point_step = num_fields * 2 # Assuming FLOAT32 fields

    # Iterate through the LiDAR points
    for point in pc2.read_points(msg, field_names=("x", "y", "z")):
        # Extract the x-coordinate of the point as an angle (modify this if your data encodes angles differently)
        point_angle = point[1]

        # Check if the point's angle is within the desired range #and 0.1 <= point[0] <= 10 
        # if min_angle <= point_angle <= max_angle and 0.1 <= point[0] <= 2 and point[2] <= 0.3:
        if min_angle <= point_angle <= max_angle and 0 <= point[0] <= 2.2: #and -1 <= point[1] <= 1: #2.1 for new large corn mature soybean #2.2 for large corn young soybean#2.3 for small corn
            filtered_points.append(point)

    # Create a new PointCloud2 message for the filtered points
    filtered_msg = PointCloud2()
    filtered_msg.header = msg.header
    filtered_msg.height = 1
    filtered_msg.width = len(filtered_points)
    filtered_msg.fields = fields  # Use the same fields as the original message
    filtered_msg.is_bigendian = False
    filtered_msg.point_step = point_step
    filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
    filtered_msg.is_dense = True
    filtered_msg.data = np.array(filtered_points, dtype=np.float32).tobytes()

    # Publish the filtered points to a new topic
    filtered_pub.publish(filtered_msg)

if __name__ == "__main__":
    rospy.init_node("lidar_data_extraction_and_publishing")
    rospy.Subscriber("/velodyne_points", PointCloud2, lidar_callback)
    filtered_pub = rospy.Publisher("/filtered_lidar_points", PointCloud2, queue_size=1)
    rospy.spin()
