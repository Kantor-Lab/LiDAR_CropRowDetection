#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.linear_model import RANSACRegressor
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def fit_two_lines(points):
    ransac = RANSACRegressor()  # Initialize the RANSAC regressor
    ransac.fit(points[:, :2], points[:, 2])  # Fit the data to the RANSAC regressor
    fitted_lines = ransac.estimator_.coef_  # Get the fitted lines
    return fitted_lines

def callback(data, marker_pub):
    pc_data = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(pc_data))

    # Fit two lines using RANSAC
    fitted_lines = fit_two_lines(points)
    print(fitted_lines)
    # Publish markers for the fitted lines
    for idx, line in enumerate(fitted_lines):
        marker = Marker()
        marker.header.frame_id = data.header.frame_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        p1 = Point()
        p1.x = 0.0
        p1.y = line[1]  # Update the index to 1 for the intercept
        p1.z = 0.0
        marker.points.append(p1)

        p2 = Point()
        p2.x = 1.0
        p2.y = line[0] * 1.0 + line[1]  # Update the index to 0 for the slope and 1 for the intercept
        p2.z = 0.0
        marker.points.append(p2)

    marker_pub.publish(marker)

def listener():
    rospy.init_node('point_cloud_listener', anonymous=True)
    marker_pub = rospy.Publisher('/NEW_line_marker', Marker, queue_size=10)
    rospy.Subscriber('/points_above_plane', PointCloud2, callback, marker_pub)
    rospy.spin()

if __name__ == '__main__':
    listener()





