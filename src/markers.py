#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point

def marker_array_publisher():
    rospy.init_node('marker_array_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/crop_gt', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # crop_location = [[0.871486, 1.21147, 0.153011], [1.43791, 1.21147, 0.15863], [1.99879, 1.21147, 0.160933],
    #                  [2.5592, 1.21147, 0.160501], [7.61017, 1.21147, 0.160453], [8.16423, 1.21147, 0.160412],
    #                  [8.71323, 1.21147, 0.160173], [9.27155, 1.21147, 0.160773], [9.81747, 1.21147, 0.159077],
    #                  [10.3517, 1.21147, 0.160894], [0.866156, -0.7,0], [1.46342, -0.7, 0], [2.0344, -0.7, 0],
    #                  [2.63908, -0.7, 0], [7.84792, -0.7, 0], [8.43649, -0.7, 0], [9.02954, -0.7, 0],[9.67741, -0.7, 0],
    #                  [10.2684, -0.7, 0], [3.2224, -0.7, 0], [3.79282, -0.7, 0], [4.34877, -0.7, 0], [4.91833, -0.7, 0],
    #                  [5.5016, -0.7, 0], [6.10254, -0.7, 0], [6.69993, -0.7, 0], [7.24714, -0.7, 0], [3.11572, 1.21147, 0],
    #                  [3.69085, 1.21147, 0], [4.26631, 1.21147, 0], [4.82564, 1.21147, 0], [5.39877, 1.21147, 0],
    #                  [5.96236, 1.21147, 0], [6.51781, 1.21147, 0], [7.06606, 1.21147, 0]]
    num_rows=2
    bushes_per_row=40
    bush_spacing=0.5
    row_spacing=1.0

    crop_location = []
    for row in range(num_rows):
        y = row * row_spacing
        z = 0
        for bush in range(bushes_per_row):
            x = bush * bush_spacing
            temp = []
            temp.append(x)
            temp.append(y)
            temp.append(z)
            crop_location.append(temp)

    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        header = Header(stamp=rospy.Time.now(), frame_id="velodyne")

        # Add markers to the MarkerArray
        for i in range(len(crop_location)):
            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(crop_location[i][0] + 0.510412, crop_location[i][1] + 0.133716, 0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        # Publish the MarkerArray
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        marker_array_publisher()
    except rospy.ROSInterruptException:
        pass