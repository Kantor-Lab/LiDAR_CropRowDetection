#!/usr/bin/env python3
import rospy
from dummy_vision.msg import line_list, line_2pts
from CMU_EKF_Node.msg import line_polar, lines_org
from CMU_Path_Planning_Node.msg import path
from geometry_msgs.msg import Point
def midpoint(x1, y1, x2, y2):
    return (x1 + x2) / 2, (y1 + y2) / 2

def line_callback(msg):
    # Assuming msg is of type LineArray with fields: header, lines, and num_lines
    # print("here")
    if msg.num_lines != 2:
        rospy.logwarn("Expected exactly 2 lines, got {}".format(msg.num_lines))
        return
    
    
    # Extract line endpoints
    line1 = msg.lines[0]
    line2 = msg.lines[1]
    
    # offset1 = (line1.y1 + line2.y1)/2#0.1

    # line1.y1 += offset1
    # line2.y1 += offset1
    # offset2 = (line1.y2 + line2.y2)/2#0.1

    # line1.y2 += offset2
    # line2.y2 += offset2
    # Calculate midpoints
    mx1, my1 = midpoint(line1.x1, line1.y1, line2.x1, line2.y1)
    mx2, my2 = midpoint(line1.x2, line1.y2, line2.x2, line2.y2)
    # mx1, my1 = midpoint(line1.x1 + 3, line1.y1, line2.x1 + 3, line2.y1)
    # mx2, my2 = midpoint(line1.x2 + 3, line1.y2, line2.x2 + 3, line2.y2)
    # Generate PointsArray message
    points_msg = path()
    points_msg.header = msg.header
    
    # Creating a set of points between mx1, my1 and mx2, my2
    num_points = 10  # Number of points you want between mx1, my1 and mx2, my2
    for i in range(num_points + 1):
        ratio = i / float(num_points)
        x = mx2 + ratio * (mx1 - mx2)
        y = my2 + ratio * (my1 - my2)
        point = Point(x=x, y=y, z=0.0)
        points_msg.pts.append(point)
    # points_msg.sort(key=lambda p: p.x)
    # Publish the message
    pub.publish(points_msg)

if __name__ == '__main__':
    rospy.init_node('line_midpoint_publisher')
    
    # Subscriber to the topic with the line messages
    rospy.Subscriber('/lines', line_list, line_callback)
    
    # Publisher for the new points array
    pub = rospy.Publisher('/path_planned', path, queue_size=10)
    
    rospy.spin()