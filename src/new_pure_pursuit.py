#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from dummy_vision.msg import line_2pts, line_list
from CMU_Path_Planning_Node.msg import path
import numpy as np
from geometry_msgs.msg import Point, Twist

def Pure_pursuit(goal_point):
    local_x = goal_point.x
    local_y = goal_point.y
    curvature = 2* local_y/(local_x ** 2 + local_y ** 2)
    wheelbase_length = 1.2192
    steering_angle = np.arctan(curvature * wheelbase_length)
    steering_angle = curvature
    angle_to_lookahead = np.arctan2(local_y, local_x)
    
    twist = Twist()
    # Set the desired linear and angular velocities
    twist.linear.x = 0.3  # Move forward at 1 m/s
    twist.angular.z = -steering_angle * 0.5  # Rotate at 0.5 rad/s
    # twist.angular.z = -angle_to_lookahead
    # Publish the Twist message
    steering_pub.publish(twist)

def callback(msg):
    # Extract points from the message
    points = msg.pts

    # Convert to list of Point objects for easier handling
    waypoints = [Point(x=point.x, y=point.y) for point in points]
    index = 10 #1-10
    goal_point = waypoints[index-1]
    print("goal point", goal_point)
    Pure_pursuit(goal_point)
def listener():
    rospy.init_node('pure_pursuit_node', anonymous=True)
    rospy.Subscriber('/path_planned', path, callback)  # Replace 'your_topic' and 'YourMessageType'
    global steering_pub
    steering_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Replace with actual topic and message type
    rospy.spin()

if __name__ == '__main__':
    listener()