#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Header
from dummy_vision.msg import line_list, line_2pts
from CMU_EKF_Node.msg import line_polar, lines_org

LEFT = 0
CENTER = 1
RIGHT = 2


def points_to_polar(msg):
    lines = []
    for i in range(msg.num_lines):
        x1 = msg.lines[i].x1
        y1 = msg.lines[i].y1
        x2 = msg.lines[i].x2
        y2 = msg.lines[i].y2

        dx = x2 - x1
        dy = y2 - y1
        det = (dx * dx) + (dy * dy)
        num = ((dy * -y1) + (dx * -x1)) / det

        closestX = x1 + (num * dx)
        closestY = y1 + (num * dy)

        a = dy
        b = -dx
        c = -((a * x1) + (b * y1))

        polar = line_polar()
        polar.distance = abs(c / b)
        polar.theta = -math.atan2(closestY, closestX)

        if polar.theta < 0:
            polar.direction = LEFT
        elif polar.theta > 0:
            polar.direction = RIGHT
        else:
            polar.direction = CENTER

        lines.append(polar)

    return lines

def organize_lines(lines, spots):
    organized = lines_org()
    assigned = [-1, -1, -1, -1]

    for _ in range(2):
        for i in range(4):
            if assigned[i] != -1:
                continue

            closest_line_dist = float('inf')
            closest_line_index = -1
            for j in range(len(lines)):
                position = lines[j].distance * (RIGHT if lines[j].direction == RIGHT else -1)
                dist = abs(position - spots[i])
                if dist < closest_line_dist:
                    closest_line_dist = dist
                    closest_line_index = j

            line_position = lines[closest_line_index].distance * (RIGHT if lines[closest_line_index].direction == RIGHT else -1)
            closest_spot_dist = float('inf')
            closest_spot_index = -1
            for j in range(4):
                dist = abs(spots[j] - line_position)
                if dist < closest_spot_dist:
                    closest_spot_dist = dist
                    closest_spot_index = j

            if closest_spot_index == i or (assigned[closest_spot_index] != -1 and assigned[closest_spot_index] != closest_line_index):
                assigned[i] = closest_line_index

    if assigned[0] != -1:
        organized.far_left = lines[assigned[0]]
    if assigned[1] != -1:
        organized.left = lines[assigned[1]]
    if assigned[2] != -1:
        organized.right = lines[assigned[2]]
    if assigned[3] != -1:
        organized.far_right = lines[assigned[3]]

    return organized

def callback(msg):
    polar_lines = points_to_polar(msg)
    ROW_SPACING = 0.762
    predicted_row_distances = [-ROW_SPACING * 2, -ROW_SPACING, ROW_SPACING, ROW_SPACING * 2]
    organized = organize_lines(polar_lines, predicted_row_distances)
    lines_org_pub.publish(organized)

if __name__ == '__main__':
    rospy.init_node('line_polar_converter')

    line_topic = '/lines'
    organized_topic = '/ekf_lines'

    rospy.Subscriber(line_topic, line_list, callback)
    lines_org_pub = rospy.Publisher(organized_topic, lines_org, queue_size=10)

    rospy.spin()
