#!/usr/bin/env python3
#rosservice call /point_turn "{left: true}"

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import math
import numpy as np
from Lidar_RowDetect.srv import PointTurn, PointTurnResponse
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        delta_error = error - self.prev_error

        self.integral += error * delta_time
        derivative = delta_error / delta_time if delta_time > 0 else 0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        self.last_time = current_time

        return output

class RobotController:
    def __init__(self):
        rospy.init_node('pid_controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=1)
        self.current_angle = 0.0
        self.current_position = [0.0, 0.0]
        
        self.angle_pid = PIDController(0.6, 0.0, 0.1)
        self.dist_pid = PIDController(0.6, 0.0, 0.1)
        
        self.rate = rospy.Rate(10)
        
        self.initial_angle = 0
        self.i = 0
        self.j = 0
        self.initial_orientation = None
        self.initial_position = None

        self.service = rospy.Service('point_turn', PointTurn, self.handle_point_turn)
    def odom_callback(self, data):
        if self.i == 0:
            self.initial_orientation = data.pose.pose.orientation
            _, _, self.initial_angle = euler_from_quaternion([self.initial_orientation.x, self.initial_orientation.y,
                                                              self.initial_orientation.z,self.initial_orientation.w])
            self.initial_position = data.pose.pose.position
            self.i += 1
        if self.j == 0:
            self.initial_position = data.pose.pose.position
            self.j += 1
        if self.initial_orientation:
            orientation_q = data.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            _, _, self.current_angle = euler_from_quaternion(orientation_list)
            self.current_angle -= self.initial_angle
            self.current_position = [data.pose.pose.position.x, data.pose.pose.position.y]
    def rotate(self, target_angle):
        twist = Twist()
        while not rospy.is_shutdown():
            self.current_angle = math.atan2(math.sin(self.current_angle), math.cos(self.current_angle))
            angle_error = target_angle - self.current_angle
            # angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            print("target angle", target_angle)
            print("current angle", self.current_angle)

            # print(angle_error)
            # angle_error = target_angle - self.current_angle
            print(angle_error)
            control_signal = self.angle_pid.update(angle_error)
            
            twist.angular.z = control_signal
            self.pub.publish(twist)
            
            if abs(angle_error) < 0.01:
                break
            
            self.rate.sleep()
        # self.i = 0
        twist.angular.z = 0
        self.pub.publish(twist)
    
    def move_forward(self, distance):
        # start_position = self.current_position[:]
        twist = Twist()
        while not rospy.is_shutdown():
            print("current position", self.current_position[0], self.current_position[1])
            print("initial position", self.initial_position.x, self.initial_position.y)
            dist_error = distance - math.sqrt((self.current_position[0] - self.initial_position.x)**2 +
                                              (self.current_position[1] - self.initial_position.y)**2)
            print("dist error", dist_error)
            control_signal = self.dist_pid.update(dist_error)
            
            twist.linear.x = control_signal
            self.pub.publish(twist)
            
            if abs(dist_error) < 0.01:
                break
            
            self.rate.sleep()
        self.j = 0
        twist.linear.x = 0
        self.pub.publish(twist)
    
    def run(self, left):
        if left:
            print("left turn")
            # rospy.sleep(1)
            # self.move_forward(2.0)    # Move forward 2 meters
            rospy.sleep(1)
            self.rotate(math.pi / 2)  # Rotate 90 degrees
            rospy.sleep(1)
            self.move_forward(1.62)    # Move forward 2 meters
            rospy.sleep(1)
            self.rotate(math.pi)  # Rotate another 90 degrees
            rospy.sleep(1)
            # self.move_forward(2.0)    # Move forward 2 meters
            # rospy.sleep(1)
            # self.rotate(3 * math.pi/2)  # Rotate another 90 degrees
            # rospy.sleep(1)
            # self.move_forward(2.0)    # Move forward 2 meters
            # rospy.sleep(1)
            # self.rotate(2 * math.pi)  # Rotate another 90 degrees
            # rospy.sleep(1)
        else:
            print("right turn")
            # rospy.sleep(1)
            # self.move_forward(2.0)    # Move forward 2 meters
            rospy.sleep(1)
            self.rotate(-math.pi / 2)  # Rotate 90 degrees
            rospy.sleep(1)
            self.move_forward(1.62)    # Move forward 2 meters
            rospy.sleep(1)
            self.rotate(-math.pi)  # Rotate another 90 degrees
            rospy.sleep(1)
            # self.move_forward(2.0)    # Move forward 2 meters
            # rospy.sleep(1)
    def handle_point_turn(self, req):
        # Call the run method based on the service request
        self.i = 0
        self.j = 0
        self.initial_orientation = None
        self.initial_position = None
        self.run(req.left)
        return PointTurnResponse(success=True)  # Return a success response
if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        # robot_controller.run(1,0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
