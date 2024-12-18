#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_interfaces.srv import PointTurn  # Adjust to match your package name
import math
import time
import threading  # Import threading for running operations in a separate thread


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


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.current_angle = 0.0
        self.current_position = [0.0, 0.0]
        self.angle_pid = PIDController(2.0, 0.0, 0.1)
        self.dist_pid = PIDController(1.0, 0.0, 0.1)

        self.initial_angle = 0
        self.angle_after_turn = 0
        self.i = 0
        self.j = 0
        self.initial_orientation = None
        self.initial_position = None

        # Define the service
        self.service = self.create_service(PointTurn, 'point_turn', self.handle_point_turn)
        self.get_logger().info("PointTurn service is ready.")

    def quaternion_to_yaw(self, quaternion):
        # Manually compute yaw if necessary
        x, y, z, w = quaternion
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
        return yaw

    def odom_callback(self, data):
        if self.i == 0:
            self.initial_orientation = data.pose.pose.orientation
            self.initial_angle = self.quaternion_to_yaw([self.initial_orientation.x,
                                                          self.initial_orientation.y,
                                                          self.initial_orientation.z,
                                                          self.initial_orientation.w])
            
            self.initial_position = data.pose.pose.position
            self.i += 1
        if self.j == 0:
            self.initial_position = data.pose.pose.position
            self.j += 1
        if self.initial_orientation:
            orientation_q = data.pose.pose.orientation
            self.current_angle = self.quaternion_to_yaw([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            self.current_angle -= self.initial_angle
            # self.current_angle -= self.angle_after_turn
            self.current_position = [data.pose.pose.position.x, data.pose.pose.position.y]
            # print("current angle", self.current_angle)
            # print("current position", self.current_position)
    def rotate(self, target_angle):
        twist = Twist()
        while rclpy.ok():
            # print("current angle", self.current_angle)
            self.current_angle = math.atan2(math.sin(self.current_angle), math.cos(self.current_angle))
            angle_error = target_angle - self.current_angle
            print("target angle", target_angle)
            print("current angle", self.current_angle)
            print(angle_error)
            control_signal = self.angle_pid.update(angle_error)
            
            twist.angular.z = control_signal
            self.pub.publish(twist)

            if abs(angle_error) < 0.1:
                break
        twist.angular.z = 0.0
        self.angle_after_turn = self.current_angle
        # print("angle after turn", self.angle_after_turn)
        self.pub.publish(twist)

    def move_forward(self, distance):
        print("moving forward")
        twist = Twist()
        while rclpy.ok():
            dist_error = distance - math.sqrt((self.current_position[0] - self.initial_position.x) ** 2 +
                                              (self.current_position[1] - self.initial_position.y) ** 2)
            control_signal = self.dist_pid.update(dist_error)

            twist.linear.x = control_signal
            self.pub.publish(twist)

            if abs(dist_error) < 0.1:
                break

        self.j = 0
        twist.linear.x = 0.0
        self.pub.publish(twist)

    def run(self, left):
        if left:
            self.get_logger().info("Performing left turn")
            time.sleep(1)
            self.rotate(math.pi / 2)  # Rotate 90 degrees
            time.sleep(1)
            self.move_forward(1.62)  # Move forward 1.62 meters
            time.sleep(1)
            self.rotate(math.pi)  # Rotate another 90 degrees
            time.sleep(1)
        else:
            self.get_logger().info("Performing right turn")
            time.sleep(1)
            self.rotate(-math.pi / 2)  # Rotate 90 degrees to the right
            time.sleep(1)
            self.move_forward(1.62)  # Move forward 1.62 meters
            time.sleep(1)
            self.rotate(-math.pi)  # Rotate another 90 degrees to the right
            time.sleep(1)

    def handle_point_turn(self, request, response):
        # Launch the `run()` operation in a separate thread to avoid blocking
        self.i = 0
        self.j = 0
        thread = threading.Thread(target=self.run, args=(request.left,))
        thread.start()
        
        # Respond immediately to the service request
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Shutting down robot controller")
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
