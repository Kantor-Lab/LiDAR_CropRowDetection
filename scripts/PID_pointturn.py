#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_interfaces.srv import PointTurn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time
import threading

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
        
        # Create callback groups for concurrent execution
        self.odom_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        
        # Publishers and subscribers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10,
            callback_group=self.odom_callback_group
        )

        # Initialize controllers and state variables
        self.angle_pid = PIDController(2.0, 0.0, 0.1)
        self.dist_pid = PIDController(1.0, 0.0, 0.1)
        
        # State variables
        self.current_angle = 0.0
        self.current_position = [0.0, 0.0]
        self.initial_angle = 0.0
        self.initial_position = None
        self.task_running = False
        self.i = 0  # To track if initial orientation has been set
        
        # Threading and synchronization
        self.lock = threading.Lock()
        
        # Create service with its callback group
        self.service = self.create_service(
            PointTurn,
            'point_turn',
            self.handle_point_turn,
            callback_group=self.service_callback_group
        )
        self.get_logger().info("PointTurn service is ready.")

    def quaternion_to_yaw(self, quaternion):
        x, y, z, w = quaternion
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

    def odom_callback(self, data):
        with self.lock:
            # Update initial position and orientation if i is 0
            if self.i == 0:
                self.initial_position = data.pose.pose.position
                orientation_q = data.pose.pose.orientation
                self.initial_angle = self.quaternion_to_yaw([
                    orientation_q.x, orientation_q.y,
                    orientation_q.z, orientation_q.w
                ])
                self.i += 1

            # Update current position and orientation
            orientation_q = data.pose.pose.orientation
            self.current_angle = self.quaternion_to_yaw([
                orientation_q.x, orientation_q.y,
                orientation_q.z, orientation_q.w
            ]) - self.initial_angle
            
            self.current_position = [
                data.pose.pose.position.x,
                data.pose.pose.position.y
            ]

    async def handle_point_turn(self, request, response):
        if self.task_running:
            response.success = False
            return response

        self.task_running = True
        self.i = 0  # Reset i to get new initial states
        
        try:
            if request.left:
                self.get_logger().info("Performing left turn")
                await self.rotate(math.pi / 2)  # 90 degrees left
                await self.move_forward(1.62)
                await self.rotate(math.pi)      # Another 90 degrees left
            else:
                self.get_logger().info("Performing right turn")
                await self.rotate(-math.pi / 2)  # 90 degrees right
                await self.move_forward(1.62)
                await self.rotate(-math.pi)      # Another 90 degrees right
            
            response.success = True
            self.get_logger().info("Point turn completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error during point turn: {str(e)}")
            response.success = False
        
        finally:
            self.task_running = False
            
        return response

    async def rotate(self, target_angle):
        twist = Twist()
        rate = self.create_rate(20)  # Create rate object at 20Hz
        
        while rclpy.ok():
            with self.lock:
                angle_error = target_angle - self.current_angle
            
            if abs(angle_error) < 0.01:  # Tolerance of 0.05 radians
                break
                
            control_signal = self.angle_pid.update(angle_error)
            twist.angular.z = max(min(control_signal, 1.0), -1.0)  # Limit angular velocity
            self.pub.publish(twist)
            try:
                rate.sleep()  # Note: Changed from await rate.sleep()
            except Exception:
                pass
        
        # Stop rotation
        twist.angular.z = 0.0
        self.pub.publish(twist)

    async def move_forward(self, distance):
        twist = Twist()
        rate = self.create_rate(20)  # Create rate object at 20Hz
        
        while rclpy.ok():
            with self.lock:
                if self.i == 0:
                    try:
                        rate.sleep()  # Note: Changed from await rate.sleep()
                    except Exception:
                        pass
                    continue
                    
                current_distance = math.sqrt(
                    (self.current_position[0] - self.initial_position.x) ** 2 +
                    (self.current_position[1] - self.initial_position.y) ** 2
                )
                dist_error = distance - current_distance
            
            if abs(dist_error) < 0.05:  # Tolerance of 0.05 meters
                break
                
            control_signal = self.dist_pid.update(dist_error)
            twist.linear.x = max(min(control_signal, 0.5), -0.5)  # Limit linear velocity
            self.pub.publish(twist)
            try:
                rate.sleep()  # Note: Changed from await rate.sleep()
            except Exception:
                pass
        
        # Stop movement
        twist.linear.x = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    # Use MultiThreadedExecutor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from robot_interfaces.srv import PointTurn  # Adjust to match your package name
# import math
# import time
# import threading


# class PIDController:
#     def __init__(self, Kp, Ki, Kd):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.prev_error = 0
#         self.integral = 0
#         self.last_time = time.time()

#     def update(self, error):
#         current_time = time.time()
#         delta_time = current_time - self.last_time
#         delta_error = error - self.prev_error

#         self.integral += error * delta_time
#         derivative = delta_error / delta_time if delta_time > 0 else 0

#         output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

#         self.prev_error = error
#         self.last_time = current_time

#         return output


# class RobotController(Node):
#     def __init__(self):
#         super().__init__('robot_controller')
#         self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

#         # Initialize variables for odometry and service
#         self.current_angle = 0.0
#         self.current_position = [0.0, 0.0]
#         self.angle_pid = PIDController(2.0, 0.0, 0.1)
#         self.dist_pid = PIDController(1.0, 0.0, 0.1)

#         self.initial_angle = 0
#         self.angle_after_turn = 0
#         self.i = 0  # To track if initial orientation has been set
#         self.j = 0  # To track if initial position has been set
#         self.initial_orientation = None
#         self.initial_position = None

#         # Define the service
#         self.service = self.create_service(PointTurn, 'point_turn', self.handle_point_turn)
#         self.get_logger().info("PointTurn service is ready.")

#         self.lock = threading.Lock()

#     def handle_point_turn(self, request, response):
#         # Handle service requests concurrently
#         self.i = 0
#         self.j = 0
#         thread = threading.Thread(target=self.run_task, args=(request.left, response))
#         thread.start()
#         print("response", response)
#         return response  # Respond immediately to acknowledge receipt

#     def quaternion_to_yaw(self, quaternion):
#         x, y, z, w = quaternion
#         return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

#     def odom_callback(self, data):
#         # Update odometry data and synchronize the state updates
#         with self.lock:
#             if self.i == 0:
#                 # print("getting initial angle")
#                 self.initial_orientation = data.pose.pose.orientation
#                 self.initial_angle = self.quaternion_to_yaw([
#                     self.initial_orientation.x,
#                     self.initial_orientation.y,
#                     self.initial_orientation.z,
#                     self.initial_orientation.w
#                 ])
#                 # print("getting initial position")
#                 self.initial_position = data.pose.pose.position
#                 self.i += 1

#             if self.j == 0:
#                 # print("getting initial position")
#                 self.initial_position = data.pose.pose.position
#                 self.j += 1

#             orientation_q = data.pose.pose.orientation
#             self.current_angle = self.quaternion_to_yaw([
#                 orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
#             ])
#             self.current_angle -= self.initial_angle
#             self.current_position = [data.pose.pose.position.x, data.pose.pose.position.y]

#             # Update the initial position and orientation after each task
            

#     def rotate(self, target_angle):
#         twist = Twist()
#         while rclpy.ok():
#             with self.lock:
#                 angle_error = target_angle - self.current_angle
#             control_signal = self.angle_pid.update(angle_error)
#             twist.angular.z = control_signal
#             self.pub.publish(twist)

#             if abs(angle_error) < 0.05:
#                 break
#         twist.angular.z = 0.0
#         self.pub.publish(twist)

#     def move_forward(self, distance):
#         twist = Twist()
#         while rclpy.ok():
#             with self.lock:
#                 dist_error = distance - math.sqrt(
#                     (self.current_position[0] - self.initial_position.x) ** 2 +
#                     (self.current_position[1] - self.initial_position.y) ** 2
#                 )
#             control_signal = self.dist_pid.update(dist_error)
#             twist.linear.x = control_signal
#             self.pub.publish(twist)

#             if abs(dist_error) < 0.05:
#                 break
#         twist.linear.x = 0.0
#         self.pub.publish(twist)

#     def run_task(self, left, response):
#         try:
#             if left:
#                 self.get_logger().info("Performing left turn")
#                 self.rotate(math.pi / 2)  # Rotate 90 degrees left
#                 self.move_forward(1.72)  # Move forward 1.72 meters
#                 self.rotate(math.pi)     # Rotate another 90 degrees left
#             else:
#                 self.get_logger().info("Performing right turn")
#                 self.rotate(-math.pi / 2)  # Rotate 90 degrees right
#                 self.move_forward(1.72)    # Move forward 1.72 meters
#                 self.rotate(-math.pi)      # Rotate another 90 degrees right

#             self.get_logger().info("Finished point turn task")
#             response.success = True
#         except Exception as e:
#             self.get_logger().error(f"Error during task execution: {e}")
#             response.success = False

#     def shutdown(self):
#         self.get_logger().info("Shutting down robot controller...")
#         rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     robot_controller = RobotController()
#     try:
#         rclpy.spin(robot_controller)
#     except KeyboardInterrupt:
#         robot_controller.get_logger().info("Shutting down robot controller")
#     finally:
#         robot_controller.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
