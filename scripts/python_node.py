#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class PythonNode(Node):
    def __init__(self):
        super().__init__('python_node')
        self.get_logger().info('Python Node is running!')

def main(args=None):
    rclpy.init(args=args)
    node = PythonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
