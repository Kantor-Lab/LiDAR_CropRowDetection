#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import PointTurn
import threading

class PointTurnClient(Node):
    def __init__(self):
        super().__init__('point_turn_client')
        self.client = self.create_client(PointTurn, 'point_turn')
        self.response_received = threading.Event()

    def send_request(self, left_turn: bool):
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            if not rclpy.ok():
                return False

        # Create request
        request = PointTurn.Request()
        request.left = left_turn
        self.get_logger().info(f"Sending point turn request. Left turn: {left_turn}")

        # Send request
        future = self.client.call_async(request)
        
        print("response callback")
        # Add callback for when response is received
        future.add_done_callback(self.response_callback)
        
        return future

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Point turn completed successfully')
            else:
                self.get_logger().error('Point turn failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed {str(e)}')
        finally:
            self.response_received.set()

def main(args=None):
    rclpy.init(args=args)
    client = PointTurnClient()
    
    # Start spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,))
    spin_thread.start()

    try:
        # Send request
        future = client.send_request(left_turn=False)
        
        # Wait for response
        client.response_received.wait()
        
    finally:
        # Cleanup
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
# import rclpy
# from rclpy.node import Node
# from robot_interfaces.srv import PointTurn  # Adjust to match your package name
# import time

# class PointTurnClient(Node):
#     def __init__(self):
#         super().__init__('point_turn_client')

#         # Create a client for the PointTurn service
#         self.client = self.create_client(PointTurn, 'point_turn')

#         # Wait for the service to be available
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service not available, waiting again...')

#         # Create a request object
#         self.request = PointTurn.Request()

#     def send_request(self, left_turn: bool):
#         self.request.left = left_turn
#         self.get_logger().info(f"Sending point turn request. Left turn: {left_turn}")

#         # Call the service asynchronously and get the future object
#         future = self.client.call_async(self.request)

#         # Monitor the future status
#         response = future.result()
#         print("response.success", response.success)
#         while rclpy.ok() and not future.done():
#             self.get_logger().info("Waiting for service to complete...")
#             time.sleep(0.1)  # Add a small delay to avoid overloading the CPU

#         # Process the result after the future completes
#         self.on_service_response(future)

#     def on_service_response(self, future):
#         try:
#             response = future.result()
#             if response.success:
#                 self.get_logger().info('Point turn completed successfully.')
#                 print("Service response: Success")
#             else:
#                 self.get_logger().info('Point turn failed.')
#                 print("Service response: Failed")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed with exception: {e}")
#             print(f"Service call failed with exception: {e}")

# def main(args=None):
#     rclpy.init(args=args)

#     # Create the client node
#     client_node = PointTurnClient()

#     # Send a request for a left turn
#     client_node.send_request(left_turn=False)

#     # Shutdown rclpy
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
