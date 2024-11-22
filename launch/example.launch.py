from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='testing',
        #     executable='cpp_node',
        #     name='cpp_node'
        # ),
        # Node(
        #     package='testing',
        #     executable='python_node.py',
        #     name='python_node'
        # )
        Node(
            package='testing',
            executable='Lidar_FOV.py',
            name='python_node'
        )
    ])
