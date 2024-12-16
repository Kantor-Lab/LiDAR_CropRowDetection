from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
lidar_rowdetect = get_package_share_directory('lidar_rowdetect')
# rviz2_config = PathJoinSubstitution([lidar_rowdetect, 'rviz', 'rowdetection.rviz'])
rviz2_config = PathJoinSubstitution([FindPackageShare("lidar_rowdetect"), "rviz", "rowdetection.rviz"])

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
        # Node(
        #     package='testing',
        #     executable='Lidar_FOV.py',
        #     name='python_node'
        # )
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             parameters=[{'use_sim_time': True}],
            #  remappings=[
            #     ('/tf', 'tf'),
            #     ('/tf_static', 'tf_static')
            #  ],
             output='screen'),
    ])
