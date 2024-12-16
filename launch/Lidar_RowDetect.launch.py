from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
lidar_rowdetect = get_package_share_directory('lidar_rowdetect')
rviz2_config = PathJoinSubstitution([lidar_rowdetect, 'rviz', 'rowdetection.rviz'])
def generate_launch_description():
    return LaunchDescription([
        # Log info (optional for debugging)
        LogInfo(
            condition=None,
            msg="Starting LiDAR processing nodes"
        ),

        # Filter out points below virtual plane
        Node(
            name="Virtual_plane",
            package="lidar_rowdetect",
            executable="Virtual_groundplane.py",
            output="screen"
        ),

        # K-means clustering
        Node(
            name="K_means",
            package="lidar_rowdetect",
            executable="Row_Detection.py",
            output="screen"
        ),
        
        # RANSAC line fitting
        Node(
            name="RANSAC",
            package="lidar_rowdetect",
            executable="Ransac_fittingline.py",
            output="screen"
        ),
        
        # Ground Truth marker
        # Node(
        #     name="marker",
        #     package="Lidar_RowDetect",
        #     executable="markers.py",
        #     output="screen"
        # ),
        
        # RealTime Odom
        Node(
            name="Change_odom",
            package="lidar_rowdetect",
            executable="change_odom.py",
            output="screen"
        ),

        # RViz visualization
        # Node(
        #     name="rviz",
        #     package="rviz2",
        #     executable="rviz2",
        #     arguments=["-d", "$(find lidar_rowdetect)/config/test.rviz"],
        #     output="screen"
        # ),
        
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
