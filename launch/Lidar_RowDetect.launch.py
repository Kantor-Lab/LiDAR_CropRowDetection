from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        # Log info (optional for debugging)
        LogInfo(
            condition=None,
            msg="Starting LiDAR processing nodes"
        ),

        # Filter based on LiDAR FOV
        Node(
            name="LiDAR_FOV",
            package="Lidar_RowDetect",
            executable="Lidar_FOV.py",
            output="screen"
        ),

        # Filter out points below virtual plane
        Node(
            name="Virtual_plane",
            package="Lidar_RowDetect",
            executable="Virtual_groundplane.py",
            output="screen"
        ),

        # K-means clustering
        Node(
            name="K_means",
            package="Lidar_RowDetect",
            executable="Row_Detection.py",
            output="screen"
        ),
        
        # RANSAC line fitting
        Node(
            name="RANSAC",
            package="Lidar_RowDetect",
            executable="Ransac_fittingline.py",
            output="screen"
        ),
        
        # Ground Truth marker
        Node(
            name="marker",
            package="Lidar_RowDetect",
            executable="markers.py",
            output="screen"
        ),
        
        # RealTime Odom
        Node(
            name="Change_odom",
            package="Lidar_RowDetect",
            executable="change_odom.py",
            output="screen"
        ),
        
        # EKF
        Node(
            name="EKF",
            package="Lidar_RowDetect",
            executable="waypoints.py",
            output="screen"
        ),

        # RViz visualization
        Node(
            name="rviz",
            package="rviz2",
            executable="rviz2",
            arguments=["-d", "$(find Lidar_RowDetect)/config/rowdetection.rviz"],
            output="screen"
        ),
    ])
