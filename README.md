# LiDAR Crop Row Detection In ROS2-Humble

## ROS2 Humble example
check [here] for the subscriber, publisher, Node initialization example

check [here](https://github.com/Ruiji-Liu/robot_interfaces)  for the msg and srv example
## Setting up the package
Download and set up the Ubuntu 22.04 and [ros2 humble](https://docs.ros.org/en/humble/Installation.html) first.
### Other Required Packages
[Cuml Package](https://docs.rapids.ai/install)  (Recommend install it in conda environments)  
[Amiga simulation environments](https://github.com/Kantor-Lab/Amiga_Simulation-Environments/tree/ros2-humble) 
### Install the package
```
mkdir -p amiga_ws/src
cd amiga_ws/src
git clone -b ros2-humble https://github.com/Kantor-Lab/LiDAR_CropRowDetection.git
cd ~/amiga_ws && colcon build
```
### Launch the package
```
ros2 launch lidar_rowdetect Lidar_RowDetect.launch.py
```

