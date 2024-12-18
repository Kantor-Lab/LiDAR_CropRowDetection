# LiDAR Crop Row Detection In ROS2-Humble

## ROS2 Humble example
check [here] for the subscriber, publisher, Node initialization example

check [here] for the msg and srv example
## Try it
Download and set up the [ros2 humble](https://docs.ros.org/en/humble/Installation.html) first.
### Other Required Packages
[Cuml Package](https://docs.rapids.ai/install)  (Recommend install it in conda environments)  
[Amiga simulation environments](https://github.com/Kantor-Lab/Amiga_Simulation-Environments.git) 
### Install the package
```
mkdir -p amiga_ws/src
cd amiga_ws/src
git clone -b ros2-humble https://github.com/Kantor-Lab/LiDAR_CropRowDetection.git
cd ~/amiga_ws && colcon build
```
### Launch the world
```
ros2 launch amiga_gazebo amiga_playen.launch.py
```
### Check
Publish velocity message to /cmd_vel, the Amiga robot should move.
