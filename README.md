# LiDAR_CropRowDetection
This is a crop row detection algorithms based on LiDAR data
## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/Kantor-Lab/LiDAR_CropRowDetection.git
cd ~/catkin_ws && catkin_make
```
## Other Required Packages
[Amiga simulation environments](https://github.com/Kantor-Lab/Amiga_Simulation-Environments.git)

## Running
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch Lidar_RowDetect Lidar_RowDetect.launch
```
## Visualization
Visualization of the LiDAR row detection algorithm (top) while robot operating in the simulated young soybean fields (bottom). The visualization tool provides
information on detected crop positions in the current robot frame(red points), crop ground truth positions(green points), and predicted crop row positions and orientations(blue lines)
![visualization](https://github.com/Kantor-Lab/LiDAR_CropRowDetection/assets/78890103/5b16d715-b282-44e9-83a9-f7a187468eab)

