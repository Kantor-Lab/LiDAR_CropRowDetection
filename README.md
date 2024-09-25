# LiDAR_CropRowDetection
This is a crop row detection algorithm based on LiDAR data with a nonlinear Model Predictive Control (MPC) as the controller to reach autonomous navigation in the agriculture fields.

[[Project page]](https://ruiji-liu.github.io/crop_row_detection.github.io/)
[[Paper]](https://arxiv.org/abs/2403.17774)
[[Video]](https://youtu.be/FYJuxgDMiHE)

## Visualization
Autonomous navigation by using this navigation system in both simulated fields featuring various crops (corn and soybean) at different growth stages (young and mature), as well as in a real corn field.

https://github.com/user-attachments/assets/8c187567-b874-4b32-b83d-da6c80f96077
## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/Kantor-Lab/LiDAR_CropRowDetection.git
cd ~/catkin_ws && catkin_make
```
## Other Required Packages
[Cuml Package](https://docs.rapids.ai/install)  (Recommend install it in conda environments)  
[Amiga simulation environments](https://github.com/Kantor-Lab/Amiga_Simulation-Environments.git)  

## Running
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch Lidar_RowDetect Lidar_RowDetect.launch
```


## Citation 
if you have used this project in your recent works please reference to it:

```bash

@misc{liu2024overcanopyautonomousnavigationcropagnostic,
      title={Towards Over-Canopy Autonomous Navigation: Crop-Agnostic LiDAR-Based Crop-Row Detection in Arable Fields}, 
      author={Ruiji Liu and Francisco Yandun and George Kantor},
      year={2024},
      eprint={2403.17774},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2403.17774}, 
}
