# LiDAR_CropRowDetection
This is a crop row detection algorithm based on LiDAR data with pure pursuit as the controller to reach autonomous navigation in the agriculture fields

Check out the video [here](https://youtu.be/FYJuxgDMiHE) or by clicking the image below.


[![Watch the video](https://img.youtube.com/vi/FYJuxgDMiHE/0.jpg)](https://youtu.be/FYJuxgDMiHE)


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
## Visualization
Autonomous navigation in both simulated fields with different crop (corn and soybean) and growth stages (young and grown) and real corn field.

https://github.com/user-attachments/assets/8c187567-b874-4b32-b83d-da6c80f96077

## Citation 
if you use this project in your recent works please refernce to it by:

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
