# Pose Estimation

This repository contains two solutions for pose estimation:
1) Using [Find Object](https://github.com/introlab/find-object/tree/noetic-devel)\
Find object package relies on the image processing approach to estimate the location of the object in front of the camera. It takes a reference image of the object and tries to find similar points on the input image.

2) Using [DOPE](https://github.com/NVlabs/Deep_Object_Pose)\
Deep Object Pose Estimation (DOPE) relies on the machine learning approach estimate the location of the object in front of the camera. More information on branch _**dope-noetic**_.

## Setup
 * [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
 * [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
 * [Jetson Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)
 * [Realsense](https://github.com/IntelRealSense/realsense-ros#installation-instructions)
 * [Find Object](https://github.com/introlab/find-object)

## Setup the workspace
```bash
cd
mkdir -p pose_estimation/src
cd ~/pose_estimation/src
git clone -b noetic-devel https://github.com/introlab/find-object.git
git clone https://github.com/patsyuk03/pose_estimation.git
cd ~/pose_estimation
catkin build
source devel/setup.bash
```
The workspace contains the launch file _pose_estimation.launch_. This launch file brings up _realsense camera_ and positions it facing downwards using _static transform publisher_. Then it calls _find object_ that starts to detect objects from the image topic published by camera node. Finaly the coordinates are transformed to the correct frame and published to topic _/object_pose_ that can be visualized in RViz. 

### **Start Pose Estimation** 
```bash
roslaunch pose_estimation pose_estimation.launch rviz:=true
```

### **Add New Object** 