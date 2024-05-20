# Pose Estimation

The goal of the project is to estimate the location of the object using camera. In course of compleating the project two approaches were tested. First approach utilises [Find Object](https://github.com/introlab/find-object/tree/noetic-devel) ROS package to estimate objerct's position. This approach relies on image processing and does not require much time for setup, however, it is heavily dependent on fixed camera and lightsource locations. Second approach utilises Deep Object Pose Estimation ([DOPE](https://github.com/NVlabs/Deep_Object_Pose)) where object's pose is detected by a pretrained model. If the model is trained enough, the detection in case of the second approach, can be more reliable in comparison to the first one. However, the training time can take a very long time.

### Test Objects
 <img src="./media/test_objects.jpg" width="400" height="400" />


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

 <img src="./media/setup.jpg" width="300" height="400" />

## Setup the workspace
```bash
sudo apt-get install ros-$ROS_DISTRO-find-object-2d
```
```bash
cd
mkdir -p pose_estimation/src
cd ~/pose_estimation/src
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
<img src="./media/demo.gif" width="690" height="380" />

### **Add New Object** 
```bash
roslaunch pose_estimation pose_estimation.launch add_objects:=true
```
<img src="./media/add_object.gif" width="552" height="474" />
