# Robothon 2023 Grand-Challenge REAL Team Report

<p align="center">
<strong>Authors: Yiting HE, Yuyang ZHANG, Linyi HUANG, Ronghe QIU</strong>
</p>

This repository contains the source code of the REAL Team's participation in the Robothon 2023 Grand Challenge competition.


# Hardware Setup
The robot platform utilized by our team in the Robothon Grand Challenge is illustrated in the figure below.
<p align="center">
  <img height="600" src="https://github.com/harryting-yiting/Robothon_REAL/blob/main/Assets/Hardware_Overview.jpeg">
</p>

The hardware system is composed of four primary components:

* **UR 5e Collaborative Robot**: A six-degree-of-freedom collaborative robot mounted on a fixed table, designed to ensure precise and efficient movement.
* **RealSense D435 RGBD Camera**: Mounted on the last joint of the robot, this camera supplies crucial RGB information to our system, enabling accurate object recognition and localization.
* **Two-Finger Gripper**: A gripper that is mounted on the final joint of the robot and provides versatile manipulation capabilities.
* **Self-designed End Effector**: A end effector that is design for special purpose of the task. It is mounted on the two-finger gripper.



# Software modules
We have divided the software into five separate sub workspaces, each containing a driver or a module. The primary code developed for the competition is stored in the robot main workspace to minimize re-compilation and reduce dependency between various drivers.

* **ws_realsense_driver**: Realsense driver
* **ws_ur_driver**: UR 5 driver
* **ws_handeye_callibration**: Hand-eye calibration
* **ws_robot_main**: Robot main (contains detection and planning)
* **ws_vision**: Vision-related codes


## System Requirements

* Ubuntu 20.04
* ROS1 Noetic
* MoveIt 1 Noetic


## Software Dependency
The following are the third-party modules utilized by the REAL Team:
- [ROS](https://www.ros.org/)
- [Universal robot ros driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [Intel realsense ros driver](https://github.com/IntelRealSense/realsense-ros)
- [UR RTDE Control](https://sdurobotics.gitlab.io/ur_rtde/)
- [Hand-eye Calibration](https://github.com/JStech/moveit_calibration)
- [Yolo v5](https://github.com/ultralytics/yolov5)


## Build Workspaces

### 1. Build ws_realsense_driver
#### 1.1 Install librealsense2

* Register the server's public key:

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
```
* Add the server to the list of repositories:
```bash
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

* Install the libraries :
```bash
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```
#### 1.2 Build Workspace
```bash
cd <your project directory>/ws_realsense_driver
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source <your install path>/Robothon_REAL/ws_realsense_driver/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
> The above code is adopted from
> https://github.com/leggedrobotics/realsense-ros-rsl
> Choose Method 2: The RealSense™ distribution:

#### 1.3 Install Realsense for python SDK

```bash
pip install pyrealsense2
```

### 2. Build ws_ur_driver
#### 2.1 Install Dependencies

```bash
cd <your install path>/Robothon_REAL/ws_ur_driver
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
```
##### 2.2 Build Workspace
```bash
cd <your project directory>/ws_ur_driver
catkin_make
echo "source <your install path>/Robothon_REAL/ws_ur_driver/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> The above code is adopted from
> https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

### 3. Build ws_handeye_callibration

**NOTE:** The code which is alrealy in the folder `ws_handeye_callibration/src`

### 4. Build ws_robot_main
#### 4.1 Install RS485 Gripper Dependency

```bash
sudo apt install python3-pip
pip install pyserial
sudo usermod -aG dialout $USER
#reboot
```

#### 4.2 Install ur_rtde package
```bash
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
pip install --user ur_rtde
```
> The code above is adopted from
> https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html

#### 4.3 Build Workspace
```bash
catkin_make
```
### 5. Build ws_vision
```bash
cd <your project directory>/ws_vision
catkin_make
```

## Set up Paths
```bash
cd <your project directory>
echo "source <your project directory>/ws_handeye_callibration/devel/setup.bash" >> ~/.bashrc
echo "source <your project directory>/ws_realsense_driver/devel/setup.bash" >> ~/.bashrc
echo "source <your project directory>/ws_ur_driver/devel/setup.bash" >> ~.bashrc
echo "source <your project directory>/ws_robot_main/devel/setup.bash" >> ~.bashrc
echo "source <your project directory>/ws_vision/devel/setup.bash" >> ~.bashrc
```

## Launch 

**NOTE** Please run each launch in a seperate terminal
<!-- 1. Launch UR Controller and Driver

```bash
cd ~/Desktop/Projects/Robothon_ROS1/ws_ur_driver/src/hkustgz_ur_launch/launch
roslaunch highbay_ur5e.launch 
``` -->
1. Launch ur_rtde controller
```bash
rosrun robot_main ur_rtde_test.py
```
After this step, you can connect the robot control panel to ROS.

<!-- 2. Launch Moveit Planner and RVIZ
```bash
roslaunch ur5e_moveit_config moveit_planning_execution.launch
```
```bash
roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz
``` -->
2. Launch DH Gripper
```bash
rosrun robot_main dhGripperActionServer.py
rosrun actionlib_tools axclient.py /dhgripper_action
```

3. Launch the Camera Driver
For Realsense-ROS based algorithm:
```bash
cd ~
roslaunch realsense2_camera rs_camera.launch
```

For button detection based CV algorithm:
```bash
roslaunch robothon2023_vision board_loc.launch
```

For screen detection based Improved-YOLOv5 algorithm:
```bash
roslaunch yolov5_ros yolov5.launch
cd ~/Robothon_REAL/ws_vision/src/yolov5_ros/src
python bbox_subscriber.py
```

4. Launch Our Magic Code
```bash
cd ~/Desktop/Projects/Robothon_ROS1/ws_robot_main
source devel/setup.bash
rosrun robot_main main.py
```

5. Start Planning
```bash
rostopic pub /StartPlanning std_msgs/Bool True
```

# Authors

- Yiting HE, [Harryting](https://github.com/harryting-yiting)
- Yuyang ZHANG, [Ryan Zhang](https://github.com/RyanZ328)
- Linyi HUANG, [Lucky Huang](https://github.com/Hly-123)
- Ronghe QIU, [ConnerQiu](https://github.com/ConnerQiu)


