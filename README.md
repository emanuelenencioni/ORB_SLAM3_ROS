# ROS wrapper for ORB-SLAM3

A ROS wrapper for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). Based on [this package](https://github.com/thien94/orb_slam3_ros_wrapper). This is a modified version of ORB SLAM3, which includes a method in the API that returns all the map points in the active map.

Tested with ORB-SLAM3 V1.0, primarily on Ubuntu 20.04.

# Installation

General guide: first, install all of ORB SLAM3 dependencies. Then, install this package in a ```catkin build``` environment.

## 1. ORB-SLAM3

- Install the [prerequisites](https://github.com/UZ-SLAMLab/ORB_SLAM3#2-prerequisites). Make sure to download Pangolin release (0.8), opencv 4.4.

- Make sure that **`libORB_SLAM3.so`** is created in the *ORB_SLAM3/lib* folder. If not, check the issue list from the [original repo](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues) and retry.

## 2. ORB_SLAM3_ROS

- Clone the package. Note that it should be a `catkin build` workspace.
```
cd ~/catkin_ws/src/
git clone https://github.com/emanuelenencioni/ORB_SLAM3_ROS.git
```
- Build ORB_SLAM3:
```
cd /ORB_SLAM3_ROS/ORB_SLAM3
chmod +x build.sh
./build.sh

```

- Build the package normally.
```
cd ~/catkin_ws/
catkin build
```

- (Optional) Install `hector-trajectory-server` to visualize the trajectory.
```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

- If everything works fine, you can now try the different launch files in the `launch` folder.

## 3. How to run

### EuRoC dataset:

- In one terminal, launch the node:
```
roslaunch orb_slam3_ros_wrapper euroc_monoimu.launch
```
- In another terminal, playback the bag:
```
rosbag play MH_01_easy.bag
```
Similarly for other sensor types.

# Topics
The following topics are published by each node:
- `/orb_slam3/map` ([`PointCloud2`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): all point of the map.
- `/orb_slam3/camera_pose` ([`PoseStamped`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)): current left camera pose in world frame, as returned by ORB-SLAM3.
- `tf`: transformation from camera frame to world frame.

P.S. There is also Italian [documentation](https://github.com/emanuelenencioni/ORB_SLAM3_ROS/blob/main/doc/documentation.pdf) available  that explains the code and includes evaluations of the system.
