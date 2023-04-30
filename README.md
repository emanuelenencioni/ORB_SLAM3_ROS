# ROS wrapper for ORB-SLAM3

A ROS wrapper for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). The main idea is to use the ORB-SLAM3 as a standalone library and interface with it instead of putting everything together. For that, you can check out [this package](https://github.com/thien94/orb_slam_3_ros).

Tested with ORB-SLAM3 V1.0, primarily on Ubuntu 20.04.

- **Pros**:
  - Easy to update [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3#orb-slam3) indepedently.
  - Easy to replace different variants that are not built for ROS.
- **Cons**:
  - Dependent on the exposed APIs from ORB-SLAM3.
  - Development involves more steps (1. Make changes in ORB-SLAM3 library -> 2. Build ORB-SLAM3 -> 3. Change the roswrapper if necessary -> 4. Test).
  - Might break when dependencies or upstream changes.


# Installation

General guide: first, install ORB-SLAM3 normally with all of its dependencies (any location is fine). Then, install this package in a ```catkin build``` environment.

## 1. ORB-SLAM3

- Install the [prerequisites](https://github.com/UZ-SLAMLab/ORB_SLAM3#2-prerequisites). Make sure to download Pangolin release (0.8), opencv 4.4.

- Make sure that **`libORB_SLAM3.so`** is created in the *ORB_SLAM3/lib* folder. If not, check the issue list from the [original repo](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues) and retry.

## 2. orb_slam3_ros_wrapper

- Clone the package. Note that it should be a `catkin build` workspace.
```
cd ~/catkin_ws/src/
git clone https://github.com/thien94/orb_slam3_ros_wrapper.git
```
- Build ORB_SLAM3:
```
cd /orb_slam3_ros_wrapper/ORB_SLAM3
chmod +x build.sh
./build.sh

```

- Build the package normally.
```
cd ~/catkin_ws/
catkin build
```

- Next, copy the `ORBvoc.txt` file from `ORB-SLAM3/Vocabulary/` folder to the `config` folder in this package. Alternatively, you can change the `voc_file` param in the launch file to point to the right location.

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
- `/orb_slam3/map_points` ([`PointCloud2`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)): all keypoints being tracked.
- `/orb_slam3/camera_pose` ([`PoseStamped`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)): current left camera pose in world frame, as returned by ORB-SLAM3.
- `tf`: transformation from camera frame to world frame.
