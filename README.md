# Wheel-Legged SLAM
We proposed the Wheel-Legged SLAM: an indoor LiDAR-Inertial SLAM integrating kinematic model of wheel-legged robots, which contains:

1) a taylored dual LiDAR system (including LIVOX AVIA and LIVOX MID360) to extend the robot's perception range.
2) Two innovative factors in the backend optimization.

Please refer to the paper "Wheel-Legged SLAM: an indoor LiDAR-Inertial SLAM integrating kinematic model of wheel-legged robots" for more details.

In this repository, we provide 3 packages:
1) wheellegged_salm: This algorithm integrates FAST-LIO as the frontend and ISAM2 as the backend. The topic subscribed in this package include: a) the merged LiDAR topic: /livox/lidar, b) the IMU topic from AVIA: /livox/imu_3JEDL8C00164531, c) the joint angle topic of the wheel-legged robot: /joint_angle.
2) CloudMerging: This package fuses the AVIA's lidar topic and MID360's lidar topic together and publishes: a) the topic "/livox/lidar" in "livox_ros_driver::CustomMsg", b) the topic "/test_pointcloud" in "sensor_msgs::PointCloud2".
3) usbcan: Considering that the joint information of our wheel-legged robot uses the CAN protocol, we have adopted the ZLG USBCAN-II hardware for serial data conversion. The code in this package converts the joint information from the CAN protocol into ROS topics. It is required when performing data collection or real-time experiments.

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
The Package works in the Ubuntu 20.04 environment with the corresponding version of ROS.

### 1.2. **PCL && Eigen**
We use the PCL >= 1.8 and Eigen >= 3.3.4.

### 1.3. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

### 1.4. **livox_ros_driver2**
Follow [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2)

### 1.5. **Sophus**
We use the old version of Sophus
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ../ -DUSE_BASIC_LOGGING=ON
make
sudo make install
```


## 2. Build Wheel-legged SLAM
Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/Karltommy/WL-SLAM.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 3. Rosbag
We provide a package in the corridor scene to facilitate the running of our algorithm. Please click the link to acquire this bag file: https://drive.google.com/drive/folders/1PB3HYFsAgZApcqUqrHr1okaR-4f-VVhb?usp=sharing


## 4. Run
Run the CloudMerging launch file to obtain the merged LiDAR topic .
```
roslaunch CloudMerging cloudmerg.launch
```
Run the Wheel-Legged SLAM algorithm.
```
roslaunch wheellegged_slam run.launch
```
Run the rosbag file.
```
rosbag play corridor.bag
```

## 5. Acknowledgements
This algorithm is based on the following work, and we would like to express our gratitude their authors.
[FAST-LIO](https://github.com/hku-mars/FAST_LIO).
[S-FAST-LIO](https://github.com/zlwang7/S-FAST_LIO)
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
