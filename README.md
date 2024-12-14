# Wheel-Legged SLAM


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
The Package works in the Ubuntu 20.04 environment with the corresponding version of ROS.

### 1.2. **PCL && Eigen**
PCL >= 1.8, Eigen >= 3.3.4.

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
git clone 
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 3. Rosbag
Here we provide a rosbag in the corridor scene.


## 4. Run

roslaunch CloudMerging cloudmerg.launch
roslaunch wheellegged_slam run.launch


## 7. Acknowledgements
Thanks for the authors of [FAST-LIO](https://github.com/hku-mars/FAST_LIO).

Thanks for the authors of [S-FAST-LIO](https://api.star-history.com/svg?repos=zlwang7/S-FAST_LIO&type=Date)

Thanks for the authors of [LIO-SAM]
