# IA-HeLiC
## IMU-assisted Target-free Extrinsic Calibration of Heterogeneous LIDARs Based on Continuous-time Optimization

This is the repo. of [IA-HeLiC](https://cslinzhang.github.io/IA-HeLiC), a calibration framework of heterogeneous LiDAR systems assisted by IMU. In this repo., we provided a ROS catkin package implementing IA-HeLiC.

## Prerequisites

This package runs on Ubuntu 20.04 (Focal Fossa) with [ROS Noetic](http://wiki.ros.org/noetic/Installation), and requires the following libraries:

| Library | Version |
|--|--|
| [Boost](https://www.boost.org) | 1.71.0 |
| [Ceres](http://ceres-solver.org) | 1.14.0 |
| [Eigen](https://eigen.tuxfamily.org) | 3.3 |
| [fmt](https://fmt.dev) | 10.0.0 |
| [glog](https://github.com/google/glog) | 0.4.0 |
| [Kontiki](https://github.com/hovren/kontiki) | (included) |
| [Livox ROS Driver](https://github.com/Livox-SDK/livox_ros_driver) | 2.6.0 |
| [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) | 2.2.0 |
| [OpenCV](https://opencv.org) | 4.2.0 |
| [OpenMP](https://www.openmp.org) | |
| [PCL](https://pointclouds.org) | 1.10.0 |
| [ndt_omp](https://github.com/koide3/ndt_omp) | commit 0852c95 |
| [Sophus](https://github.com/strasdat/Sophus) | 1.22.10 |

### Installing Libraries

1. Run the following command in the terminal:
    ```bash
    sudo apt-get install libboost-dev libceres-dev libceres1 libeigen3-dev libgoogle-glog-dev libopencv-dev libpcl-dev
    ```
1. Follow the installation guide of [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK/blob/master/README.md).

1. Download fmt, ndt_omp and Sophus, and install them:
    ```bash
    # install fmt
    git clone https://github.com/fmtlib/fmt.git
    cd fmt/
    mkdir build && cd build
    cmake .. && make && sudo make install

    # install ndt_omp
    git clone https://github.com/koide3/ndt_omp.git
    cd ndt_omp/
    mkdir build && cd build
    cmake .. && make && sudo make install

    # install Sophus
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus/
    mkdir build && cd build
    cmake .. && make && sudo make install
    ```

### Compile This Package

Put this package in a ROS workspace along with Livox ROS Driver and compile this package before running:
```bash
cd catkin_ws/
mkdir -p src/
cd src && catkin_init_workspace
git clone https://github.com/Livox-SDK/livox_ros_driver.git
git clone https://github.com/Yan-1999/IA-HeLiC.git
cd ..
catkin_make
```
## Run with Our Dataset

**IMPORTENT:** Due to the data size, please ensure that you have at least **48G** memory (32G pysical + 16G swap recommended) when running IA-HeLiC with these data sequences.

Download our dataset ([Baidu Netdisk link](https://pan.baidu.com/s/1BpMhSwwXUqyQ2VxEKwrOKg), password: pqvm), and then **edit the LiDAR trajectoy paths and output file path in `config/calib_node.yaml`** accordingly. It's highly recommended to decompress the bags first for faster reading speed:

```bash
# subsitute with your bag path
rosbag decompress /path/to/your/bag/seqX.bag
```
Then run the following command to calibrate (don't forget to source the `setup.bash` first):

```bash
# subsitute with your bag path
roslaunch ia_helic calib_node.launch bag_path:=/path/to/your/bag/seqX.bag
```
Then you will see the calibration file in the path you given.

## Run with Your Data

Preprocess the sensor data with LiDAR odometries is required to obtain LiDAR trajectoies. We've tested our IA-HeLiC with the following odometries:

- [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) for mechanical spinning LiDAR;
- [LI-Init](https://github.com/hku-mars/LiDAR_IMU_Init) for solid-state LiDAR.

Please provide IA-HeLiC with the LiDAR odometries in [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats) format:
```text
timestamp x y z q_x q_y q_z q_w
```
and the ROS bag with sensor data. Extrinsics between IMU and LiDAR are not required by IA-HeLiC.

Then edit the `lidar_types`, `lidar_odoms`, `topics` and `imu_acc_scale_by` items in `config/calib_node.yaml` accordingly.
And now you are ready for calibration:

```bash
# subsitute with your bag path
roslaunch ia_helic calib_node.launch bag_path:=/path/to/your/bag/seqX.bag
```

## Acknowledgements

This implementation is based on the work of [LVI-ExC](https://github.com/peterWon/LVI-ExC) and [LI-Calib](https://github.com/APRIL-ZJU/lidar_IMU_calib).

