# Multi-Robot Graph SLAM using LIDAR

This repository contains a ROS2 multi-robot 3D LIDAR SLAM implementation based on the [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) package. The package is tested on ROS2 humble.

## Dependencies

- OpenMP
- PCL
- g2o
- suitesparse

The following ROS packages are required:

- geodesy   
- nmea_msgs
- pcl_ros

## Installation

```
sudo apt install python3-vcstool
git clone https://github.com/aserbremen/Multi-Robot-Graph-SLAM
cd Multi-Robot-Graph-SLAM
mkdir src
vcs import src < multi_robot_graph_slam.repos
colcon build --symlink-install
```

## Usage

Check out the repository [mrg_sim](https://github.com/aserbremen/mrg_sim) for testing out the multi-robot SLAM implementation in a simulated environment using Gazebo (tested on Fortress)