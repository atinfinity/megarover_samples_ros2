# megarover_samples_ros2

## Introduction

This is a ROS2 Package to develop package of megarover using Gazebo.  
I used model, mesh and world files of <https://github.com/vstoneofficial/megarover_samples> as a reference.

## Requirements

- ROS2 Foxy

## Build

```
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/src
$ git clone https://github.com/atinfinity/megarover_samples_ros2.git
$ cd ..
$ colcon build --symlink-install
$ source ~/dev_ws/install/setup.bash
```

## Launch Gazebo

```
$ ros2 launch megarover_samples_ros2 vmegarover_with_sample_world.launch.py
```

## Run Teleop

```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Launch RViz

```
$ source ~/dev_ws/install/setup.bash
$ rviz2 -d ~/dev_ws/src/megarover_samples_ros2/rviz/mapping.rviz
```

## ToDo

- [x] 2D Scan
- [x] odometry
- [x] teleop
- [ ] slam_toolbox
- [ ] amcl
- [ ] navigation

