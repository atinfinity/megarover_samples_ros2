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

## Mapping
### Launch Gazebo

```
$ source ~/dev_ws/install/setup.bash
$ ros2 launch megarover_samples_ros2 vmegarover_with_sample_world.launch.py
```

### Launch Slam Toolbox for Mapping

```
$ source ~/dev_ws/install/setup.bash
$ ros2 launch megarover_samples_ros2 vmegarover_mapping.launch.py
```

### Launch Teleop

```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Save Map

```
$ mkdir ~/maps
$ ros2 launch megarover_samples_ros2 vmegarover_save_map.launch.py
```

## ToDo

- [x] 2D Scan
- [x] odometry
- [x] teleop
- [x] slam_toolbox
- [ ] amcl
- [ ] navigation

