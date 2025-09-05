# Ros2 gazebo for warehouse AGV/AMR

This repo aims to develop a simulation environment for warehouse AGV/AMR for study & research.

## Getting Started for pure gazebo environment

### Environment and dependencies
- Ubuntu 22.04 Jammy,
- ROS2 Humble,
- Gazebo Classic (EOL as 2025.1) (will switch to later versions)

### Installing Gazebo classic
Official website for installation: [Tutorial](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) <br />

* Setup package source 
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
* Update
```
sudo apt-get update
```
* Install
```
sudo apt-get install gazebo
sudo apt-get install libgazebo-dev
```
And also the ros2-gazebo package: [Tutorial](https://classic.gazebosim.org/tutorials?tut=ros2_installing)
```
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Build package and run

Build
```
cd ros_ws/
colcon build --symlink-install
```
Run example
```
ros2 launch my_agv2 agv5_demo.launch.py
```
Controller:
- navigation stack with map
- joy controller
```
ros2 run joy_controller joy_controller_node
```
- keyboard control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Lidar-Camera fusion
This part is modified based on existing repos:

https://github.com/AbdullahGM1/l2i_fusion_detection

https://github.com/mgonzs13/yolo_ros

Dependency:
```
pip install ultralytics
```
> [!NOTE]
> Some packages may require specific numpy version

## TODO
- [ ] Add local model path to source
- [ ] Switch to Issac for vision based training

## Help
Changye Ma (c7ma@uwaterloo.ca)
