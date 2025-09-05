# Ros2 gazebo for warehouse AGV/AMR

This repo aims to develop a simulation environment for warehouse AGV/AMR for study & research.

## Getting Started

### Environment and dependencies
- Ubuntu 22.04 Jammy,
- ROS2 Humble,
- Gazebo Classic (EOL as 2025.1)

```
pip install ultralytics
```

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

* Build
```
cd ros_ws/
colcon build --symlink-install
```
* Run example
```
ros2 launch my_agv2 agv5_demo.launch.py
```
## TODO
- [ ] Add local model path to source
- [ ] Switch to Issac for vision based training
- [ ] 
- [ ] 

## Help
Changye Ma (c7ma@uwaterloo.ca)
