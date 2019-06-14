# dr_navigation

## Prerequirements
### cartographer_ros
Download cartographer on cartographer workspace.
```
$ sudo apt-get update
$ sudo apt-get install -y python-wstool python-rosdep ninja-build
$ cd ~
$ mkdir -p cartographer_ws
$ cd cartographer_ws
$ wstool init src
$ wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
$ wstool update -t src
$ src/cartographer/scripts/install_proto3.sh
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
Checkout to master branch
```
$ cd ~/cartographer_ws/src/cartographer
$ git checkout master
$ cd ~/cartographer_ws/src/cartographer_ros
$ git checkout master
```
Compile
```
$ cd ~/cartographer_ws
$ catkin_make_isolated --install --use-ninja
```

## Build dr_navigation
Clone github repository.
```
$ cd ~
$ mkdir -p dr_ws/src
$ cd dr_ws/src
$ git clone https://github.com/hdh7485/dr_navigation.git
```
Build
```
$ cd ~/dr_ws
$ source devel/setup.bash
$ catkin_make
```

## Launch
```
$ sudo chmod 777 /dev/ttyACM0 /dev/ttyUSB0
$ roslaunch dr_2dnav dr_configuration.launch
$ roslaunch dr_2dnav move_base.launch
$ roslaunch dr_control dr_control.launch
```
