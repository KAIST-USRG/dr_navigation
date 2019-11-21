# dr_navigation

## Prerequirements
1. SMACH
2. omoros
3. move_base_flex

## Build dr_navigation
Clone github repository.
```
$ cd ~
$ mkdir -p dr_ws/src
$ cd dr_ws/src
$ git clone https://github.com/kaist-usrg/dr_navigation.git
$ git clone https://github.com/hdh7485/teleop_twist_joy.git
```
Build
```
$ cd ~/dr_ws
$ source devel/setup.bash
$ catkin_make
```

## Launch dr_navigation example
```
$ sudo chmod 777 /dev/ttyACM0 /dev/ttyUSB0
$ roslaunch dr_2dnav dr_configuration.launch
$ roslaunch dr_2dnav move_base.launch
$ roslaunch dr_control dr_control.launch
```
## dr_vision
Installation
1) Install Tensorflow "python2" with compatible version with CUDA, cuDNN see link https://www.tensorflow.org/install/source#gpu
2) Install Realsense camera SDK package and Realsense ROS package
3) Install install object_detection/research in tensorflow model. See link https://github.com/tensorflow/models/research/object_detection/g3doc/installation.md

Run
1) Call the node of realsense camera
```
$ roslaunch realsense2_camera rs_rgbd.launch
```
2) Run the pedestrain light detector (if not run on GPU, it will be very very slow!)
```
$ rosrun dr_vision pred_light_detection.py
```
