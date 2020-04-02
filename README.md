# dr_navigation

## Launch in cityhall 20200402
1. roslaunch dr_2dnav dr_configurations.launch 
2. roslaunch dr_2dnav mbf_predefined_path.launch 
3. rosrun dr_2dnav where_am_i_b.py 
4. rosrun dr_2dnav path_action_client.py
5. rosrun dr_vision pred_light_detection_b.py 

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

For lift button detection
1) Call the node of realsense camera
```
$ roslaunch realsense2_camera rs_rgbd.launch
```
2) Run lidar sensor (don't forget to check the port name and give permission)
```
rosrun teraranger evo _portname:=/dev/ttyACM0 _sensor_type:=Evo_3m
```
check the publish topic by
```
rostopic echo /teraranger_evo
```
3) Run the button detector
```
$ rosrun dr_vision button_inside_detection.py
```

UR5 arm
1) Ping the IP of the robot arm to check the connection
```
$ ping 192.168.1.102
```
2) Run these commands
```
$ roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.102 [reverse_port:=REVERSE_PORT]
$ roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
$ roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```
3) Turn on the controller node (need to install moveit_tutorials package, and the code is in our slack)
```
$ roslaunch moveit_tutorials move_group_interface_tutorial.launch
```
