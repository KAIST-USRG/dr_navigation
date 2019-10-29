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
$ git clone https://github.com/hdh7485/dr_navigation.git
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
