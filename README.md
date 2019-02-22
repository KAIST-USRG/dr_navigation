# dr_2dnav
## Run with Cartographer
1. cd catkin_carto_ws && source install_isolated/setup.bash
2. sudo chmod 777 /dev/ttyACM0 /dev/ttyUSB0
3. roslaunch dr_2dnav dr_configurations.launch
4. roslaunch dr_2dnav move_base.launch
5. rosrun DR_Cont dr_control

## Using rosbag
1. rosbag play --clock <bag_file.bag> 
2. roslaunch dr_2dnav move_base_sim.launch

## Realtime
1. turn on velodyne_driver
2. turn on imu
3. run cartographer for localization
4. roslaunch dr_2dnav move_base.launch

## Cartographer Reference
- [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/index.html)
- [Localization only](https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html#localization-only)
- [Pure Localization in a Given Map](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)
- [Running Cartographer ROS on your own bag](https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html)
- [Github issue: Pure Localization Demo 2D #1122](https://github.com/googlecartographer/cartographer_ros/issues/1122)
- [Github issue: cannot switch to pure localization #1015](https://github.com/googlecartographer/cartographer_ros/issues/1015)
