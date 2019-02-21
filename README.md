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
