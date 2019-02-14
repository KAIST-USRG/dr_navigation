# dr_2dnav
## Using rosbag
1. rosbag play --clock <bag_file.bag> 
2. roslaunch dr_2dnav move_base_sim.launch
## Realtime
1. turn on velodyne_driver
2. turn on imu
3. run cartographer for localization
4. roslaunch dr_2dnav move_base.launch
