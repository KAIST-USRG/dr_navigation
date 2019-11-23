# dr_comm
## JSON data information (중요도 순)
### 1. GPS
GPS는 Latitude(위도), Longitude(경도), Height(높이) 정보를 얻어온다. 각각 gps_lat, gps_lon, gps_hei 키에 저장된다.
### 2. IMU
9축 IMU를 사용하며 IMU를 사용한다. 회전 속도, 선가속도, 방향이 각각 3축을 가지고 있다. imu_gyro_x, y, z는 IMU가 측정한 회전 속도, imu_acc_x, y, z는 선가속도, imu_ori_x, y, z는 방향을 의미한다.
### 3. Odometry
odom_x_speed 는 로봇의 속도이며, odom_yaw_rate는 로봇의 z축 회전 속도를 의미한다. odom_x와 odom_y는 3D pointcloud 위에서 로봇의 위치를 의미한다.
