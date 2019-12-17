#!/usr/bin/env python
import json
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

class JsonWriter:
    def __init__(self):
        self.file_path = rospy.get_param('~json_path', 'sensor_data.json')
        rospy.loginfo(self.file_path)

        #self.index_sub = rospy.Subscriber('index', Int32, self.index_callback)
        self.imu_sub = rospy.Subscriber('/gx5/imu/data', Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber('gps', NavSatFix, self.gps_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.time_callback)

        self.LAST_INDEX = 1000
        self.current_status = 'IDLE'
        self.index_data = Int32()
        self.imu_data = Imu()
        self.gps_data = NavSatFix()
        self.odom_data = Odometry()

        self.json_data = {}

        self.make_json_data()

    def index_callback(self, index_data):
        self.index_data = index_data
        if self.index_data == 0:
            self.current_status = 'Idle'
        elif self.index_data >= self.LAST_INDEX:
            self.current_status = 'Arrived'
        else:
            self.current_status = 'Shipping'

    def imu_callback(self, imu_data):
        self.imu_data = imu_data

    def gps_callback(self, gps_data):
        self.gps_data = gps_data

    def odom_callback(self, odom_data):
        self.odom_data = odom_data
        if self.odom_data.twist.twist.linear.x < 0.1:
            self.current_status = 'Idle'
        else:
            self.current_status = 'Shipping'

    def time_callback(self, timer):
        self.make_json_data()
        self.write_file(self.file_path)

    def make_json_data(self):
        data = {}
        data['sensors'] = []
        data['sensors'].append({
            'current_status': self.current_status,

            'imu_acc_x': self.imu_data.linear_acceleration.x,
            'imu_acc_y': self.imu_data.linear_acceleration.y,
            'imu_acc_z': self.imu_data.linear_acceleration.z,
            'imu_gyro_x': self.imu_data.angular_velocity.x,
            'imu_gyro_y': self.imu_data.angular_velocity.y,
            'imu_gyro_z': self.imu_data.angular_velocity.z,
            'imu_ori_x': self.imu_data.orientation.x,
            'imu_ori_y': self.imu_data.orientation.y,
            'imu_ori_z': self.imu_data.orientation.z,
            'imu_ori_w': self.imu_data.orientation.w,
        
            'gps_lat': self.gps_data.latitude,
            'gps_lon': self.gps_data.longitude,
            'gps_hei': self.gps_data.altitude,
        
            #'left_wheel_speed': ,
            #'left_wheel_angle': ,
            #'right_wheel_speed': ,
            #'right_wheel_angle': ,
            
            #'odom_x': self.odom_data.pose.pose.position.x,
            #'odom_y': self.odom_data.pose.pose.position.y,
            'x_speed': self.odom_data.twist.twist.linear.x,
            'yaw_rate': self.odom_data.twist.twist.angular.z
        })
        self.json_data = data
        

    def write_file(self, file_name):
        with open(file_name, 'w') as outfile:
            json.dump(self.json_data, outfile)

if __name__ == '__main__':
    try:
        rospy.init_node('json_writer', anonymous=True)
        json_writer = JsonWriter()
        while not rospy.is_shutdown():
            rospy.logdebug(json_writer.json_data)
    except rospy.ROSInterruptException:
        pass
