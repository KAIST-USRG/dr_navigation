import json
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class JsonWriter:
    def __init__(self):
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber('gps', NavSatFix, self.gps_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.time_callback)

        self.imu_data = Imu()
        self.gps_data = NavSatFix()
        self.odom_data = Odometry()

        self.json_data = {}

    def imu_callback(self, imu_data):
        self.imu_data = imu_data

    def gps_callback(self, gps_data):
        self.gps_data = gps_data

    def odom_callback(self, odom_data):
        self.odom_data = odom_data

    def time_callback(self, timer):
        rospy.logdebug('time_callback')
        self.make_json_data()

    def make_json_data(self):
        data = {}
        data['sensors'] = []
        data['sensors'].append({
            'imu_acc_x': 'Scott',
            'imu_acc_y': 'Scott',
            'imu_acc_z': 'Scott',
            'imu_gyro_x': 'Scott',
            'imu_gyro_y': 'Scott',
            'imu_gyro_z': 'Scott',
            'imu_ori_x': 'Scott',
            'imu_ori_y': 'Scott',
            'imu_ori_z': 'Scott',
            'imu_ori_w': 'Scott',
        
            'gps_lat': 'Scott',
            'gps_lon': 'Scott',
            'gps_hei': 'Scott',
        
            #'left_wheel_speed': ,
            #'left_wheel_angle': ,
            #'right_wheel_speed': ,
            #'right_wheel_angle': ,
            
            'odom_x': 1,
            'odom_y': 2,
            'odom_speed': 3
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
            json_writer.write_file("sensor_data.json")
            rospy.logdebug(json_writer.json_data)
    except rospy.ROSInterruptException:
        pass
