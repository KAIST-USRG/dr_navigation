#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf

class Robot:
   rospy.init_node('Cmd_publisher', anonymous=True)
   def __init__(self):
    self.M = 0
    rospy.Subscriber("/cmd_vel_original", Twist, self.cmd_vel_callback)
    rospy.spin()
   def cmd_vel_callback(self,data):
    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    data.linear.x= data.linear.x*0.5
    data.angular.z=data.angular.z*5
    pub_cmd.publish(data)
if __name__ == '__main__':
    try:
	Robot()
    except rospy.ROSInterruptException:
        pass
