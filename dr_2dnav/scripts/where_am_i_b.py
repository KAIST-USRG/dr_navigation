#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class Robot:
   rospy.init_node('Cmd_publisher', anonymous=True)
   def __init__(self):
    f = open("path_final.txt", 'r')
    lines = f.readlines()
    f.close()
    index =0
    self.change=False
    self.moveUp_1_flag_infront_count = 0
    self.moveUp_2_flag_infront_count = 0
    self.moveUp_3_flag_infront_count = 0
    self.moveUp_4_flag_infront_count = 0
    self.moveToDefault_flag_infront=0
    self.moveUp_1_flag = False
    self.moveUp_2_flag = False
    self.moveUp_3_flag = False
    self.moveUp_4_flag = False
    self.go_1 = False
    self.go_2 = False
    self.green = False
    self.light_index = 0
    self.angular_velocity=0
    self.M = np.zeros((len(lines),2))
    for line in lines:
        value = line.split()
        self.M[index,0] = float(value[0])
        self.M[index,1] = float(value[1])
        index = index+1
    rospy.Subscriber("/odom", Odometry, self.index_callback)
    rospy.Subscriber("/cmd_vel_original", Twist, self.cmd_vel_callback)
    rospy.Subscriber("/green_light", Bool, self.Traffic_light_callback)
    rospy.Subscriber("/gx5/imu/data", Imu, self.imu_callback)
    rospy.Subscriber("/moveUp_1_finish", Bool, self.moveUp_1_finish_callback)
    rospy.Subscriber("/moveUp_2_finish", Bool, self.moveUp_2_finish_callback)
    rospy.Subscriber("/moveUp_3_finish", Bool, self.moveUp_3_finish_callback)
    rospy.Subscriber("/moveUp_4_finish", Bool, self.moveUp_4_finish_callback)
#    rospy.Subscriber("/dist_back", Float32, self.dist_back_callback)
#    rospy.Subscriber("/dist_front", Float32, self.dist_front_callback)
    rospy.spin()
   def moveUp_1_finish_callback(self,data):
    self.moveUp_1_flag = True
   def moveUp_2_finish_callback(self,data):
    self.moveUp_2_flag = True
   def moveUp_3_finish_callback(self,data):
    self.moveUp_3_flag = True
   def moveUp_4_finish_callback(self,data):
    self.moveUp_4_flag = True
   def Traffic_light_callback(self,data):
    pub_traffic = rospy.Publisher('/green', Bool, queue_size=1)
    if (self.ind>=900) and (self.ind<1530):
#       print('green_light')
       if (self.light_index>=1):
          self.green = True
       self.light_index +=1
       pub_traffic.publish(self.green)
       print(self.light_index) 
   def imu_callback(self,data):
    self.angular_velocity=-data.angular_velocity.z
#    print(self.angular_velocity)
   def index_callback(self,data):
    pub = rospy.Publisher('/index', Int32, queue_size=1)
    D = np.zeros((self.M.shape[0],1))
    for i in range(self.M.shape[0]):
        D[i] = np.sqrt((self.M[i,0]-data.pose.pose.position.x)**2+(self.M[i,1]-data.pose.pose.position.y)**2)
    self.ind = np.argmin(D)
    pub.publish(self.ind)

   def cmd_vel_callback(self,data):
    pub_cmd = rospy.Publisher('/move_base/cmd_vel', Twist, queue_size=10)
    pub_moveUp_1 = rospy.Publisher('/moveUp_flag_1', Bool, queue_size=1)
    pub_moveUp_2 = rospy.Publisher('/moveUp_flag_2', Bool, queue_size=1)
    pub_moveUp_3 = rospy.Publisher('/moveUp_flag_3', Bool, queue_size=1)
    pub_moveUp_4 = rospy.Publisher('/moveUp_flag_4', Bool, queue_size=1)
    pub_change = rospy.Publisher('/change_flag', Bool, queue_size=1)
    scailing_indoor_factor =0.333
    scailing_down_factor = 0.5
    scailing_up_factor = 1.5
#    data.linear.x=0.6
    if (self.ind >120) and (self.ind <338):                  #from curve1 to crosswalk
        data.linear.x=0.2
        data.angular.z=data.angular.z*0.5
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >=560) and (self.ind<830):
        data.linear.x=0.2
        data.angular.z=data.angular.z
        pub_cmd.publish(data)
    elif (self.ind>=830) and (self.ind<1500) and (self.green==False):           #Traffic light Red
        data.linear.x=0
        data.angular.z=0
        pub_cmd.publish(data)
#        print("Red light")
    elif (self.ind>830) and (self.ind<1500) and (self.green==True):           #Traffic light Green
        data.linear.x=data.linear.x
        data.angular.z=data.angular.z
        pub_cmd.publish(data)
        print("Green light") 
    elif (self.ind >=1570) and (self.ind) <1707:              #crosswalk
        data.linear.x=0.8
        data.angular.z=data.angular.z*2
        pub_cmd.publish(data)
        print("Boost")
    elif (self.ind >=1707) and (self.ind) <1920:              #crosswalk stuck
        data.linear.x=data.linear.x*scailing_down_factor
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >=1920) and (self.ind) <2680:              #subway
        data.linear.x=0.6
        data.angular.z=data.angular.z*2
        pub_cmd.publish(data)
        print("normal")
    elif (self.ind >2680) and (self.ind) <2746:              #curve 2
        data.linear.x=0.2
        data.angular.z=data.angular.z
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >2746) and (self.ind) <3080:               #parking
        data.linear.x=0.6
        data.angular.z=data.angular.z*2
        pub_cmd.publish(data)
        print("normal")
    elif (self.ind >=3080) and (self.ind) <3235:               #curve 3
        data.linear.x=0.3
        data.angular.z=data.angular.z*0.5
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >=3235) and (self.ind) <3920:               #Hill
        data.linear.x=0.6
        data.angular.z=data.angular.z*2
        pub_cmd.publish(data)
        print("normal")
    elif (self.ind >=3920) and (self.ind) <4491:                #in front of cityhall
        data.linear.x=data.linear.x*scailing_down_factor
        data.angular.z=data.angular.z
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >=4491) and (self.ind) <5083:                #elevator
#        data.linear.x=data.linear.x*scailing_indoor_factor
        data.linear.x=0.15
        data.angular.z=data.angular.z	
        pub_cmd.publish(data)
        print("slower")
#    elif self.ind >= 4684 and self.ind<4691 and self.moveUp_1_flag == False and self.moveUp_2_flag == False:
#        data.linear.x = 0
#        data.linear.z = 0
#        pub_cmd.publish(data)
#        if self.moveUp_1_flag_infront_count ==0:
#          pub_moveUp_1.publish(True)
#          self.moveUp_1_flag_infront_count +=1
#    elif self.ind >= 4684 and self.ind<4691 and self.moveUp_1_flag == True and self.moveUp_2_flag == False:
#        data.linear.x = 0
#        data.linear.z = 0
#        pub_cmd.publish(data)
#        if self.moveUp_2_flag_infront_count ==0:
#          pub_moveUp_2.publish(True)
#          self.moveUp_2_flag_infront_count +=1
#    elif self.ind >= 4684 and self.ind<5083 and self.moveUp_1_flag == True and self.moveUp_2_flag == True:
#        data.linear.x=0.1
#        pub_cmd.publish(data)
#        print("slower")
#    elif sself.ind >= 4684 and self.ind<4691 and self.moveUp_3_flag == False and self.moveUp_4_flag == False:
#        data.linear.x = 0
#        data.linear.z = 0
#        pub_cmd.publish(data)
#        if self.moveUp_3_flag_infront_count ==0:
#          pub_moveUp_3.publish(True)
#          self.moveUp_3_flag_infront_count +=1
#    elif self.ind == 4703 and self.moveUp_3_flag == True and self.moveUp_2_flag == False:
#        data.linear.x = 0
#        data.linear.z = 0
#        pub_cmd.publish(data)
#        if self.moveUp_4_flag_infront_count ==0:
#         pub_moveUp_4.publish(True)
#          self.moveUp_2_flag_infront_count +=1
#    elif self.ind == 4703 and self.moveUp_3_flag == True and self.moveUp_4_flag == True:#
#        data.linear.x=0.15
#       data.angular.z=data.angular.z	
#       pub_cmd.publish(data)
#        print("slower")
    else:
#        data.linear.x=data.linear.x
#        data.angular.z=data.angular.z*2
	data.linear.x = 0
        data.angular.z=0
        pub_cmd.publish(data)
        print("normal")

if __name__ == '__main__':
    try:
	Robot()
    except rospy.ROSInterruptException:
        pass
