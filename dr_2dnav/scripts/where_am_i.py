#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Robot:
   rospy.init_node('Cmd_publisher', anonymous=True)
   def __init__(self):
    f = open("path_cityhall_once_2.txt", 'r')
    lines = f.readlines()
    f.close()
    index =0
    self.green = False
    self.light_index = 0
    self.M = np.zeros((len(lines),2))
    for line in lines:
        value = line.split()
        self.M[index,0] = float(value[0])
        self.M[index,1] = float(value[1])
        index = index+1
    rospy.Subscriber("/odom", Odometry, self.index_callback)
    rospy.Subscriber("/cmd_vel_original", Twist, self.cmd_vel_callback)
    rospy.Subscriber("/green_light", Bool, self.Traffic_light_callback)
    rospy.spin()

   def Traffic_light_callback(self,data):
    pub_traffic = rospy.Publisher('/green', Bool, queue_size=1)
    if self.light_index>10:
       self.green = True
    self.light_index +=1
    pub_traffic.publish(self.green)    

   def index_callback(self,data):
    pub = rospy.Publisher('/index', Int32, queue_size=1)
    D = np.zeros((self.M.shape[0],1))
    for i in range(self.M.shape[0]):
        D[i] = np.sqrt((self.M[i,0]-data.pose.pose.position.x)**2+(self.M[i,1]-data.pose.pose.position.y)**2)
    self.ind = np.argmin(D)
    #print(self.ind)
    pub.publish(self.ind)

   def cmd_vel_callback(self,data):
    pub_cmd = rospy.Publisher('/move_base/cmd_vel', Twist, queue_size=10)
    scailing_indoor_factor =0.333
    scailing_down_factor = 0.5
    scailing_up_factor = 1.5
    if (self.ind >237) and (self.ind <2000):                  #from curve1 to crosswalk
        data.linear.x=data.linear.x*scailing_down_factor
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind>=2000) and (self.ind<2280) and (self.green==False):           #Traffic light Red
        data.linear.x=0
        data.angular.z=0
        pub_cmd.publish(data)
        print("Red light")
    elif (self.ind>2000) and (self.ind<2280) and (self.green==True):           #Traffic light Green
        pub_cmd.publish(data)
        print("Green light") 
    elif (self.ind >=2280) and (self.ind) <2460:              #crosswalk
        data.linear.x=0.8
        data.angular.z=data.angular.z*0.5
        pub_cmd.publish(data)
        print("Boost")
    elif (self.ind >=2480) and (self.ind) <2600:              #crosswalk stuck
        data.linear.x=data.linear.x*scailing_down_factor
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >=2600) and (self.ind) <3420:              #subway
        data.linear.x=0.6
        pub_cmd.publish(data)
        print("normal")
    elif (self.ind >3420) and (self.ind) <3550:              #curve 2
        data.linear.x=data.linear.x*scailing_down_factor
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >3550) and (self.ind) <3900:               #parking
        data.linear.x=0.6
        pub_cmd.publish(data)
        print("normal")
    elif (self.ind >=3900) and (self.ind) <4120:               #curve 3
        data.linear.x=0.3
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >=4120) and (self.ind) <4950:               #Hill
        data.linear.x=0.6
        pub_cmd.publish(data)
        print("normal")
    elif (self.ind >=4950) and (self.ind) <5735:                #in front of cityhall
        data.linear.x=data.linear.x*scailing_down_factor
        pub_cmd.publish(data)
        print("slower")
    elif (self.ind >=5735) and (self.ind) <6125:                #elevator
#        data.linear.x=data.linear.x*scailing_indoor_factor
        data.linear.x=0.15
        pub_cmd.publish(data)
        print("slower")
    else:
        pub_cmd.publish(data)
        print("normal")

if __name__ == '__main__':
    try:
	Robot()
    except rospy.ROSInterruptException:
        pass
