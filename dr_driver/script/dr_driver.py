#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import odrive
from odrive.enums import *
import time
import math

class Odrive():
    def __init__(self):
        self.odrv0 = odrive.find_any()
        rospy.loginfo("Connected!!")
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.vel_gain = 0.06
        self.odrv0.axis0.controller.config.vel_integrator_gain = 0.08
        self.odrv0.axis1.controller.config.vel_gain = 0.06
        self.odrv0.axis1.controller.config.vel_integrator_gain = 0.08
        self.odom_pub = rospy.Publisher("odom_encoder", Odometry, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.twist_cb, queue_size=1)
        self.seq = 0

        self.ROBOT_WIDTH = 0.560
        self.WHEEL_RADIUS = 0.127
        self.GAIN = 6.92

    def twist_cb(self, cmd_vel):
    
        rospy.logdebug(self.odrv0.axis0.error)
        rospy.logdebug(self.odrv0.axis1.error)
    
        self.left_rpm = (cmd_vel.linear.x - cmd_vel.angular.z*self.ROBOT_WIDTH/2) / (self.WHEEL_RADIUS * 0.10472) * self.GAIN
        self.right_rpm = (cmd_vel.linear.x + cmd_vel.angular.z*self.ROBOT_WIDTH/2) / (self.WHEEL_RADIUS * 0.10472) * self.GAIN
    
        self.odrv0.axis0.controller.vel_setpoint = self.left_rpm
        self.odrv0.axis1.controller.vel_setpoint = self.right_rpm 
    
        rospy.logdebug(str(self.left_rpm) + ' '\
                      + str(self.right_rpm) + ' '\
                      + str(cmd_vel.linear.x) + ' '\
                      + str(cmd_vel.linear.y) + ' '\
                      + str(cmd_vel.angular.z))
    
    def timer_cb(self, event=None):
        self.odom_msg = Odometry()
        self.odom_msg.header.seq = self.seq
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.left_v = self.odrv0.axis0.encoder.vel_estimate / self.GAIN * self.WHEEL_RADIUS * 0.10472
        self.right_v = self.odrv0.axis1.encoder.vel_estimate / self.GAIN * self.WHEEL_RADIUS * 0.10472
        self.odom_msg.twist.twist.linear.x = (self.left_v + self.right_v)/2
        self.odom_pub.publish(self.odom_msg)

if __name__ == '__main__':
    rospy.init_node('diff_robot_control', anonymous=True)
    odrv = Odrive()
    timer = rospy.Timer(rospy.Duration(0.01), odrv.timer_cb)
    rospy.spin()
