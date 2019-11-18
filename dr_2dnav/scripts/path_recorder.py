#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    rospy.loginfo(data.pose)
    f = open("path_1F.txt",'a')
    data = str(data.pose.pose.position.x) + ' ' \
        + str(data.pose.pose.position.y) + ' ' \
        + str(data.pose.pose.position.z) + ' ' \
        + str(data.pose.pose.orientation.x) + ' ' \
        + str(data.pose.pose.orientation.y) + ' ' \
        + str(data.pose.pose.orientation.z) + ' ' \
        + str(data.pose.pose.orientation.w) + '\n'
    f.write(data)
    f.close()
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback)
    #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
