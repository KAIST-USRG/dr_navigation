#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf

import actionlib
from mbf_msgs.msg import ExePathAction
import mbf_msgs.msg 

def smach_client(path_msg):
    rospy.loginfo(path_msg)
    client = actionlib.SimpleActionClient('move_base_flex/exe_path', ExePathAction)
    client.wait_for_server()
    goal = mbf_msgs.msg.ExePathGoal(path = path_msg)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def talker():
    f = open("path.txt", 'r')
    lines = f.readlines()
    f.close()
    posestamp_list = []
    seq = 0

    header_msg = Header()
    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = "map"

    for line in lines:
        path_msg = Path()
        posestamp_msg = PoseStamped()
        pose_msg = Pose()
        value = line.split()
        pose_msg.position.x = float(value[0])
        pose_msg.position.y = float(value[1])
        pose_msg.position.z = 0
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = float(value[5])
        pose_msg.orientation.w = float(value[6])
        posestamp_msg.pose = pose_msg
        posestamp_msg.header = header_msg
        posestamp_list.append(posestamp_msg)

    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = "map"

    path_msg.header = header_msg
    path_msg.poses = posestamp_list

    return path_msg

if __name__ == '__main__':
    try:
        rospy.init_node('path_action_client', anonymous=True)
        path_msg = talker()
        result = smach_client(path_msg)
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        pass
