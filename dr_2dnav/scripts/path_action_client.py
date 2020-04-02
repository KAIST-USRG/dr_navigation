#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import tf

import actionlib
from mbf_msgs.msg import ExePathAction
import mbf_msgs.msg

def change_callback(data):
    f = open("path_final.txt", 'r')
    f_= open("path.txt",'r')
    lines = f.readlines()
    lines_ = f_.readlines()
    f.close()
    f_.close()
    posestamp_list = []
    posestamp_list_ = []
    seq = 0
    seq_ = 0

    header_msg = Header()
    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = "map"
    header_msg_ = Header()
    header_msg_.seq = seq_
    header_msg_.stamp = rospy.Time.now()
    header_msg_.frame_id = "map"

    for line in lines:
        path_msg = Path()
        posestamp_msg = PoseStamped()
        pose_msg = Pose()
        value = line.split()
        pose_msg.position.x = float(value[0])
        pose_msg.position.y = float(value[1])
        pose_msg.position.z = float(0.0)
        quaternion = (
        float(value[3]),
        float(value[4]),
        float(value[5]),
        float(value[6]))
        euler = tf.transformations.euler_from_quaternion(quaternion)
        quaternion_b = tf.transformations.quaternion_from_euler(0, 0, euler[2])
        pose_msg.orientation.x = float(quaternion_b[0])
        pose_msg.orientation.y = float(quaternion_b[1])
        pose_msg.orientation.z = float(quaternion_b[2])
        pose_msg.orientation.w = float(quaternion_b[3])     
        posestamp_msg.pose = pose_msg
        posestamp_msg.header = header_msg
        posestamp_list.append(posestamp_msg)

    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = "map"

    path_msg.header = header_msg
    path_msg.poses = posestamp_list
    client = actionlib.SimpleActionClient('move_base_flex/exe_path', ExePathAction)
    print("in")
    client.cancel_all_goals()
    rospy.loginfo(path_msg)
    goal = mbf_msgs.msg.ExePathGoal(path = path_msg)
    client.send_goal(goal)
    print("previous goal is canceld and new goal is detected")
    client.wait_for_result()
    rospy.loginfo(client.get_result)
    print("End")

    
def smach_client(path_msg):
    rospy.loginfo(path_msg)
    client = actionlib.SimpleActionClient('move_base_flex/exe_path', ExePathAction)
    client.wait_for_server()
    goal = mbf_msgs.msg.ExePathGoal(path = path_msg)
    client.send_goal(goal)
    client.wait_for_result()
    print("end")
    return client.get_result()

def talker():
    f = open("path_final.txt", 'r')
    f_= open("path.txt",'r')
    lines = f.readlines()
    lines_ = f_.readlines()
    f.close()
    f_.close()
    posestamp_list = []
    posestamp_list_ = []
    seq = 0
    seq_ = 0

    header_msg = Header()
    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = "map"
    header_msg_ = Header()
    header_msg_.seq = seq_
    header_msg_.stamp = rospy.Time.now()
    header_msg_.frame_id = "map"

    for line in lines:
        path_msg = Path()
        posestamp_msg = PoseStamped()
        pose_msg = Pose()
        value = line.split()
        pose_msg.position.x = float(value[0])
        pose_msg.position.y = float(value[1])
        pose_msg.position.z = float(0.0)
        quaternion = (
        float(value[3]),
        float(value[4]),
        float(value[5]),
        float(value[6]))
        euler = tf.transformations.euler_from_quaternion(quaternion)
        quaternion_b = tf.transformations.quaternion_from_euler(0, 0, euler[2])
        pose_msg.orientation.x = float(quaternion_b[0])
        pose_msg.orientation.y = float(quaternion_b[1])
        pose_msg.orientation.z = float(quaternion_b[2])
        pose_msg.orientation.w = float(quaternion_b[3])     
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
       rospy.init_node('path_action_client',anonymous=True)
       rospy.Subscriber("/change", Bool, change_callback)
       path_msg = talker()
       result = smach_client(path_msg)
       rospy.loginfo(result)
       rospy.spin()
       
    except rospy.ROSInterruptException:
        pass
