#!/usr/bin/env python
import rospy
import time
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf

import actionlib
from mbf_msgs.msg import ExePathAction
import mbf_msgs.msg 

def smach_client(path_msg):
    client = actionlib.SimpleActionClient('move_base_flex/exe_path', ExePathAction)
    client.wait_for_server()
    goal = mbf_msgs.msg.ExePathGoal(path=path_msg, controller="dwa")
    #goal = mbf_msgs.msg.ExePathGoal(path=path_msg)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('path_action_client', anonymous=True)
        pub = rospy.Publisher('/path', Path, queue_size=1)
        time.sleep(0.5)
        f = open("KI_1F_L_path.txt", 'r')
        lines = f.readlines()
        f.close()
        posestamp_list = []
        seq = 0

        header_msg = Header()
        header_msg.seq = seq
        header_msg.stamp = rospy.Time.now()
        header_msg.frame_id = "map"

        path_msg = Path()
        for line in lines:
            posestamp_msg = PoseStamped()
            pose_msg = Pose()
            value = line.split()
            pose_msg.position.x = float(value[0])
            pose_msg.position.y = float(value[1])
            pose_msg.position.z = float(value[2])
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

        pub.publish(path_msg)
        rospy.loginfo(path_msg)
        result = smach_client(path_msg)
    except rospy.ROSInterruptException:
        pass
