#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf

def talker():
    pub = rospy.Publisher('/path', Path, queue_size=10)
    rospy.init_node('path_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    f = open("path_1F.txt", 'r')
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
        #rospy.loginfo(value)
        pose_msg.position.x = float(value[0])
        pose_msg.position.y = float(value[1])
        pose_msg.position.z = float(value[2])
        #pose_msg.orientation.x = float(value[3])
        #pose_msg.orientation.y = float(value[4])
        #pose_msg.orientation.z = float(value[5])
        #pose_msg.orientation.w = float(value[6])
        quaternion_original=(float(value[3]),
                             float(value[4]),
                             float(value[5]),
                             float(value[6]))
        euler = tf.transformations.euler_from_quaternion(quaternion_original)
        roll = 0
        pitch = 0
        yaw   = euler[2]
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_msg.orientation.x = float(quaternion[0])
        pose_msg.orientation.y = float(quaternion[1])
        pose_msg.orientation.z = float(quaternion[2])
        pose_msg.orientation.w = float(quaternion[3])        
        posestamp_msg.pose = pose_msg
        posestamp_msg.header = header_msg
        posestamp_list.append(posestamp_msg)
    rospy.loginfo(posestamp_list)

    while not rospy.is_shutdown():
        header_msg.seq = seq
        header_msg.stamp = rospy.Time.now()
        header_msg.frame_id = "map"

        path_msg.header = header_msg
        path_msg.poses = posestamp_list

        pub.publish(path_msg)
        seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
