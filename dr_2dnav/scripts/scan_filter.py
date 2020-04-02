#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
class Robot:
   rospy.init_node('scan_filter', anonymous=True)
   def __init__(self):
    self.threshold = 4.0
    rospy.Subscriber("/scan_raw", LaserScan, self.scan_callback)
    rospy.spin()
   def scan_callback(self,data):
    scan_b = data
    scan_b.ranges = list(data.ranges)
    pub_scan = rospy.Publisher('/scan_filter', LaserScan, queue_size=10)
    for i in range(len(data.ranges)):
      if math.isinf(data.ranges[i])==True or data.ranges[i]>10:
         scan_b.ranges[i]=self.threshold
      else:
         scan_b.ranges[i]=data.ranges[i]
    scan_b.ranges = tuple(scan_b.ranges)
    print(scan_b.ranges)
    pub_scan.publish(scan_b)
if __name__ == '__main__':
    try:
	Robot()
    except rospy.ROSInterruptException:
        pass
