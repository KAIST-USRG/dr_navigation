#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

class Traffic_light_predict:
   def __init__(self):
    rospy.Subscriber("/camera/color/image_raw", Image, self.ImageCallback)
    self.bridge= CvBridge()
    self.green_index=0
    self.LT = [175,362]
    self.RB = [228,622]
    self.threshold = 100
    self.common_app=[]
    self.common_app_1=[]
    self.green_light_flag = True
    rospy.spin()
  
   def ImageCallback(self,data):
     pub_greenLight = rospy.Publisher('/green_light',Bool,queue_size=1)
     cv_image     = self.bridge.imgmsg_to_cv2(data, "bgr8")
     cv_image_r     = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
     cv_image_crop = cv_image_r[self.LT[0]:self.RB[0],self.LT[1]:self.RB[1]]
     R,G,B    = cv2.split(cv_image_crop)
     R=np.ravel(R)
     G=np.ravel(G)
     B=np.ravel(B)
     ind_R = np.array(np.where(R>150))
     ind_G = np.array(np.where(G<100))
     ind_B = np.array(np.where(B<100))
     ind_R_1 = np.array(np.where(R<200))
     ind_G_1 = np.array(np.where(G==255))
     ind_B_1 = np.array(np.where(B==255))
     ind_R = ind_R.tolist()
     ind_G = ind_G.tolist()
     ind_B = ind_B.tolist()
     ind_R_1 = ind_R_1.tolist()
     ind_G_1 = ind_G_1.tolist()
     ind_B_1 = ind_B_1.tolist()
     common_element= list(set(ind_R[0]) & set(ind_G[0]) & set(ind_B[0]))
     common_element_1= list(set(ind_R_1[0]) & set(ind_G_1[0]) & set(ind_B_1[0]))
     self.common_app.append(len(common_element))
     self.common_app_1.append(len(common_element_1))
     if len(self.common_app)>=self.threshold:
      self.common_app = self.common_app[len(self.common_app)-(self.threshold):len(self.common_app)]
      self.common_app_1 = self.common_app_1[len(self.common_app_1)-(self.threshold):len(self.common_app_1)]
      common_array = np.array(self.common_app)
      common_array_1 = np.array(self.common_app_1)
      ind_zero = np.where(common_array==0)
      ind_zero_1 = np.where(common_array_1 !=0)
      print('red index=%d' %(len(ind_zero[0])))
      print('green_index=%d' %(len(ind_zero_1[0])))
      if ((len(ind_zero[0])>=self.threshold-10) and (len(ind_zero_1[0])>=self.threshold-10)):
        print('green_light')
        pub_greenLight.publish(self.green_light_flag)
     cv2.rectangle(cv_image, (self.LT[1],self.LT[0]), (self.RB[1],self.RB[0]), (0,255,0), 1)
     cv2.imshow('image',cv_image)
     cv2.waitKey(30)

if __name__ == '__main__':
    rospy.init_node('Traffic_light_predict', anonymous=True)
    try:
     Traffic_light_predict()
    except rospy.ROSInterruptException:
        pass
