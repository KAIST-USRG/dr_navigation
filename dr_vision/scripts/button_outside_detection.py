#!/usr/bin/env python

import rospkg
import math
import rospy
import cv2
import os
import sys
import tensorflow as tf
import numpy as np


from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo, Range
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilites
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# Name of the directory containing the object detection module we're using
rospack = rospkg.RosPack()
MODEL_NAME = 'inference_graph_outside'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(rospack.get_path('dr_vision'), 'models', MODEL_NAME, 'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(rospack.get_path('dr_vision'), 'models', 'training_outside', 'labelmap.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 2

## Load the label map.
# Label maps map indices to category names, so that when our convolution
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()

config = tf.ConfigProto()
config.gpu_options.allow_growth = True

with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph, config=config)

# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')


def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]



class BboxDepth:
    def __init__(self):
        self.bridge = CvBridge()
        self.lidar_sub = rospy.Subscriber("/teraranger_evo", Range, self.lidarCallback)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback)
        self.bbox_up_pose_pub = rospy.Publisher("/bbox_pose", Pose, queue_size=10)
        self.bbox_up_conf_pub = rospy.Publisher("/bbox_up_conf", Float32, queue_size=10)
        self.P_mat = [615.4674072265625, 0.0, 319.95697021484375,
                      0.0, 0.0, 615.7725219726562,
                      245.20480346679688, 0.0, 0.0,
                      0.0, 1.0, 0.0]

        self.focal_x = 615.4674072265625
        self.focal_y = 615.7725219726562
        self.offset_x = 319.95697021484375
        self.offset_y = 245.20480346679688
        self.width = 640
        self.height = 480
        self.depth_valid = True
        self.bbox_up_pose_msg = Pose()
        self.bbox_up_conf_msg = Float32()

    def lidarCallback(self, msg):
        if math.isnan(msg.range):
            self.depth_valid = False
            return
        self.lidar_depth = msg.range
        self.depth_valid = True

    def transform_coordinate(self, np_array):
        '''
        :param np_array: array of normalized coordinate [y_min x_min y_max x_max]
        :return: temp: transform np_array to the coordinate in the image frame
                       [x_min y_min x_max y_max]
        '''
        temp = np.zeros(shape=np_array.shape)
        "Top left point"
        temp[:, 0] = np_array[:, 1] * self.width  # x_min
        temp[:, 1] = np_array[:, 0] * self.height  # y_min

        "Bottom right point"
        temp[:, 2] = np_array[:, 3] * self.width  # x_max
        temp[:, 3] = np_array[:, 2] * self.height  # y_max

        return temp

    def map_class(self, n):
        if n == 1:
            return 'up'
        elif n == 2:
            return 'down'
        else:
            return None
            


    def imageCallback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
            # i.e. a single-column array, where each item in the column has the pixel RGB value
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame_expanded = np.expand_dims(frame, axis=0)

            # Perform the actual detection by running the model with the image as input
            (boxes, scores, classes, num) = sess.run(
                [detection_boxes, detection_scores, detection_classes, num_detections],
                feed_dict={image_tensor: frame_expanded})

            index = np.squeeze(scores >= 0.90)

            boxes_high_conf = self.transform_coordinate(np.squeeze(boxes)[index])   # map normalized coordinate to the pixel coordinate
            classes_high_conf = np.squeeze(classes)[index]
            scores_high_conf = np.squeeze(scores)[index]
            class_up_idx = (classes_high_conf == 1) # find class up
            bbox_up = boxes_high_conf[class_up_idx]
            score_up = scores_high_conf[class_up_idx]

            if len(bbox_up) == 1:  # expect only "ONE" bounding box of class 10
                bbox_pos = self.estimate_position(bbox_up)
                bbox_conf = score_up
                print("bbox_pos", bbox_pos, bbox_conf)
                
                # PUBLISH bbox of class 10 [x_position, y_position, DEPTH]
                self.bbox_up_pose_msg.position.x = bbox_pos[0][0] 
                self.bbox_up_pose_msg.position.y = bbox_pos[0][1]
                self.bbox_up_pose_msg.position.z = bbox_pos[0][2]
                self.bbox_up_pose_pub.publish(self.bbox_up_pose_msg)

                # PUBLISH confidence of bbox of class 10
                self.bbox_up_conf_msg.data = bbox_conf
                self.bbox_up_conf_pub.publish(self.bbox_up_conf_msg)

            # DEBUG CODE (deleted if not needed)
            # # printing the detection result on the terminal
            # num_detection = len(bbox_conf)
            # if num_detection == 0:
            #     pass
            # elif num_detection == 1:
            #     print('class: ', self.map_class(bbox_class), ' score: ', bbox_conf[0], ' position: ', bbox_pos)
            #     print("---------------------------------------")
            # else:
            #     for i in range(len(bbox_conf)):
            #         print('class: ', self.map_class(bbox_class[i]), ' score: ', bbox_conf[i], ' position: ', bbox_pos[i])
            #     print("---------------------------------------")


            # Draw the results of the detection (aka 'visualize the results')
            vis_util.visualize_boxes_and_labels_on_image_array(
                frame,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=8,
                min_score_thresh=0.90)

            # All the results have been drawn on the frame, so it's time to display it.
            cv2.imshow('Object detector', frame)
            cv2.waitKey(30)

        except CvBridgeError as e:
            print(e)

    def estimate_position(self, array_bbox):
        bbox_pos = []
        for i in range(array_bbox.shape[0]):
            # bounding box
            boundingbox = array_bbox[i]
            x_LT = int(boundingbox[0])  # left top
            y_LT = int(boundingbox[1])
            x_RB = int(boundingbox[2])  # right bottom
            y_RB = int(boundingbox[3])

            if self.depth_valid == False:
                continue

            # Estimate x, y using depth(z)
            x_bb_center = (x_LT+x_RB)/2
            y_bb_center = (y_LT+y_RB)/2
            
            # Convert from 2d pixel coordi to 3d coordi
            # X = (x-offset_x)Z/fx, Y = (y-offset_y)Z/fy
            X = (x_bb_center-self.offset_x)*self.lidar_depth/self.focal_x # m scale
            Y = (y_bb_center-self.offset_y)*self.lidar_depth/self.focal_y # m scale

            bbox_pos.append([X, Y, self.lidar_depth])

        # DEBUG CODE (deleted if not needed)
        # print("depth", np.array(bbox_pos))
        return bbox_pos

if __name__ == '__main__':
    # code added for using ROS
    rospy.init_node('bbox_outside_lift')
    bboxdepth = BboxDepth()

    rospy.spin()
