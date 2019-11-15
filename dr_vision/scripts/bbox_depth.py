#!/usr/bin/env python

import rospy
import cv2
import os
import sys
import tensorflow as tf
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilites
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


# Name of the directory containing the object detection module we're using
MODEL_NAME = 'inference_graph'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, 'training', 'labelmap.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 7

## Load the label map.
# Label maps map indices to category names, so that when our convolution
# network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)

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
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,
                                          self.depthCallback, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.imageCallback)

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

            boxes_toPrint = self.transform_coordinate(np.squeeze(boxes)[index])

            # if(len(boxes_toPrint) >= 1):
            #     print(boxes_toPrint, boxes_toPrint.shape)
            #     print("-----------------------------")

            for i in range(boxes_toPrint.shape[0]):
                self.estimate_position(boxes_toPrint[i])

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
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def depthCallback(self, data):

        try:
            depth_image_raw = self.bridge.imgmsg_to_cv2(data, "16UC1")
            # smooth filtering
            self.depth_image = cv2.blur(depth_image_raw, (5, 5))
            # self.estimate_position()
        except CvBridgeError as e:
            print(e)

    def estimate_position(self, boundingbox):
        print(boundingbox)
        # bounding box
        x_LT = int(boundingbox[0]) # left top
        y_LT = int(boundingbox[1])
        x_RB = int(boundingbox[2]) # right bottom
        y_RB = int(boundingbox[3])

        # Extract bounding box region
        bbox_depth_arr = self.depth_image[x_LT:x_RB, y_LT:y_RB]

        # nan value -> 0
        # print(bbox_depth_arr[~np.isnan(bbox_depth_arr)])
        np.nan_to_num(bbox_depth_arr, copy=False) # nan to 0

        if bbox_depth_arr[np.nonzero(bbox_depth_arr)].size != 0:
            min_depth = np.min(bbox_depth_arr[np.nonzero(bbox_depth_arr)])
        else:
            min_depth = 0

        # Estimate x, y using depth(z)
        x_bb_center = (x_LT+x_RB)/2
        y_bb_center = (y_LT+y_RB)/2

        # Convert from 2d pixel coordi to 3d coordi

        X = (x_bb_center-self.offset_x)*min_depth/self.focal_x  # mm scale
        Y = (y_bb_center-self.offset_y)*min_depth/self.focal_y  # mm scale

        # Convert to m scale
        # X /= 1000.
        # Y /= 1000.
        # min_depth /= 1000.

        print("depth", X, Y, min_depth)
        print("------------------------------")


if __name__ == '__main__':
    # code added for using ROS
    rospy.init_node('bbox_depth')
    bboxdepth = BboxDepth()

    rospy.spin()
