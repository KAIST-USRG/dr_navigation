#!/usr/bin/env python

import rospkg
import rospy
import cv2
import os
import sys
import tensorflow as tf
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilites
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


# Name of the directory containing the object detection module we're using
rospack = rospkg.RosPack()
MODEL_NAME = 'inference_graph_ped_light'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(rospack.get_path('dr_vision'), 'models', MODEL_NAME, 'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(rospack.get_path('dr_vision'), 'models', 'training_ped_light', 'labelmap.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 1

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


class light_detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.imageCallback)
        self.bbox_conf_pub = rospy.Publisher('bbox_conf', numpy_msg(Floats), queue_size=10)
        self.bbox_class_pub = rospy.Publisher('bbox_class', numpy_msg(Floats), queue_size=10)

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

            index = np.squeeze(scores >= 0.70)

            bbox_conf = np.squeeze(scores)[index]
            bbox_class = np.squeeze(classes)[index]

            self.bbox_conf_pub.publish(bbox_conf.flatten())
            self.bbox_class_pub.publish(bbox_class.flatten())

            # Draw the results of the detection (aka 'visualize the results')
            vis_util.visualize_boxes_and_labels_on_image_array(
                frame,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=8,
                min_score_thresh=0.70)

            # All the results have been drawn on the frame, so it's time to display it.
            cv2.imshow('ped_light detector', frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # code added for using ROS
    rospy.init_node('bbox_depth')
    detector = light_detector()

    rospy.spin()
