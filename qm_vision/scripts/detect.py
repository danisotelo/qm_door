#!/usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage
from ultralytics import YOLO

@torch.no_grad()
class YoloDetector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.view_image = rospy.get_param("~view_image")
        self.threshold_lower = rospy.get_param("~threshold_l", 30)
        self.threshold_upper = rospy.get_param("~threshold_u", 255)
        
        # Initialize weights
        weights = rospy.get_param("~weights")
        
        # Initialize model
        self.model = YOLO(weights)

        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue = 1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size = 1
            )
        
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size = 10
            )
            self.mask_pub = rospy.Publisher(
                rospy.get_param("~output_mask_topic"), Image, queue_size = 10
            )

        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data):
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        im_c = im.copy()

        results = self.model.predict(im, max_det = 1)
        mask = results[0].masks.data.cpu().numpy().squeeze()
        mask = cv2.resize(mask, (im_c.shape[1], im_c.shape[0]))
        im0 = results[0].plot()
        
        if self.publish_image:
            masked_rgb = np.zeros_like(im_c)
            masked_rgb[mask > 0.5] = im_c[mask > 0.5]

            mask_msg = self.bridge.cv2_to_imgmsg(mask, "32FC1")
            mask_msg.header.stamp = data.header.stamp # Ensure timestamps match
            self.mask_pub.publish(mask_msg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(masked_rgb, "bgr8"))

        if self.view_image:
            cv2.imshow("original", im)
            cv2.imshow("mask", masked_rgb)
            cv2.waitKey(1)

if __name__ == "__main__":
    
    rospy.init_node("masking", anonymous = True)
    detector = YoloDetector()
    rospy.spin()
