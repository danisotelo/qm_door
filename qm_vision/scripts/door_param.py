#!/usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from ultralytics import YOLO

@torch.no_grad()
class YoloDetector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~dconfidence_threshold")
        self.iou_thres = rospy.get_param("~diou_threshold")
        self.max_det = rospy.get_param("~dmaximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.view_image = rospy.get_param("~view_image")
        self.threshold_lower = rospy.get_param("~dthreshold_l", 30)
        self.threshold_upper = rospy.get_param("~dthreshold_u", 255)
        
        # Initialize weights
        weights = rospy.get_param("~dweights")
        
        # Initialize model
        self.model = YOLO(weights)

        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking=True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )
        
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        # Initialize pixel width publisher
        self.pixel_width_pub = rospy.Publisher(rospy.get_param("~output_width_topic"), Float32, queue_size=10)

        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data):
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        im_c = im.copy()

        # Perform inference
        results = self.model(im)

        # Print attributes of results for debugging
        rospy.loginfo(f"Results type: {type(results)}")
        for result in results:
            rospy.loginfo(f"Result attributes: {dir(result)}")
            if hasattr(result, 'boxes'):
                rospy.loginfo(f"Boxes: {result.boxes}")
            if hasattr(result, 'masks'):
                rospy.loginfo(f"Masks: {result.masks}")

        # Extract bounding box of the door
        door_box = None
        detected_classes = []
        for result in results:
            if hasattr(result, 'boxes') and result.boxes is not None:
                for box in result.boxes:
                    box_data = box.xyxy.cpu().numpy().squeeze()
                    label = int(box.cls)
                    detected_classes.append(result.names[label])
                    if result.names[label] == 'Door':
                        door_box = box_data
                        break
        
        # Print detected classes
        rospy.loginfo(f"Detected classes: {detected_classes}")

        if door_box is not None:
            x1, y1, x2, y2 = door_box[:4]
            width = x2 - x1
            height = y2 - y1
            door_width = max(width, height)

            rospy.loginfo(f"Door width in pixels: {door_width}")
            
            # Publish the pixel width
            self.pixel_width_pub.publish(door_width)

            # Draw the bounding box on the image
            cv2.rectangle(im_c, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            
            # Put the width text on the image
            cv2.putText(im_c, f'Width: {door_width:.2f} pixels', 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        else:
            rospy.loginfo("No door detected.")

        # Visualize and publish results if required
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_c, "bgr8"))

        if self.view_image:
            cv2.imshow("original", im)
            cv2.imshow("annotated", im_c)
            cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("masking", anonymous=True)
    detector = YoloDetector()
    rospy.spin()
