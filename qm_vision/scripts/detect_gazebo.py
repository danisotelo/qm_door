#!/usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
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

        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking=True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(input_image_topic, CompressedImage, self.callback, queue_size=1)
        else:
            self.image_sub = rospy.Subscriber(input_image_topic, Image, self.callback, queue_size=1)

        self.camera_info_sub = rospy.Subscriber(rospy.get_param("~camera_info_topic"), CameraInfo, self.camera_info_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(rospy.get_param("~input_depth_topic"), Image, self.depth_callback, queue_size=1)
        
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(rospy.get_param("~output_image_topic"), Image, queue_size=10)
            self.mask_pub = rospy.Publisher(rospy.get_param("~output_mask_topic"), Image, queue_size=10)
            self.filtered_depth_pub = rospy.Publisher(rospy.get_param("~output_masked_depth_nr_topic"), Image, queue_size=10)
        
        self.pc_pub = rospy.Publisher(rospy.get_param("~output_pc_topic"), PointCloud2, queue_size=10)

        # Initialize CV_Bridge
        self.bridge = CvBridge()

        self.current_mask = None
        self.current_depth = None

        # Camera intrinsic parameters
        self.camera_info_received = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def callback(self, data):
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        im_c = im.copy()

        results = self.model.predict(im, max_det=self.max_det)
        mask = results[0].masks.data.cpu().numpy().squeeze()
        mask = cv2.resize(mask, (im_c.shape[1], im_c.shape[0]))
        im0 = results[0].plot()

        self.current_mask = mask
        
        if self.publish_image:
            masked_rgb = np.zeros_like(im_c)
            masked_rgb[mask > 0.5] = im_c[mask > 0.5]

            mask_msg = self.bridge.cv2_to_imgmsg(mask, "32FC1")
            mask_msg.header.stamp = data.header.stamp  # Ensure timestamps match
            self.mask_pub.publish(mask_msg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(masked_rgb, "bgr8"))

        if self.view_image:
            cv2.imshow("original", im)
            cv2.imshow("mask", masked_rgb)
            cv2.waitKey(1)

        # If we have a depth image, process it
        if self.current_depth is not None:
            self.process_depth()

    def camera_info_callback(self, data):
        self.fx = data.K[0]
        self.fy = data.K[4]
        self.cx = data.K[2]
        self.cy = data.K[5]
        self.camera_info_received = True
        #rospy.loginfo(f"Camera info received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            self.current_depth = depth_image

            # If we have a mask, process the depth
            if self.current_mask is not None:
                self.process_depth()
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_depth(self):
        mask = self.current_mask
        depth_image = self.current_depth

        if mask is not None and depth_image is not None:
            #rospy.loginfo("Processing masked depth image.")
            # Apply the mask to the depth image
            masked_depth = np.where(mask > 0, depth_image, 0)

            if self.publish_image:
                masked_depth_msg = self.bridge.cv2_to_imgmsg(masked_depth, "32FC1")
                masked_depth_msg.header.stamp = rospy.Time.now()
                self.filtered_depth_pub.publish(masked_depth_msg)

            # Generate and publish the point cloud
            self.publish_point_cloud(masked_depth, depth_image.shape)

    def publish_point_cloud(self, masked_depth, depth_shape, downsample_factor = 8):
        points = []
        height, width = depth_shape

        for v in range(0, height, downsample_factor):
            for u in range(0, width, downsample_factor):
                depth_value = masked_depth[v, u]
                if depth_value > 0:
                    z = depth_value
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    points.append([x, y, z])

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_link_optical"  # Adjust this to your frame
        filtered_pc = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(filtered_pc)
        #rospy.loginfo(f"Published filtered point cloud with {len(points)} points.")

if __name__ == "__main__":
    rospy.init_node("handle_pc", anonymous=True)
    detector = YoloDetector()
    rospy.spin()