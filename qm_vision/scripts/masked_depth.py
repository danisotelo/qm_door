#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import copy
from sensor_msgs.msg import Image

class ClusterDetector():
    def __init__(self): 
        self.image_data = None
        self.depth_data = None
        self.bridge = CvBridge()
        self.view_image = rospy.get_param("~view_image", False)

        # Get the masked image
        self.image_sub = rospy.Subscriber(
            rospy.get_param("~output_mask_topic"), Image, self.image_callback, queue_size=1
        )

        # Get the raw depth image
        self.depth_sub = rospy.Subscriber(
            rospy.get_param("~input_depth_topic"), Image, self.depth_callback, queue_size=1
        )

        self.depth_pub = rospy.Publisher(
            rospy.get_param("~output_masked_depth_nr_topic"), Image, queue_size=1
        )


class ClusterDetector():
    def __init__(self): 
        self.image_data = None
        self.depth_data = None
        self.bridge = CvBridge()
        self.view_image = rospy.get_param("~view_image", False)

        # Get the masked image
        self.image_sub = rospy.Subscriber(
            rospy.get_param("~output_mask_topic"), Image, self.image_callback, queue_size=1
        )

        # Get the raw depth image
        self.depth_sub = rospy.Subscriber(
            rospy.get_param("~input_depth_topic"), Image, self.depth_callback, queue_size=1
        )

        self.depth_pub = rospy.Publisher(
            rospy.get_param("~output_masked_depth_nr_topic"), Image, queue_size=1
        )

    def image_callback(self, data: Image):
        try:
            # Check the encoding of the incoming image
            if data.encoding == "32FC1":
                base = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                # Convert to 8-bit single channel image
                self.image_data = cv2.normalize(base, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            else:
                base = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
                self.image_data = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def depth_callback(self, data: Image):
        self.depth_data = data

        send = Image()
        send = copy.deepcopy(self.depth_data)
        
        if self.image_data is not None:
            try:
                # Convert the depth image to a cv::Mat
                depth_mat = self.bridge.imgmsg_to_cv2(self.depth_data, desired_encoding="passthrough")
                cv_depth = np.array(depth_mat, dtype=np.uint16)

                # Ensure the mask is of the correct type and size
                if self.image_data.shape != cv_depth.shape:
                    self.image_data = cv2.resize(self.image_data, (cv_depth.shape[1], cv_depth.shape[0]), interpolation=cv2.INTER_NEAREST)

                # Apply masking to the depth image
                res = cv2.bitwise_and(cv_depth, cv_depth, mask=self.image_data)
                
                # Visualize images
                if self.view_image:
                    cv2.imshow("depth", res)
                    cv2.imshow("color_mask", self.image_data)
                    cv2.waitKey(1)

                send.data = self.bridge.cv2_to_imgmsg(res, encoding="16UC1").data
                self.depth_pub.publish(send)
            except CvBridgeError as e:
                rospy.logerr(f"Error processing depth image: {e}")

if __name__ == "__main__":
    rospy.init_node("masked_depth_nr", anonymous=True)
    detector = ClusterDetector()
    rospy.spin()