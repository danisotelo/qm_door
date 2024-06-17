#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from qm_vision.srv import ComputeRadius, ComputeRadiusResponse

class DoorRadiusEstimator:
    def __init__(self):
        self.bridge = CvBridge()
        self.centroid = None
        self.axis_of_rotation = None
        self.camera_matrix = None
        self.distance_to_door = None

        # Subscribers 
        self.handle_mask_pub = rospy.Subscriber("/handle/mask", Image, self.handle_mask_callback)
        self.rgb_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camera_info_callback)
        self.distance_sub = rospy.Subscriber("/door/distance", Float32, self.distance_callback)

        # Publisher
        self.radius_service = rospy.Service("/compute_radius", ComputeRadius, self.handle_compute_radius)
        self.edge_image_pub = rospy.Publisher("/door/edge_detection", Image, queue_size=10)
        self.annotated_image_pub = rospy.Publisher("/edge_detection/annotated", Image, queue_size=10)

    def handle_mask_callback(self, msg):
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            # Threshold the mask to get binary image
            _, binary_mask = cv2.threshold(mask, 0.5, 1, cv2.THRESH_BINARY)
            # Find contours
            contours, _ = cv2.findContours(binary_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Compute the centroid of the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    self.centroid = (cX, cY)
                    rospy.loginfo(f"Centroid: {self.centroid}")
                else:
                    rospy.logwarn("Moment calculation resulted in division by zero.")
            else:
                rospy.logwarn("No contours found in the handle mask.")
        except CvBridgeError as e:
            rospy.logerr(e)

    def rgb_image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo("RGB image received")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)

            # Publish the edge-detected image for debugging
            try:
                edge_image_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
                self.edge_image_pub.publish(edge_image_msg)
            except CvBridgeError as e:
                rospy.logerr("Failed to convert and publish edge image: %s", str(e))

            lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
            if lines is not None:
                for rho, theta in lines[:, 0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    self.axis_of_rotation = ((x1 + x2) // 2, (y1 + y2) // 2)
                    rospy.loginfo(f"Axis of rotation: {self.axis_of_rotation}")
                    break

                # Draw the axis of rotation on the image
                if self.axis_of_rotation is not None:
                    cv2.circle(image, self.axis_of_rotation, 5, (0, 0, 255), -1)
                    rospy.loginfo("Axis of rotation plotted on image.")

                # Publish the annotated image
                try:
                    annotated_image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                    self.annotated_image_pub.publish(annotated_image_msg)
                except CvBridgeError as e:
                    rospy.logerr("Failed to convert and publish annotated image: %s", str(e))
           
            else:
                rospy.logwarn("No lines found in the RGB image.")
        except CvBridgeError as e:
            rospy.logerr(e)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        rospy.loginfo(f"Camera matrix: \n{self.camera_matrix}")

    def distance_callback(self, msg):
        self.distance_to_door = msg.data
        rospy.loginfo(f"Distance to door: {self.distance_to_door}")

    def compute_radius(self):
        if self.centroid is not None and self.axis_of_rotation is not None and self.camera_matrix is not None and self.distance_to_door is not None:
            pixel_distance = np.linalg.norm(np.array(self.centroid) - np.array(self.axis_of_rotation))
            fx = self.camera_matrix[0, 0]
            radius = (pixel_distance * self.distance_to_door) / fx
            return radius
        else:
            rospy.logwarn("One or more required values are missing: "
                          f"centroid={self.centroid}, axis_of_rotation={self.axis_of_rotation}, "
                          f"camera_matrix={self.camera_matrix is not None}, distance_to_door={self.distance_to_door}")
        return None
    
    def handle_compute_radius(self, req):
        radius = self.compute_radius()
        if radius is not None:
            return ComputeRadiusResponse(radius)
        else:
            return ComputeRadiusResponse(-1)

if __name__ == "__main__":
    rospy.init_node("door_radius_estimator")
    estimator = DoorRadiusEstimator()
    rospy.spin()
