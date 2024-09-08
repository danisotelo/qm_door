#!/usr/bin/env python3

# Standard library imports
import numpy as np

# Third-party library imports
import cv2
import pcl

# ROS imports
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

# Custom imports
from qm_vision.srv import ComputeRadius, ComputeRadiusResponse

class DoorParametersEstimator:
    def __init__(self):
        self.bridge = CvBridge()
        self.centroid = None
        self.axis_of_rotation = None
        self.camera_matrix = None
        self.distance_to_door = None
        self.radius = None
        self.typedoor = 0
        self.centroid3d = None
        self.view_image = rospy.get_param("~view_image")
        self.publish_image = rospy.get_param("~publish_image")

        # Subscribers 
        self.handle_mask_sub = rospy.Subscriber(rospy.get_param("~output_mask_topic"), Image, self.handle_mask_callback)
        self.rgb_image_sub = rospy.Subscriber(rospy.get_param("~input_image_topic"), Image, self.rgb_image_callback)
        self.camera_info_sub = rospy.Subscriber(rospy.get_param("~camera_info_topic"), CameraInfo, self.camera_info_callback)
        self.pointcloud_sub = rospy.Subscriber(rospy.get_param("~input_pc_topic"), PointCloud2, self.pointcloud_callback)

        # Publishers
        self.radius_service = rospy.Service("/compute_radius", ComputeRadius, self.handle_compute_radius)
        if self.publish_image:
            self.edge_image_pub = rospy.Publisher("/door/edge_detection", Image, queue_size=10)
            self.annotated_image_pub = rospy.Publisher("/door/rotation_axis", Image, queue_size=10)
        
        self.distance_pub = rospy.Publisher("/door/distance", Float32, queue_size = 10)

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
                    #rospy.loginfo(f"Centroid: {self.centroid}")
                else:
                    rospy.logwarn("Moment calculation resulted in division by zero.")
            else:
                rospy.logwarn("No contours found in the handle mask.")
        except CvBridgeError as e:
            rospy.logerr(e)

    def rgb_image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #rospy.loginfo("RGB image received")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)

            # Publish the edge-detected image for debugging
            if self.publish_image:
                try:
                    edge_image_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
                    self.edge_image_pub.publish(edge_image_msg)
                    #rospy.loginfo("Published edge-detected image")
                except CvBridgeError as e:
                    rospy.logerr("Failed to convert and publish edge image: %s", str(e))

            lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
            if lines is not None:

                if self.centroid is None:
                    #rospy.logwarn("Centroid is not set. Cannot calculate handle position.")
                    return
                
                handle_x, handle_y = self.centroid

                # Look for vertical lines around the opposite side of the handle
                axis_line = None
                min_distance = float('inf')
                for rho, theta in lines[:, 0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    # Check if the line is vertical
                    if abs(x1 - x2) < 200:
                        distance_to_opposite = abs(x0 - (image.shape[1] - handle_x))
                        if distance_to_opposite < min_distance:
                            min_distance = distance_to_opposite
                            axis_line = (x1, y1, x2, y2)

                # Draw the axis of rotation
                if axis_line is not None:
                    x1, y1, x2, y2 = axis_line
                    if self.publish_image:
                        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        #rospy.loginfo("Axis of rotation plotted on image")
                        self.axis_of_rotation = (x0, y0)

                    # Compute the intersection of the horizontal line through centroid with axis of rotation
                    if x1 != x2: 
                        slope = (y2 - y1) / (x2 - x1)
                        intercept = y1 - slope * x1
                        intersection_x = (handle_y - intercept) / slope
                        intersection_y = handle_y

                        # Draw the intersection point and radius on the image
                        intersection_point = (int(intersection_x), int(intersection_y))
                        if self.publish_image:
                            cv2.circle(image, intersection_point, 5, (255, 0, 0), -1)
                            cv2.line(image, (handle_x, handle_y), (int(intersection_x), handle_y), (0, 255, 0), 2)

                        # Calculate the horizontal distance (radius)
                        radius = abs(handle_x - intersection_x)
                        #rospy.loginfo(f"Calculated radius: {radius}")
                        self.radius = radius
                        self.typedoor = int(np.sign(handle_x - intersection_x))

                if self.publish_image:
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
        #rospy.loginfo(f"Camera matrix: \n{self.camera_matrix}")

    def compute_radius(self):
        if self.radius is not None and self.distance_to_door is not None and self.camera_matrix is not None:
            fx = self.camera_matrix[0, 0]
            radius_in_meters = (self.radius * self.distance_to_door) / fx
            return radius_in_meters
        else:
            rospy.logwarn("Missing parameters: radius=%s, distance_to_door=%s, camera_matrix=%s",
                          self.radius, self.distance_to_door, self.camera_matrix)
            return None
    
    def handle_compute_radius(self, req):
        radius = self.compute_radius()
        if radius is not None and self.typedoor is not None:
            return ComputeRadiusResponse(radius = radius, typedoor = self.typedoor)
        else:
            return ComputeRadiusResponse(radius = -1, typedoor = 0)
        
    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 to PCL PointCloud
        cloud = pcl.PointCloud()
        points_list = []

        for point in pc2.read_points(msg, skip_nans = True):
            points_list.append([point[0], point[1], point[2]])

        cloud.from_list(points_list)

        # Apply RANSAC to extract the plane
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)
        inliers, coefficients = seg.segment()

        if len(inliers) == 0:
            #rospy.loginfo("Could not estimate a planar model.")
            return
        
        # Extract the normal vector to the plane
        normal_vector = coefficients[:3]
        d = coefficients[3]

        # Calculate the real distance
        norm = np.linalg.norm(normal_vector)
        self.distance_to_door = abs(d / norm)

        # rospy.loginfo(f"Distance to plane: {self.distance_to_door}")
        self.distance_pub.publish(self.distance_to_door)
        #rospy.loginfo(f"Normal vector: {normal_vector}")

if __name__ == "__main__":
    rospy.init_node("door_radius_estimator")
    estimator = DoorParametersEstimator()
    rospy.spin()
