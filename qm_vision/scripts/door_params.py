#!/usr/bin/env python3
import rospy
import pcl
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Point, PointStamped
from cv_bridge import CvBridge, CvBridgeError
from qm_vision.srv import ComputeRadius, ComputeRadiusResponse
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs

class DoorParametersEstimator:
    def __init__(self):
        self.bridge = CvBridge()
        self.centroid = None
        self.axis_of_rotation = None
        self.camera_matrix = None
        self.distance_to_door = None
        self.radius = None
        self.centroid3d = None
        self.view_image = rospy.get_param("~view_image")
        self.publish_image = rospy.get_param("~publish_image")

        # Subscribers 
        self.handle_mask_sub = rospy.Subscriber(rospy.get_param("~output_mask_topic"), Image, self.handle_mask_callback)
        self.rgb_image_sub = rospy.Subscriber(rospy.get_param("~input_image_topic"), Image, self.rgb_image_callback)
        self.camera_info_sub = rospy.Subscriber(rospy.get_param("~camera_info_topic"), CameraInfo, self.camera_info_callback)
        self.distance_sub = rospy.Subscriber("/door/distance", Float32, self.distance_callback)
        self.pointcloud_sub = rospy.Subscriber(rospy.get_param("~input_pc_topic"), PointCloud2, self.pointcloud_callback)
        self.centroid3d_sub = rospy.Subscriber(rospy.get_param("~output_centroid_topic"), PointStamped, self.centroid3d_callback)

        # Publisher
        self.radius_service = rospy.Service("/compute_radius", ComputeRadius, self.handle_compute_radius)
        self.edge_image_pub = rospy.Publisher("/door/edge_detection", Image, queue_size=10)
        self.annotated_image_pub = rospy.Publisher("/door/rotation_axis", Image, queue_size=10)
        self.distance_pub = rospy.Publisher("/door/distance", Float32, queue_size=10)
        self.normal_vector_pub = rospy.Publisher("/door/normal_vector", Vector3, queue_size=10)
        if self.publish_image:
            self.marker_pub = rospy.Publisher("/door/normal_vector_marker", Marker, queue_size=10)

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
            rospy.loginfo("RGB image received")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)

            # Publish the edge-detected image for debugging
            try:
                edge_image_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
                self.edge_image_pub.publish(edge_image_msg)
                rospy.loginfo("Published edge-detected image")
            except CvBridgeError as e:
                rospy.logerr("Failed to convert and publish edge image: %s", str(e))

            lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
            if lines is not None:
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
                        distance_to_opposite = abs(x1 - (image.shape[1] - handle_x))
                        if distance_to_opposite < min_distance:
                            min_distance = distance_to_opposite
                            axis_line = (x1, y1, x2, y2)

                # Draw the axis of rotation
                if axis_line is not None:
                    x1, y1, x2, y2 = axis_line
                    if self.publish_image:
                        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        #rospy.loginfo("Axis of rotation plotted on image")
                        self.axis_of_rotation = ((x1 + x2) // 2, (y1 + y2) // 2)

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

    def distance_callback(self, msg):
        self.distance_to_door = msg.data
        #rospy.loginfo(f"Distance to door: {self.distance_to_door}")

    def compute_radius(self):
        if self.centroid is not None and self.axis_of_rotation is not None and self.camera_matrix is not None and self.distance_to_door is not None:
            fx = self.camera_matrix[0, 0]
            radius_in_meters = (self.radius * self.distance_to_door) / fx
            return radius_in_meters
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
            rospy.loginfo("Could not estimate a planar model.")
            return
        
        # Extract the normal vector to the plane
        normal_vector = coefficients[:3]
        d = coefficients[3]

        # Normalize the normal vector
        norm = np.linalg.norm(normal_vector)
        normal_vector = normal_vector / norm

        # Calculate the distance to the plane from the camera link
        distance_to_plane = abs(d / norm)

        # Publish the distance to the plane
        distance_msg = Float32()
        distance_msg.data = distance_to_plane
        self.distance_pub.publish(distance_msg)

        # Publish the normal vector
        normal_vector_msg = Vector3()
        normal_vector_msg.x = normal_vector[0]
        normal_vector_msg.y = normal_vector[1]
        normal_vector_msg.z = normal_vector[2]
        self.normal_vector_pub.publish(normal_vector_msg)

        # Visualize the normal vector in Rviz
        if self.publish_image:
            self.visualize_normal_vector(normal_vector)

        #rospy.loginfo(f"Distance to plane: {distance_to_plane}")
        #rospy.loginfo(f"Normal vector: {normal_vector}")

    def centroid3d_callback(self, msg):
        self.centroid3d = [msg.point.x, msg.point.y, msg.point.z]
        # rospy.loginfo(f"Centroid received: x={self.centroid_x}, y ={self.centroid_y}, z = {self.centroid_z}")
    
    def visualize_normal_vector(self, normal_vector):
        if self.centroid3d is None:
            rospy.logwarn("Centroid is not set. Cannot visualize normal vector.")
            return
        
        try: 
            # Create a TransformListener
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)

            # Transform centroid from camera_link to world frame
            centroid_point = PointStamped()
            centroid_point.header.frame_id = "camera_link_optical"
            centroid_point.point.x = self.centroid3d[0]
            centroid_point.point.y = self.centroid3d[1]
            centroid_point.point.z = self.centroid3d[2]

            transform = tf_buffer.lookup_transform("world", "camera_link_optical", rospy.Time(0), rospy.Duration(1.0))
            centroid_world = tf2_geometry_msgs.do_transform_point(centroid_point, transform)

            # Transform normal vector
            normal_vector_point = PointStamped()
            normal_vector_point.header.frame_id = "camera_link_optical"
            normal_vector_point.point.x = self.centroid3d[0] - normal_vector[0]/2
            normal_vector_point.point.y = self.centroid3d[1] - normal_vector[1]/2
            normal_vector_point.point.z = self.centroid3d[2] - normal_vector[2]/2

            normal_world = tf2_geometry_msgs.do_transform_point(normal_vector_point, transform)

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "normal_vector"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Start point of the arrow (origin)
            start_point = centroid_world.point
            # End point of the arrow (normal vector scaled for visibility)
            end_point = normal_world.point

            marker.points = [start_point, end_point]

            marker.scale.x = 0.05
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            # Initialize orientation to identity
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            self.marker_pub.publish(marker)
            rospy.loginfo("Published normal vector marker")
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Exception: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("door_radius_estimator")
    estimator = DoorParametersEstimator()
    rospy.spin()
