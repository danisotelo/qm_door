#!/usr/bin/env python3

# Standard library imports
import numpy as np

# Third-party library imports
import cv2
import pcl
import torch

# ROS imports
import rospy
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs

# Custom or specific package imports
from ultralytics import YOLO
from qm_vision.srv import ComputeRef, ComputeRefResponse

@torch.no_grad()
class DoorCenter:
    def __init__(self):

        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.max_det = rospy.get_param("~maximum_detections", 1)
        self.classes = rospy.get_param("~classes", None)
        self.view_image = rospy.get_param("~view_image")
        self.threshold_lower = rospy.get_param("~threshold_l", 30)
        self.threshold_upper = rospy.get_param("~threshold_u", 255)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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

        self.depth_sub = rospy.Subscriber(rospy.get_param("~depth_image_topic"), Image, self.depth_callback, queue_size=1)
        self.camera_depth_info_sub = rospy.Subscriber(rospy.get_param("~camera_depth_info_topic"), CameraInfo, self.camera_depth_info_callback, queue_size=1)
        self.camera_color_info_sub = rospy.Subscriber(rospy.get_param("~camera_info_topic"), CameraInfo, self.camera_info_callback, queue_size=1)
        self.pointcloud_sub = rospy.Subscriber(rospy.get_param("~input_pc_topic"), PointCloud2, self.pointcloud_callback)
        self.centroid3d_sub = rospy.Subscriber(rospy.get_param("~output_centroid_topic"), PointStamped, self.centroid3d_callback)
        
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(rospy.get_param("~output_image_topic"), Image, queue_size=10)
            self.image_door_center_pub = rospy.Publisher("/door/center", Image, queue_size=10)
            self.marker_pub = rospy.Publisher("/door/normal_vector_marker", Marker, queue_size=10)

        # Initialize reference service    
        self.service = rospy.Service("get_ref", ComputeRef, self.handle_get_ref)

        # Initialize CV_Bridge
        self.bridge = CvBridge()
        self.depth_image = None

        # YOLO model inference
        self.x1 = None
        self.y1 = None
        self.x2 = None
        self.y2 = None
        self.confidence = None

        # Camera intrinsic parameters
        self.camera_info_received = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.fxd = None
        self.fyd = None
        self.cxd = None
        self.cyd = None

        self.normal_distance = None
        self.normal_vector = None
        self.x_ref = None
        self.y_ref = None
        self.psi_ref = None
        self.centroid3d = None

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def transform_pixel_coordinates(self, x, y):
        # X_color = (x - self.cx) / self.fx
        # Y_color = (y - self.cy) / self.fy 
        # # Extrinsics assummed identity
        # x_depth = int(X_color * self.fxd + self.cxd)
        # y_depth = int(Y_color * self.fyd + self.cyd)
        x_depth = int(2 * x)
        y_depth = int(720 / 480 * y)

        return x_depth, y_depth

    def callback(self, data):
        # Ensure depth image is not None
        if self.depth_image is None:
            rospy.logwarn("Depth image is not yet available.")
            return
    
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        im_c = im.copy()

        results = self.model.predict(im, max_det = self.max_det, verbose = False)
        
        if not results[0].boxes:
            rospy.logwarn("No door detected.")
            return
        
        box = results[0].boxes.xyxy
        confidences =results[0].boxes.conf
        self.x1, self.y1, self.x2, self.y2 = map(int, box[0][:4])
        self.confidence = float(confidences[0])

        height, width, channels = im.shape
        mid_x = (self.x1 + self.x2) // 2
        mid_y = height // 2

        x_depth, y_depth = self.transform_pixel_coordinates(mid_x, mid_y)

        if self.normal_distance is None or self.normal_vector is None:
            rospy.logwarn("Normal distance has not been estimated yet.")
            return
        
        # Ensure coordinates are within depth image bounds
        if y_depth >= self.depth_image.shape[0] or x_depth >= self.depth_image.shape[1]:
            rospy.logwarn(f"Depth coordinates ({x_depth}, {y_depth}) out of bounds.")
            return
        
        depth_value = self.depth_image[y_depth, x_depth] / 1000 # In m instead of mm
        
        c = np.array([0, 0, 1])
        angle = np.arccos(np.dot(c, -self.normal_vector))
        pixel_offset = mid_x - (width // 2)
        alpha = np.arccos(self.normal_distance / depth_value)

        self.psi_ref = np.sign(self.normal_vector[0]) * (angle - np.pi)
        self.x_ref = self.normal_distance - 2 + 0.33*np.cos(self.psi_ref) # TODO -d_perp0 + d_to_arm
        self.y_ref = np.sign((self.psi_ref - alpha) * pixel_offset) * np.sqrt(depth_value**2 - self.normal_distance**2)
        psi_ref_deg = self.psi_ref * 180 / np.pi
        # rospy.loginfo(f"rgb_coord: ({mid_x}, {mid_y}), depth_coord: ({x_depth}, {y_depth})")
        # rospy.loginfo(f"psi_ref: {self.psi_ref}, alpha: {alpha}, pixel_offset: {pixel_offset}, depth_value: {depth_value}")
        # rospy.loginfo(f"Calculated X_ref: {self.x_ref}, y_ref: {self.y_ref}, psi_ref: {psi_ref_deg}")
        
        if self.publish_image:
            # Draw the bounding box on the image
            cv2.rectangle(im_c, (self.x1, self.y1), (self.x2, self.y2), (0, 255, 0), 2)
            
            # Draw the middle vertical line
            mid_x = (self.x1 + self.x2) // 2
            cv2.line(im_c, (mid_x, self.y1), (mid_x, self.y2), (255, 0, 0), 2)
            
            # Add label and confidence score on the box
            label = f"Class: {'Door'}, Conf: {self.confidence:.2f}"
            cv2.putText(im_c, label, (self.x1, self.y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            output_msg = self.bridge.cv2_to_imgmsg(im_c, encoding="bgr8")
            self.image_door_center_pub.publish(output_msg)

        if self.view_image:
            cv2.imshow("Detected Door", im_c)
            cv2.waitKey(1)

    def handle_get_ref(self, req):
        if self.x_ref is None or self.y_ref is None or self.psi_ref is None:
            rospy.logwarn("Reference has not been calculated yet.")
            return ComputeRefResponse(x_ref=float('nan'), y_ref=float('nan'), psi_ref=float('nan'))
        
        rospy.loginfo(f"Service requested: Returning x_ref: {self.x_ref}, y_ref: {self.y_ref}, psi_ref: {self.psi_ref}")
        return ComputeRefResponse(x_ref=self.x_ref, y_ref=self.y_ref, psi_ref=self.psi_ref)

    def camera_info_callback(self, data):
        self.fx = data.K[0]
        self.fy = data.K[4]
        self.cx = data.K[2]
        self.cy = data.K[5]
        self.camera_info_received = True

    def camera_depth_info_callback(self, data):
        self.fxd = data.K[0]
        self.fyd = data.K[4]
        self.cxd = data.K[2]
        self.cyd = data.K[5]

    def pointcloud_callback(self, msg):

        # Convert ROS PointCloud2 to PCL PointCloud
        cloud = pcl.PointCloud()
        points_list = []

        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        cloud.from_list(points_list)

        # Apply RANSAC to extract the plane
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)

        inliers, coefficients = seg.segment()

        if len(inliers) == 0:
            rospy.logwarn("RANSAC failed to detect a plane. No inliers found.")
            return

        # Extract the normal vector to the plane
        normal_vector = np.array(coefficients[:3])
        d = coefficients[3]

        # Check if the normal vector has zero length (which should not happen with valid input)
        norm = np.linalg.norm(normal_vector)

        # Normalize the normal vector
        self.normal_vector = normal_vector / norm

        # Calculate the distance to the plane from the camera link
        self.normal_distance = abs(d / norm)

        if self.publish_image:
            self.visualize_normal_vector(self.normal_vector)

    def centroid3d_callback(self, msg):
        self.centroid3d = [msg.point.x, msg.point.y, msg.point.z]
    
    def visualize_normal_vector(self, normal_vector):
        if self.centroid3d is None:
            rospy.logwarn("Centroid is not set. Cannot visualize normal vector.")
            return
        
        try: 
            # Transform centroid from camera_link to world frame
            centroid_point = PointStamped()
            centroid_point.header.frame_id = rospy.get_param("~optical_frame")
            centroid_point.point.x = self.centroid3d[0]
            centroid_point.point.y = self.centroid3d[1]
            centroid_point.point.z = self.centroid3d[2]

            transform = self.tf_buffer.lookup_transform("world", rospy.get_param("~optical_frame"), rospy.Time(0), rospy.Duration(1.0))
            centroid_world = tf2_geometry_msgs.do_transform_point(centroid_point, transform)

            # Transform normal vector
            normal_vector_point = PointStamped()
            normal_vector_point.header.frame_id = rospy.get_param("~optical_frame2")
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
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Exception: {str(e)}")

if __name__ == "__main__":
    try:
        rospy.init_node("ref_generator", anonymous=True)
        detector = DoorCenter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
