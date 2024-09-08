#!/usr/bin/env python

# Standard library imports
import numpy as np

# Third-party library imports
from sklearn.decomposition import PCA
from sklearn.linear_model import RANSACRegressor

# ROS imports
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class PointCloudOrientation:
    def __init__(self):
        # Initialize the node
        rospy.init_node('pointcloud_orientation', anonymous=True)

        # Subscriber to the PointCloud2 topic
        self.pc_sub = rospy.Subscriber(rospy.get_param("~output_pc_topic"), PointCloud2, self.pc_callback)

        # Publisher for the centroid
        self.centroid_pub = rospy.Publisher(rospy.get_param("~output_centroid_topic"), PointStamped, queue_size=10)

        # Publisher for the axes marker
        self.marker_pub = rospy.Publisher(rospy.get_param("~output_orientation_topic"), Marker, queue_size=10)

        # Store previous axes to ensure consistency
        self.previous_x_axis = None
        self.previous_y_axis = None
        self.previous_z_axis = None

    def pc_callback(self, msg):
        # Convert PointCloud2 message to a list of points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if not points:
            rospy.logwarn("No points in the PointCloud2 message.")
            return

        # Convert to numpy array for easy calculation
        np_points = np.array(points)

        # Compute the centroid
        centroid = np.mean(np_points, axis=0)

        # Create a PointStamped message for the centroid
        centroid_msg = PointStamped()
        centroid_msg.header.stamp = rospy.Time.now()
        centroid_msg.header.frame_id = msg.header.frame_id
        centroid_msg.point.x = centroid[0]
        centroid_msg.point.y = centroid[1]
        centroid_msg.point.z = centroid[2]

        # Publish the centroid
        self.centroid_pub.publish(centroid_msg)

        # Use RANSAC to fit a line to the points
        ransac = RANSACRegressor()
        X = np_points[:, [0, 1]]  # Use x and y coordinates
        y = np_points[:, 2]       # Use z coordinate
        ransac.fit(X, y)

        # Get the inlier points
        inlier_mask = ransac.inlier_mask_
        inlier_points = np_points[inlier_mask]

        if len(inlier_points) < 2:
            rospy.logwarn("Not enough inlier points for RANSAC.")
            return

        # Use PCA to find the main axes of the inlier points
        pca = PCA(n_components=3)
        pca.fit(inlier_points)

        # Get the principal directions
        x_axis = pca.components_[0]
        y_axis = pca.components_[1]
        z_axis = np.cross(x_axis, y_axis)  # Ensure z-axis is perpendicular to x and y

        # Invert the x-axis direction
        x_axis = -x_axis

        # Normalize the axes
        x_axis /= np.linalg.norm(x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis /= np.linalg.norm(z_axis)

        # Ensure consistency with previous axes
        if self.previous_x_axis is not None:
            x_axis = self.ensure_consistent_sign(x_axis, self.previous_x_axis)
            y_axis = self.ensure_consistent_sign(y_axis, self.previous_y_axis)
            z_axis = self.ensure_consistent_sign(z_axis, self.previous_z_axis)

        self.previous_x_axis = x_axis
        self.previous_y_axis = y_axis
        self.previous_z_axis = z_axis

        # Create a Marker message for the axes
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "axes"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width

        # Initialize marker quaternion to identity
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        # Points for the axes
        p_start = Point(centroid[0], centroid[1], centroid[2])

        # X axis (red)
        p_end_x = Point(centroid[0] + 0.1 * x_axis[0], centroid[1] + 0.1 * x_axis[1], centroid[2] + 0.1 * x_axis[2])
        marker.points.append(p_start)
        marker.points.append(p_end_x)
        marker.colors.append(self.create_color(1.0, 0.0, 0.0, 1.0))
        marker.colors.append(self.create_color(1.0, 0.0, 0.0, 1.0))

        # Y axis (green)
        p_end_y = Point(centroid[0] + 0.1 * y_axis[0], centroid[1] + 0.1 * y_axis[1], centroid[2] + 0.1 * y_axis[2])
        marker.points.append(p_start)
        marker.points.append(p_end_y)
        marker.colors.append(self.create_color(0.0, 1.0, 0.0, 1.0))
        marker.colors.append(self.create_color(0.0, 1.0, 0.0, 1.0))

        # Z axis (blue)
        p_end_z = Point(centroid[0] + 0.1 * z_axis[0], centroid[1] + 0.1 * z_axis[1], centroid[2] + 0.1 * z_axis[2])
        marker.points.append(p_start)
        marker.points.append(p_end_z)
        marker.colors.append(self.create_color(0.0, 0.0, 1.0, 1.0))
        marker.colors.append(self.create_color(0.0, 0.0, 1.0, 1.0))

        # Publish the marker
        self.marker_pub.publish(marker)

        #rospy.loginfo("Centroid: x=%f, y=%f, z=%f" % (centroid[0], centroid[1], centroid[2]))

    def ensure_consistent_sign(self, new_axis, previous_axis):
        """ Ensure the sign of the new axis is consistent with the previous axis. """
        if np.dot(new_axis, previous_axis) < 0:
            new_axis = -new_axis
        return new_axis

    def create_color(self, r, g, b, a):
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color

if __name__ == '__main__':
    try:
        PointCloudOrientation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass