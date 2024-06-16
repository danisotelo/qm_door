#!/usr/bin/env python

import rospy
import pcl
import pcl.pcl_visualization
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import sensor_msgs.point_cloud2 as pc2

def pointcloud_callback(msg):
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
        rospy.loginfo("Could not estimate a planar model for the pointcloud.")
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
    distance_pub.publish(distance_msg)

    # Publish the normal vector
    normal_vector_msg = Vector3()
    normal_vector_msg.x = normal_vector[0]
    normal_vector_msg.y = normal_vector[1]
    normal_vector_msg.z = normal_vector[2]
    normal_vector_pub.publish(normal_vector_msg)

    #rospy.loginfo(f"Distance to plane: {distance_to_plane}")
    #rospy.loginfo(f"Normal vector: {normal_vector}")

if __name__ == "__main__":
    rospy.init_node("plane_extractor_node")

    pointcloud_sub = rospy.Subscriber(rospy.get_param("~input_pc_topic"), PointCloud2, pointcloud_callback)
    distance_pub = rospy.Publisher("/door/distance", Float32, queue_size=10)
    normal_vector_pub = rospy.Publisher("/door/normal_vector", Vector3, queue_size=10)

    rospy.spin()

