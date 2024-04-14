cmake_minimum_required(VERSION 3.10)
project(qm_description)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    urdf
    xacro
)

catkin_package()