cmake_minimum_required(VERSION 3.0.2)
project(qm_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  qm_msgs
  qm_controllers
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp tf std_msgs qm_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_circle src/TestCircle.cpp)
target_link_libraries(test_circle ${catkin_LIBRARIES})