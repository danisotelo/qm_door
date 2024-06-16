cmake_minimum_required(VERSION 3.0.2)
project(qm_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  qm_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_circle src/TestCircle.cpp)
target_link_libraries(test_circle ${catkin_LIBRARIES})

add_executable(go_to_handle src/GoToHandle.cpp)
target_link_libraries(go_to_handle ${catkin_LIBRARIES})

