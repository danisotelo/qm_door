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

add_executable(TestCircle src/TestCircle.cpp)
target_link_libraries(TestCircle ${catkin_LIBRARIES})

add_executable(GoToHandle src/GoToHandle.cpp)
target_link_libraries(GoToHandle ${catkin_LIBRARIES})

