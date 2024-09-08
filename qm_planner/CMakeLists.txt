cmake_minimum_required(VERSION 3.0.2)
project(qm_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  qm_msgs
  qm_controllers
  qm_vision
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp tf std_msgs qm_msgs geometry_msgs qm_vision
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_circle src/TestCircle.cpp)
target_link_libraries(test_circle ${catkin_LIBRARIES})

add_executable(go_to_door src/GoToDoor.cpp)
target_link_libraries(go_to_door ${catkin_LIBRARIES})

add_executable(open_door src/OpenDoor.cpp)
target_link_libraries(open_door ${catkin_LIBRARIES})