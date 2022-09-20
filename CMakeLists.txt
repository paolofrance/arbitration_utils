cmake_minimum_required(VERSION 3.0.2)
project(arbitration_utils)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
roscpp
moveit_core
rosdyn_core
moveit_ros_planning_interface
object_loader_msgs
tf_conversions
rospy
std_msgs
)

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
  moveit_core
  rosdyn_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
)

add_library(
arbitration_utils src/arbitration_utils.cpp
)

add_executable(arbitration_node src/arbitration_node.cpp)
target_link_libraries( arbitration_node arbitration_utils ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} fuzzylite)
add_dependencies(arbitration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
