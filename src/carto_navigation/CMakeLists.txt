cmake_minimum_required(VERSION 3.0.2)
project(carto_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cartographer_ros
  cartographer_ros_msgs
  move_base
  teb_local_planner
)
find_package(cartographer)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agv_robot
  CATKIN_DEPENDS  cartographer_ros_msgs cartographer_ros move_base teb_local_planner
#  DEPENDS cartographer
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


