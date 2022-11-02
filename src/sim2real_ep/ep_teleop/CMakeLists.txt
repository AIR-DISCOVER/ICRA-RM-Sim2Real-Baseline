cmake_minimum_required(VERSION 3.0.2)
project(ep_teleop)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
  diagnostic_updater)
  
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES agv_robot
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_teleop_node
  src/control_teleop.cpp
  )
target_link_libraries(control_teleop_node
  ${catkin_LIBRARIES}
  )
add_executable(joy_driver
  src/joy_driver.cpp
  )
target_link_libraries(joy_driver
  ${catkin_LIBRARIES}
  )

