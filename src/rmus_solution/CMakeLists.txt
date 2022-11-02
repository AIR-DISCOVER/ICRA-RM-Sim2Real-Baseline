cmake_minimum_required(VERSION 3.0.2)
project(rmus_solution)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  setgoal.srv
  switch.srv
  graspsignal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)