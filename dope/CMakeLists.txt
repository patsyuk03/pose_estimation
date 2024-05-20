cmake_minimum_required(VERSION 3.5.2)
project(dope)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  resource_retriever
  rospy
  sensor_msgs
  std_msgs
  tf
  vision_msgs
  visualization_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs vision_msgs visualization_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/dope
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
