cmake_minimum_required(VERSION 3.8)
project(pose_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(find_object_2d REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
 

add_executable(transform_pose src/transform_pose.cpp)
ament_target_dependencies(
  transform_pose
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  find_object_2d
)

add_executable(move2object src/move2object.cpp)
ament_target_dependencies(
  move2object
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

install(TARGETS 
  transform_pose move2object
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
