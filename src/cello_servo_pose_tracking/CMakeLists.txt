cmake_minimum_required(VERSION 3.8)
project(cello_servo_pose_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(xr_pose_tracking_node src/xr_pose_tracking_node.cpp)
target_compile_features(xr_pose_tracking_node PUBLIC cxx_std_17)
ament_target_dependencies(
  xr_pose_tracking_node
  moveit_ros_planning
  moveit_servo
  rclcpp
)
target_link_libraries(
  xr_pose_tracking_node
  moveit_servo::pose_tracking
  moveit_servo::moveit_servo_lib_parameters
)

add_executable(wrist_tf_to_target_pose_node src/wrist_tf_to_target_pose_node.cpp)
target_compile_features(wrist_tf_to_target_pose_node PUBLIC cxx_std_17)
ament_target_dependencies(
  wrist_tf_to_target_pose_node
  rclcpp
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS xr_pose_tracking_node wrist_tf_to_target_pose_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
