cmake_minimum_required(VERSION 3.14)
project(g7_openarm_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(g7_openarm_mpc_reset_coordinator
  src/G7OpenarmMpcResetCoordinator.cpp
)

ament_target_dependencies(g7_openarm_mpc_reset_coordinator
  ocs2_core
  ocs2_mobile_manipulator
  ocs2_pinocchio_interface
  ocs2_msgs
  pinocchio
  rclcpp
)

install(TARGETS g7_openarm_mpc_reset_coordinator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
