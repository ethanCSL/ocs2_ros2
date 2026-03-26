cmake_minimum_required(VERSION 3.14)
project(g7_openarm_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/g7_openarm_policy_controller.cpp
  src/g7_openarm_observation_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  geometry_msgs
  hardware_interface
  ocs2_core
  ocs2_mpc
  ocs2_msgs
  ocs2_oc
  ocs2_ros_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(controller_interface g7_openarm_controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES g7_openarm_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  ocs2_core
  ocs2_mpc
  ocs2_msgs
  ocs2_oc
  ocs2_ros_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)

ament_package()
