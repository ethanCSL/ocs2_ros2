cmake_minimum_required(VERSION 3.14)
project(g7_openarm_ros)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/g7_openarm_mujoco_viewer.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
