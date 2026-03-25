cmake_minimum_required(VERSION 3.14)
project(g7_openarm_ocs2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config auto_generated DESTINATION share/${PROJECT_NAME}/)

ament_package()
