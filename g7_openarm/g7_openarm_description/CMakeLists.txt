cmake_minimum_required(VERSION 3.14)
project(g7_openarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/)

ament_package()
