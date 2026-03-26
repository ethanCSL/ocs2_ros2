cmake_minimum_required(VERSION 3.14)
project(g7_openarm_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MUJOCO_PYTHON_PACKAGE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (NOT MUJOCO_PYTHON_PACKAGE_DIR)
  message(FATAL_ERROR "Failed to locate the Python mujoco package.")
endif ()

set(MUJOCO_INCLUDE_DIR "${MUJOCO_PYTHON_PACKAGE_DIR}/include")
set(MUJOCO_PLUGIN_DIR "${MUJOCO_PYTHON_PACKAGE_DIR}/plugin")
file(GLOB MUJOCO_SHARED_LIBS "${MUJOCO_PYTHON_PACKAGE_DIR}/libmujoco.so*")
list(SORT MUJOCO_SHARED_LIBS)
list(GET MUJOCO_SHARED_LIBS -1 MUJOCO_LIBRARY)

if (NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo headers were not found in ${MUJOCO_INCLUDE_DIR}.")
endif ()

if (NOT EXISTS "${MUJOCO_LIBRARY}")
  message(FATAL_ERROR "MuJoCo shared library was not found in ${MUJOCO_PYTHON_PACKAGE_DIR}.")
endif ()

add_library(${PROJECT_NAME} SHARED
  src/G7OpenarmMujocoSystem.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  MUJOCO_PLUGIN_DIR="${MUJOCO_PLUGIN_DIR}"
)

target_link_libraries(${PROJECT_NAME}
  ${MUJOCO_LIBRARY}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${MUJOCO_PYTHON_PACKAGE_DIR}"
  INSTALL_RPATH "${MUJOCO_PYTHON_PACKAGE_DIR}"
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface g7_openarm_hardware_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES g7_openarm_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
