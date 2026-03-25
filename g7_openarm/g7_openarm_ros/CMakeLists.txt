cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(g7_openarm_ros)

set(CMAKE_BUILD_TYPE Release)

set(dependencies
        rclcpp
        rosgraph_msgs
        std_srvs
        pinocchio
        ocs2_mobile_manipulator
        ocs2_mobile_manipulator_ros
        ocs2_ros_interfaces
)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(ocs2_mobile_manipulator_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)

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

add_executable(g7_openarm_mujoco_mrt_node
        src/G7OpenarmMujocoMrtNode.cpp
        "${CMAKE_CURRENT_SOURCE_DIR}/../../basic examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorDummyVisualization.cpp"
)
target_include_directories(g7_openarm_mujoco_mrt_node PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        "${CMAKE_CURRENT_SOURCE_DIR}/../../basic examples/ocs2_mobile_manipulator_ros/include"
)
target_compile_definitions(g7_openarm_mujoco_mrt_node PRIVATE
        MUJOCO_PLUGIN_DIR="${MUJOCO_PLUGIN_DIR}"
)
target_link_libraries(g7_openarm_mujoco_mrt_node
        ${MUJOCO_LIBRARY}
        ocs2_mobile_manipulator_ros::ocs2_mobile_manipulator_ros
)
set_target_properties(g7_openarm_mujoco_mrt_node PROPERTIES
        BUILD_RPATH "${MUJOCO_PYTHON_PACKAGE_DIR}"
        INSTALL_RPATH "${MUJOCO_PYTHON_PACKAGE_DIR}"
)
ament_target_dependencies(g7_openarm_mujoco_mrt_node ${dependencies})

install(TARGETS g7_openarm_mujoco_mrt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
