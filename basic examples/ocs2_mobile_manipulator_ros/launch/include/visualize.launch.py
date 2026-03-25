import os
import launch
import launch_ros.actions
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            # default_value=get_package_share_directory('ocs2_robotic_assets') + "/resources/mobile_manipulator/franka/urdf/panda.urdf"
        ),
        launch.actions.DeclareLaunchArgument(
            name='test',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory('ocs2_mobile_manipulator_ros') + "/rviz/mobile_manipulator.rviz"
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[LaunchConfiguration("urdfFile")],
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            arguments=[LaunchConfiguration("urdfFile")],
            condition=IfCondition(LaunchConfiguration("test")),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mobile_manipulator',
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rvizconfig")],
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
