import os

import launch
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'g7_openarm_description') + '/urdf/g7_openarm.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mjcfFile',
            default_value=get_package_share_directory(
                'g7_openarm_description') + '/urdf/g7_openarm.MJCF'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'g7_openarm_ocs2') + '/config/g7_openarm/task.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=get_package_share_directory(
                'g7_openarm_ocs2') + '/auto_generated/g7_openarm'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory(
                'g7_openarm_ros') + '/rviz/g7_openarm.rviz'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enableJoystick',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enableAutoPosition',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enableDynamicFrame',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dtSim',
            default_value='0.001'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dtCtrl',
            default_value='0.01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='baseLinearKv',
            default_value='200.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='baseYawKv',
            default_value='200.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='armJointKv',
            default_value='2.33'
        ),
        launch.actions.DeclareLaunchArgument(
            name='useLegacyMrtExecution',
            default_value='false'
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[LaunchConfiguration('urdfFile')],
            parameters=[{
                'use_sim_time': True,
                'ignore_timestamp': True,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mobile_manipulator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=["-d", LaunchConfiguration('rvizconfig')],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                },
                {
                    'use_sim_time': True
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('g7_openarm_ros2_control'),
                    'launch/g7_openarm_ros2_control.launch.py',
                )
            ),
            condition=UnlessCondition(LaunchConfiguration('useLegacyMrtExecution')),
            launch_arguments={
                'dt_sim': LaunchConfiguration('dtSim'),
                'base_linear_kv': LaunchConfiguration('baseLinearKv'),
                'base_yaw_kv': LaunchConfiguration('baseYawKv'),
                'arm_joint_kv': LaunchConfiguration('armJointKv'),
                'start_robot_state_publisher': 'false',
                'use_sim_time': 'true',
            }.items(),
        ),
        Node(
            package='g7_openarm_ros2_control',
            executable='g7_openarm_mpc_reset_coordinator',
            name='g7_openarm_mpc_reset_coordinator',
            condition=UnlessCondition(LaunchConfiguration('useLegacyMrtExecution')),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                },
                {
                    'observation_topic': '/mobile_manipulator_mpc_observation'
                },
                {
                    'reset_service': '/mobile_manipulator_mpc_reset'
                },
                {
                    'use_sim_time': True
                }
            ]
        ),
        Node(
            package='g7_openarm_ros',
            executable='g7_openarm_mujoco_mrt_node',
            name='g7_openarm_mujoco_mrt',
            condition=IfCondition(LaunchConfiguration('useLegacyMrtExecution')),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'mjcfFile': launch.substitutions.LaunchConfiguration('mjcfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                },
                {
                    'dtSim': launch.substitutions.LaunchConfiguration('dtSim')
                },
                {
                    'dtCtrl': launch.substitutions.LaunchConfiguration('dtCtrl')
                },
                {
                    'baseLinearKv': launch.substitutions.LaunchConfiguration('baseLinearKv')
                },
                {
                    'baseYawKv': launch.substitutions.LaunchConfiguration('baseYawKv')
                },
                {
                    'armJointKv': launch.substitutions.LaunchConfiguration('armJointKv')
                },
                {
                    'use_sim_time': True
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_target',
            name='mobile_manipulator_target',
            condition=IfCondition(launch.substitutions.LaunchConfiguration("rviz")),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'enableJoystick': launch.substitutions.LaunchConfiguration('enableJoystick')
                },
                {
                    'enableAutoPosition': launch.substitutions.LaunchConfiguration('enableAutoPosition')
                },
                {
                    'enableDynamicFrame': launch.substitutions.LaunchConfiguration('enableDynamicFrame')
                },
                {
                    'use_sim_time': True
                }
            ]
        ),
    ])
