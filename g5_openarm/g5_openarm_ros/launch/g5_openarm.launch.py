import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Whether to open RViz.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false',
            description='Whether to run the MPC node in debug terminal mode.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'g5_openarm_description') + '/urdf/g5_openarm.urdf',
            description='URDF used by robot_state_publisher and RViz.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'g5_openarm_ocs2') + '/config/g5_openarm/task.info',
            description='Normal OCS2 configuration.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=get_package_share_directory(
                'g5_openarm_ocs2') + '/auto_generated/g5_openarm',
            description='Folder containing generated dynamics libraries.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory(
                'g5_openarm_ros') + '/rviz/g5_openarm.rviz',
            description='RViz layout to load.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enableJoystick',
            default_value='false',
            description='Whether to enable joystick control for the interactive target.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enableAutoPosition',
            default_value='false',
            description='Whether to auto-follow the end-effector pose with the target marker.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enableDynamicFrame',
            default_value='false',
            description='Whether to choose the target marker frame from the task file.'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('g5_openarm_ros'),
                    'launch/include/g5_openarm_bringup.launch.py',
                )
            ),
            launch_arguments={
                'rviz': launch.substitutions.LaunchConfiguration('rviz'),
                'debug': launch.substitutions.LaunchConfiguration('debug'),
                'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
                'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
                'libFolder': launch.substitutions.LaunchConfiguration('libFolder'),
                'rvizconfig': launch.substitutions.LaunchConfiguration('rvizconfig'),
                'enableJoystick': launch.substitutions.LaunchConfiguration('enableJoystick'),
                'enableAutoPosition': launch.substitutions.LaunchConfiguration('enableAutoPosition'),
                'enableDynamicFrame': launch.substitutions.LaunchConfiguration('enableDynamicFrame'),
            }.items(),
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
