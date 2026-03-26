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
                'g7_openarm_description') + '/urdf/g7_openarm.urdf',
            description='URDF used by robot_state_publisher and RViz.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mjcfFile',
            default_value=get_package_share_directory(
                'g7_openarm_description') + '/urdf/g7_openarm.MJCF',
            description='MuJoCo MJCF used by the closed-loop simulator.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'g7_openarm_ocs2') + '/config/g7_openarm/task_pinnzoo.info',
            description='PinnZoo-backed OCS2 configuration.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=get_package_share_directory(
                'g7_openarm_ocs2') + '/auto_generated/g7_openarm',
            description='Folder containing generated OCS2 helper libraries.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory(
                'g7_openarm_ros') + '/rviz/g7_openarm.rviz',
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
        launch.actions.DeclareLaunchArgument(
            name='dtSim',
            default_value='0.001',
            description='MuJoCo simulation timestep.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dtCtrl',
            default_value='0.01',
            description='Closed-loop control timestep.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='baseLinearKv',
            default_value='250.0',
            description='Velocity-servo gain for base planar joints.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='baseYawKv',
            default_value='200.0',
            description='Velocity-servo gain for base yaw joint.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='armJointKv',
            default_value='20.0',
            description='Velocity-servo gain for arm joints.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='useLegacyMrtExecution',
            default_value='false',
            description='Whether to use the legacy G7OpenarmMujocoMrtNode direct execution path.'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('g7_openarm_ros'),
                    'launch/include/g7_openarm_bringup.launch.py',
                )
            ),
            launch_arguments={
                'rviz': launch.substitutions.LaunchConfiguration('rviz'),
                'debug': launch.substitutions.LaunchConfiguration('debug'),
                'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
                'mjcfFile': launch.substitutions.LaunchConfiguration('mjcfFile'),
                'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
                'libFolder': launch.substitutions.LaunchConfiguration('libFolder'),
                'rvizconfig': launch.substitutions.LaunchConfiguration('rvizconfig'),
                'enableJoystick': launch.substitutions.LaunchConfiguration('enableJoystick'),
                'enableAutoPosition': launch.substitutions.LaunchConfiguration('enableAutoPosition'),
                'enableDynamicFrame': launch.substitutions.LaunchConfiguration('enableDynamicFrame'),
                'dtSim': launch.substitutions.LaunchConfiguration('dtSim'),
                'dtCtrl': launch.substitutions.LaunchConfiguration('dtCtrl'),
                'baseLinearKv': launch.substitutions.LaunchConfiguration('baseLinearKv'),
                'baseYawKv': launch.substitutions.LaunchConfiguration('baseYawKv'),
                'armJointKv': launch.substitutions.LaunchConfiguration('armJointKv'),
                'useLegacyMrtExecution': launch.substitutions.LaunchConfiguration('useLegacyMrtExecution'),
            }.items(),
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
