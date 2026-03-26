import os

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def build_robot_description(
    base_urdf_path: str,
    overlay_path: str,
    mjcf_file: str,
    dt_sim: str,
    base_linear_kv: str,
    base_yaw_kv: str,
    arm_joint_kv: str,
) -> str:
    with open(base_urdf_path, "r", encoding="utf-8") as base_file:
        base_urdf = base_file.read()
    with open(overlay_path, "r", encoding="utf-8") as overlay_file:
        overlay = overlay_file.read()

    overlay = overlay.replace("__MJCF_FILE__", mjcf_file)
    overlay = overlay.replace("__DT_SIM__", dt_sim)
    overlay = overlay.replace("__BASE_LINEAR_KV__", base_linear_kv)
    overlay = overlay.replace("__BASE_YAW_KV__", base_yaw_kv)
    overlay = overlay.replace("__ARM_JOINT_KV__", arm_joint_kv)
    overlay = overlay.rstrip() + "\n"

    closing_tag = "</robot>"
    if closing_tag not in base_urdf:
        raise RuntimeError(f"Failed to find </robot> in {base_urdf_path}")

    return base_urdf.replace(closing_tag, overlay + closing_tag, 1)


def launch_setup(context, *args, **kwargs):
    description_share = get_package_share_directory("g7_openarm_description")
    ros2_control_share = get_package_share_directory("g7_openarm_ros2_control")

    base_urdf = os.path.join(description_share, "urdf", "g7_openarm.urdf")
    mjcf_file = os.path.join(description_share, "urdf", "g7_openarm.MJCF")
    overlay = os.path.join(ros2_control_share, "urdf", "g7_openarm_ros2_control_overlay.xml")
    controllers = os.path.join(ros2_control_share, "config", "controllers.yaml")

    dt_sim = LaunchConfiguration("dt_sim")
    base_linear_kv = LaunchConfiguration("base_linear_kv")
    base_yaw_kv = LaunchConfiguration("base_yaw_kv")
    arm_joint_kv = LaunchConfiguration("arm_joint_kv")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = build_robot_description(
        base_urdf,
        overlay,
        mjcf_file,
        dt_sim.perform(context),
        base_linear_kv.perform(context),
        base_yaw_kv.perform(context),
        arm_joint_kv.perform(context),
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=IfCondition(start_robot_state_publisher),
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
            "ignore_timestamp": True,
        }],
        output="screen",
    )

    policy_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["g7_openarm_policy_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    observation_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["g7_openarm_observation_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return [
        ros2_control_node,
        robot_state_publisher,
        policy_controller_spawner,
        observation_broadcaster_spawner,
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument("dt_sim", default_value="0.001"),
        DeclareLaunchArgument("base_linear_kv", default_value="200.0"),
        DeclareLaunchArgument("base_yaw_kv", default_value="200.0"),
        DeclareLaunchArgument("arm_joint_kv", default_value="2.33"),
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == "__main__":
    generate_launch_description()
