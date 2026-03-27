#!/usr/bin/python3

import os
import time

import mujoco
import mujoco.viewer
import rclpy
from ocs2_msgs.msg import MpcObservation
from rclpy.node import Node


class G7OpenarmMujocoViewer(Node):
    BASE_JOINT_NAMES = (
        "base_x_joint",
        "base_y_joint",
        "base_yaw_joint",
    )

    ARM_JOINT_NAMES = (
        "L_1_joint", "L_2_joint", "L_3_joint", "L_4_joint", "L_5_joint", "L_6_joint", "L_7_joint",
        "R_1_joint", "R_2_joint", "R_3_joint", "R_4_joint", "R_5_joint", "R_6_joint", "R_7_joint",
    )

    def __init__(self):
        super().__init__("g7_openarm_mujoco_viewer")

        self.declare_parameter("mjcfFile", "")
        self.declare_parameter("observation_topic", "/mobile_manipulator_mpc_observation")
        self.declare_parameter("update_rate_hz", 60.0)
        self.declare_parameter("camera_distance", 3.8)
        self.declare_parameter("camera_azimuth", 135.0)
        self.declare_parameter("camera_elevation", -22.0)
        self.declare_parameter("camera_lookat", [0.0, 0.0, 0.8])

        self.mjcf_file = self.get_parameter("mjcfFile").get_parameter_value().string_value
        self.observation_topic = (
            self.get_parameter("observation_topic").get_parameter_value().string_value
        )
        self.update_rate_hz = (
            self.get_parameter("update_rate_hz").get_parameter_value().double_value
        )
        self.camera_distance = (
            self.get_parameter("camera_distance").get_parameter_value().double_value
        )
        self.camera_azimuth = (
            self.get_parameter("camera_azimuth").get_parameter_value().double_value
        )
        self.camera_elevation = (
            self.get_parameter("camera_elevation").get_parameter_value().double_value
        )
        self.camera_lookat = list(
            self.get_parameter("camera_lookat").get_parameter_value().double_array_value
        )

        if not self.mjcf_file:
            raise RuntimeError("Parameter 'mjcfFile' is required.")
        if not os.path.exists(self.mjcf_file):
            raise RuntimeError(f"MJCF file does not exist: {self.mjcf_file}")
        if self.update_rate_hz <= 0.0:
            raise RuntimeError("Parameter 'update_rate_hz' must be positive.")

        self.model = mujoco.MjModel.from_xml_path(self.mjcf_file)
        self.data = mujoco.MjData(self.model)
        self._apply_visual_tuning()
        self.base_qpos_addresses = self._resolve_qpos_addresses(self.BASE_JOINT_NAMES)
        self.arm_qpos_addresses = self._resolve_qpos_addresses(self.ARM_JOINT_NAMES)
        self.latest_state = None
        self.has_logged_first_observation = False

        self.subscription = self.create_subscription(
            MpcObservation,
            self.observation_topic,
            self._observation_callback,
            10,
        )

    def _resolve_qpos_addresses(self, joint_names):
        addresses = []
        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise RuntimeError(f"Failed to resolve MuJoCo joint '{joint_name}'.")
            addresses.append(self.model.jnt_qposadr[joint_id])
        return addresses

    def _observation_callback(self, msg):
        if len(msg.state.value) < 17:
            self.get_logger().warning(
                f"Ignoring observation with {len(msg.state.value)} state entries; expected at least 17.",
                throttle_duration_sec=2.0,
            )
            return

        self.latest_state = [float(value) for value in msg.state.value[:17]]
        if not self.has_logged_first_observation:
            self.has_logged_first_observation = True
            self.get_logger().info(
                f"Received first observation on {self.observation_topic}; viewer is now live."
            )

    def _apply_visual_tuning(self):
        return

    def _apply_latest_state(self):
        if self.latest_state is None:
            return False

        state = self.latest_state
        self.data.qpos[self.base_qpos_addresses[0]] = state[0]
        self.data.qpos[self.base_qpos_addresses[1]] = state[1]
        self.data.qpos[self.base_qpos_addresses[2]] = state[2]

        for joint_index, qpos_address in enumerate(self.arm_qpos_addresses):
            self.data.qpos[qpos_address] = state[3 + joint_index]

        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return True

    def run(self):
        update_period = 1.0 / self.update_rate_hz
        self.get_logger().info(
            f"Opening MuJoCo viewer with MJCF '{self.mjcf_file}' and topic '{self.observation_topic}'."
        )

        with mujoco.viewer.launch_passive(
            self.model,
            self.data,
        ) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.distance = self.camera_distance
            viewer.cam.azimuth = self.camera_azimuth
            viewer.cam.elevation = self.camera_elevation
            if len(self.camera_lookat) == 3:
                viewer.cam.lookat[:] = self.camera_lookat
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0.01)
                with viewer.lock():
                    self._apply_latest_state()
                viewer.sync()
                time.sleep(update_period)


def main():
    rclpy.init()
    node = None
    try:
        node = G7OpenarmMujocoViewer()
        node.run()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
