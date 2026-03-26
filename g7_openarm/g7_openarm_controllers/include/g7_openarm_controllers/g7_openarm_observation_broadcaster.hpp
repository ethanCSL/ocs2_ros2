#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace g7_openarm_controllers
{

class G7OpenarmObservationBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  size_t find_state_interface_index(const std::string & name) const;

  std::vector<std::string> state_interface_names_;
  std::string mpc_observation_topic_ = "/mobile_manipulator_mpc_observation";
  std::string debug_observation_topic_ = "/mujoco/debug_observation";
  std::string debug_input_topic_ = "/mujoco/debug_input";
  std::string debug_base_pose_topic_ = "/mujoco/debug_base_pose";
  std::string clock_topic_ = "/clock";
  std::string joint_states_topic_ = "/joint_states";
  std::string world_frame_ = "world";
  std::string base_frame_ = "AMR_base_link";

  rclcpp_lifecycle::LifecyclePublisher<ocs2_msgs::msg::MpcObservation>::SharedPtr
    mpc_observation_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<ocs2_msgs::msg::MpcObservation>::SharedPtr
    debug_observation_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    debug_input_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    debug_base_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr
    clock_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace g7_openarm_controllers
