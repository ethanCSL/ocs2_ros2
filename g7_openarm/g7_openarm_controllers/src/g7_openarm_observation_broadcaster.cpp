#include "g7_openarm_controllers/g7_openarm_observation_broadcaster.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
constexpr char kMobileBaseName[] = "mobile_base";
constexpr char kBaseXInterface[] = "x";
constexpr char kBaseYInterface[] = "y";
constexpr char kBaseYawInterface[] = "yaw";
constexpr char kCommandForwardVelocityStateInterface[] = "command_forward_velocity";
constexpr char kCommandYawRateStateInterface[] = "command_yaw_rate";
constexpr char kSimulationName[] = "simulation";
constexpr char kSimTimeStateInterface[] = "sim_time";
constexpr char kCommandVelocityStateInterface[] = "command_velocity";
constexpr const char * kJointNames[] = {
  "L_1_joint", "L_2_joint", "L_3_joint", "L_4_joint", "L_5_joint", "L_6_joint", "L_7_joint",
  "R_1_joint", "R_2_joint", "R_3_joint", "R_4_joint", "R_5_joint", "R_6_joint", "R_7_joint"};

constexpr size_t kStateDimension = 17;
constexpr size_t kInputDimension = 16;
}  // namespace

namespace g7_openarm_controllers
{

controller_interface::CallbackReturn G7OpenarmObservationBroadcaster::on_init()
{
  state_interface_names_.clear();
  state_interface_names_.emplace_back(std::string(kSimulationName) + "/" + kSimTimeStateInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseXInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseYInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseYawInterface);
  state_interface_names_.emplace_back(
    std::string(kMobileBaseName) + "/" + kCommandForwardVelocityStateInterface);
  state_interface_names_.emplace_back(
    std::string(kMobileBaseName) + "/" + kCommandYawRateStateInterface);

  for (const auto * joint_name : kJointNames)
  {
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_POSITION);
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + kCommandVelocityStateInterface);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
G7OpenarmObservationBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
G7OpenarmObservationBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}

controller_interface::CallbackReturn G7OpenarmObservationBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  mpc_observation_topic_ = auto_declare<std::string>("mpc_observation_topic", mpc_observation_topic_);
  debug_observation_topic_ =
    auto_declare<std::string>("debug_observation_topic", debug_observation_topic_);
  debug_input_topic_ = auto_declare<std::string>("debug_input_topic", debug_input_topic_);
  debug_base_pose_topic_ = auto_declare<std::string>("debug_base_pose_topic", debug_base_pose_topic_);
  clock_topic_ = auto_declare<std::string>("clock_topic", clock_topic_);
  joint_states_topic_ = auto_declare<std::string>("joint_states_topic", joint_states_topic_);
  world_frame_ = auto_declare<std::string>("world_frame", world_frame_);
  base_frame_ = auto_declare<std::string>("base_frame", base_frame_);

  mpc_observation_publisher_ =
    get_node()->create_publisher<ocs2_msgs::msg::MpcObservation>(mpc_observation_topic_, rclcpp::QoS(1));
  debug_observation_publisher_ =
    get_node()->create_publisher<ocs2_msgs::msg::MpcObservation>(debug_observation_topic_, rclcpp::QoS(1));
  debug_input_publisher_ =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(debug_input_topic_, rclcpp::QoS(1));
  debug_base_pose_publisher_ =
    get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(debug_base_pose_topic_, rclcpp::QoS(1));
  clock_publisher_ =
    get_node()->create_publisher<rosgraph_msgs::msg::Clock>(clock_topic_, rclcpp::QoS(1));
  joint_state_publisher_ =
    get_node()->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn G7OpenarmObservationBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  mpc_observation_publisher_->on_activate();
  debug_observation_publisher_->on_activate();
  debug_input_publisher_->on_activate();
  debug_base_pose_publisher_->on_activate();
  clock_publisher_->on_activate();
  joint_state_publisher_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn G7OpenarmObservationBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  mpc_observation_publisher_->on_deactivate();
  debug_observation_publisher_->on_deactivate();
  debug_input_publisher_->on_deactivate();
  debug_base_pose_publisher_->on_deactivate();
  clock_publisher_->on_deactivate();
  joint_state_publisher_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type G7OpenarmObservationBroadcaster::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (state_interfaces_.size() != state_interface_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Observation broadcaster interface binding mismatch: got %zu state interfaces, expected %zu.",
      state_interfaces_.size(), state_interface_names_.size());
    return controller_interface::return_type::ERROR;
  }

  const double sim_time =
    state_interfaces_[find_state_interface_index(std::string(kSimulationName) + "/" + kSimTimeStateInterface)]
      .get_value();
  const double base_x =
    state_interfaces_[find_state_interface_index(std::string(kMobileBaseName) + "/" + kBaseXInterface)]
      .get_value();
  const double base_y =
    state_interfaces_[find_state_interface_index(std::string(kMobileBaseName) + "/" + kBaseYInterface)]
      .get_value();
  const double base_yaw =
    state_interfaces_[find_state_interface_index(std::string(kMobileBaseName) + "/" + kBaseYawInterface)]
      .get_value();
  const double base_forward_cmd =
    state_interfaces_[find_state_interface_index(
      std::string(kMobileBaseName) + "/" + kCommandForwardVelocityStateInterface)]
      .get_value();
  const double base_yaw_cmd =
    state_interfaces_[find_state_interface_index(
      std::string(kMobileBaseName) + "/" + kCommandYawRateStateInterface)]
      .get_value();

  ocs2_msgs::msg::MpcObservation observation_msg;
  observation_msg.time = sim_time;
  observation_msg.mode = 0;
  observation_msg.state.value.resize(kStateDimension);
  observation_msg.input.value.resize(kInputDimension);
  observation_msg.state.value[0] = static_cast<float>(base_x);
  observation_msg.state.value[1] = static_cast<float>(base_y);
  observation_msg.state.value[2] = static_cast<float>(base_yaw);
  observation_msg.input.value[0] = static_cast<float>(base_forward_cmd);
  observation_msg.input.value[1] = static_cast<float>(base_yaw_cmd);

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sim_time * 1.0e9), RCL_ROS_TIME);
  joint_state_msg.name.reserve(std::size(kJointNames));
  joint_state_msg.position.reserve(std::size(kJointNames));
  joint_state_msg.velocity.reserve(std::size(kJointNames));

  std_msgs::msg::Float64MultiArray debug_input_msg;
  debug_input_msg.data.resize(kInputDimension);
  debug_input_msg.data[0] = base_forward_cmd;
  debug_input_msg.data[1] = base_yaw_cmd;

  for (size_t i = 0; i < std::size(kJointNames); ++i)
  {
    const auto joint_prefix = std::string(kJointNames[i]) + "/";
    const double joint_position =
      state_interfaces_[find_state_interface_index(joint_prefix + hardware_interface::HW_IF_POSITION)]
        .get_value();
    const double joint_velocity =
      state_interfaces_[find_state_interface_index(joint_prefix + hardware_interface::HW_IF_VELOCITY)]
        .get_value();
    const double joint_command =
      state_interfaces_[find_state_interface_index(joint_prefix + kCommandVelocityStateInterface)]
        .get_value();

    observation_msg.state.value[i + 3] = static_cast<float>(joint_position);
    observation_msg.input.value[i + 2] = static_cast<float>(joint_command);

    joint_state_msg.name.emplace_back(kJointNames[i]);
    joint_state_msg.position.push_back(joint_position);
    joint_state_msg.velocity.push_back(joint_velocity);
    debug_input_msg.data[i + 2] = joint_command;
  }

  geometry_msgs::msg::PoseStamped base_pose_msg;
  base_pose_msg.header.frame_id = world_frame_;
  base_pose_msg.header.stamp = joint_state_msg.header.stamp;
  base_pose_msg.pose.position.x = base_x;
  base_pose_msg.pose.position.y = base_y;
  base_pose_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, base_yaw);
  base_pose_msg.pose.orientation.x = q.x();
  base_pose_msg.pose.orientation.y = q.y();
  base_pose_msg.pose.orientation.z = q.z();
  base_pose_msg.pose.orientation.w = q.w();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = base_pose_msg.header;
  tf_msg.child_frame_id = base_frame_;
  tf_msg.transform.translation.x = base_x;
  tf_msg.transform.translation.y = base_y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = base_pose_msg.pose.orientation;

  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = joint_state_msg.header.stamp;

  mpc_observation_publisher_->publish(observation_msg);
  debug_observation_publisher_->publish(observation_msg);
  debug_input_publisher_->publish(debug_input_msg);
  debug_base_pose_publisher_->publish(base_pose_msg);
  clock_publisher_->publish(clock_msg);
  joint_state_publisher_->publish(joint_state_msg);
  tf_broadcaster_->sendTransform(tf_msg);

  return controller_interface::return_type::OK;
}

size_t G7OpenarmObservationBroadcaster::find_state_interface_index(const std::string & name) const
{
  const auto interface_it = std::find(state_interface_names_.begin(), state_interface_names_.end(), name);
  if (interface_it == state_interface_names_.end())
  {
    throw std::runtime_error("Missing expected state interface: " + name);
  }
  return static_cast<size_t>(std::distance(state_interface_names_.begin(), interface_it));
}

}  // namespace g7_openarm_controllers

PLUGINLIB_EXPORT_CLASS(
  g7_openarm_controllers::G7OpenarmObservationBroadcaster,
  controller_interface::ControllerInterface)
