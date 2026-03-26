#include "g7_openarm_controllers/g7_openarm_observation_broadcaster.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iterator>
#include <string>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace
{
constexpr char kMobileBaseName[] = "mobile_base";
constexpr char kForwardVelocityInterface[] = "forward_velocity";
constexpr char kYawRateInterface[] = "yaw_rate";
constexpr char kCommandForwardVelocityStateInterface[] = "command_forward_velocity";
constexpr char kCommandYawRateStateInterface[] = "command_yaw_rate";
constexpr char kBaseXInterface[] = "x";
constexpr char kBaseYInterface[] = "y";
constexpr char kBaseYawInterface[] = "yaw";
constexpr char kCommandVelocityStateInterface[] = "command_velocity";
constexpr char kSimulationName[] = "simulation";
constexpr char kSimTimeStateInterface[] = "sim_time";
constexpr const char * kCommandJointNames[] = {
  "L_1_joint", "L_2_joint", "L_3_joint", "L_4_joint", "L_5_joint", "L_6_joint", "L_7_joint",
  "R_1_joint", "R_2_joint", "R_3_joint", "R_4_joint", "R_5_joint", "R_6_joint", "R_7_joint"};
constexpr const char * kPassiveJointNames[] = {
  "AMR_FL_joint", "AMR_FLW_joint", "AMR_FR_joint", "AMR_FRW_joint",
  "AMR_RL_joint", "AMR_RLW_joint", "AMR_RR_joint", "AMR_RRW_joint",
  "gripper_LL_joint", "gripper_LR_joint", "gripper_RL_joint", "gripper_RR_joint"};
constexpr const char * kJointStateNames[] = {
  "AMR_FL_joint", "AMR_FLW_joint", "AMR_FR_joint", "AMR_FRW_joint",
  "AMR_RL_joint", "AMR_RLW_joint", "AMR_RR_joint", "AMR_RRW_joint",
  "L_1_joint", "L_2_joint", "L_3_joint", "L_4_joint", "L_5_joint", "L_6_joint", "L_7_joint",
  "gripper_LL_joint", "gripper_LR_joint",
  "R_1_joint", "R_2_joint", "R_3_joint", "R_4_joint", "R_5_joint", "R_6_joint", "R_7_joint",
  "gripper_RL_joint", "gripper_RR_joint"};
}  // namespace

namespace g7_openarm_controllers
{

controller_interface::CallbackReturn G7OpenarmObservationBroadcaster::on_init()
{
  state_interface_names_.clear();
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseXInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseYInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseYawInterface);
  state_interface_names_.emplace_back(
    std::string(kMobileBaseName) + "/" + kForwardVelocityInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kYawRateInterface);
  state_interface_names_.emplace_back(
    std::string(kMobileBaseName) + "/" + kCommandForwardVelocityStateInterface);
  state_interface_names_.emplace_back(
    std::string(kMobileBaseName) + "/" + kCommandYawRateStateInterface);
  state_interface_names_.emplace_back(std::string(kSimulationName) + "/" + kSimTimeStateInterface);
  for (const auto * joint_name : kCommandJointNames)
  {
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_POSITION);
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + kCommandVelocityStateInterface);
  }
  for (const auto * joint_name : kPassiveJointNames)
  {
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_POSITION);
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_VELOCITY);
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
  auto node = get_node();
  mpc_observation_topic_ = auto_declare<std::string>("mpc_observation_topic", mpc_observation_topic_);
  debug_observation_topic_ =
    auto_declare<std::string>("debug_observation_topic", debug_observation_topic_);
  debug_input_topic_ = auto_declare<std::string>("debug_input_topic", debug_input_topic_);
  debug_base_pose_topic_ =
    auto_declare<std::string>("debug_base_pose_topic", debug_base_pose_topic_);
  clock_topic_ = auto_declare<std::string>("clock_topic", clock_topic_);
  joint_states_topic_ = auto_declare<std::string>("joint_states_topic", joint_states_topic_);
  world_frame_ = auto_declare<std::string>("world_frame", world_frame_);
  base_frame_ = auto_declare<std::string>("base_frame", base_frame_);

  mpc_observation_publisher_ =
    node->create_publisher<ocs2_msgs::msg::MpcObservation>(mpc_observation_topic_, 10);
  debug_observation_publisher_ =
    node->create_publisher<ocs2_msgs::msg::MpcObservation>(debug_observation_topic_, 10);
  debug_input_publisher_ =
    node->create_publisher<std_msgs::msg::Float64MultiArray>(debug_input_topic_, 10);
  debug_base_pose_publisher_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(debug_base_pose_topic_, 10);
  clock_publisher_ =
    node->create_publisher<rosgraph_msgs::msg::Clock>(clock_topic_, rclcpp::ClockQoS());
  joint_state_publisher_ =
    node->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

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
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!mpc_observation_publisher_->is_activated())
  {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != state_interface_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Observation broadcaster interface binding mismatch: got %zu state interfaces, expected %zu.",
      state_interfaces_.size(), state_interface_names_.size());
    return controller_interface::return_type::ERROR;
  }

  const auto sim_time = state_interfaces_[find_state_interface_index(
      std::string(kSimulationName) + "/" + kSimTimeStateInterface)].get_value();

  ocs2_msgs::msg::MpcObservation observation_msg;
  observation_msg.time = sim_time;
  observation_msg.state.value.assign(17, 0.0f);
  observation_msg.input.value.assign(16, 0.0f);
  observation_msg.mode = 0;

  try
  {
    observation_msg.state.value[0] = static_cast<float>(
      state_interfaces_[find_state_interface_index(std::string(kMobileBaseName) + "/" + kBaseXInterface)]
        .get_value());
    observation_msg.state.value[1] = static_cast<float>(
      state_interfaces_[find_state_interface_index(std::string(kMobileBaseName) + "/" + kBaseYInterface)]
        .get_value());
    observation_msg.state.value[2] = static_cast<float>(
      state_interfaces_[find_state_interface_index(std::string(kMobileBaseName) + "/" + kBaseYawInterface)]
        .get_value());

    for (size_t joint_index = 0; joint_index < std::size(kCommandJointNames); ++joint_index)
    {
      const std::string joint_prefix = std::string(kCommandJointNames[joint_index]) + "/";
      observation_msg.state.value[3 + joint_index] = static_cast<float>(
        state_interfaces_[find_state_interface_index(joint_prefix + hardware_interface::HW_IF_POSITION)]
          .get_value());
    }

    observation_msg.input.value[0] = static_cast<float>(
      state_interfaces_[find_state_interface_index(
        std::string(kMobileBaseName) + "/" + kCommandForwardVelocityStateInterface)]
        .get_value());
    observation_msg.input.value[1] = static_cast<float>(
      state_interfaces_[find_state_interface_index(
        std::string(kMobileBaseName) + "/" + kCommandYawRateStateInterface)]
        .get_value());

    for (size_t joint_index = 0; joint_index < std::size(kCommandJointNames); ++joint_index)
    {
      const std::string joint_prefix = std::string(kCommandJointNames[joint_index]) + "/";
      observation_msg.input.value[2 + joint_index] = static_cast<float>(
        state_interfaces_[find_state_interface_index(joint_prefix + kCommandVelocityStateInterface)]
          .get_value());
    }
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Failed to assemble observation/debug messages: %s", error.what());
    return controller_interface::return_type::ERROR;
  }

  mpc_observation_publisher_->publish(observation_msg);
  debug_observation_publisher_->publish(observation_msg);

  std_msgs::msg::Float64MultiArray input_msg;
  input_msg.data.assign(observation_msg.input.value.begin(), observation_msg.input.value.end());
  debug_input_publisher_->publish(input_msg);

  const rclcpp::Time ros_sim_time(
    static_cast<int64_t>(std::llround(sim_time * 1.0e9)), RCL_ROS_TIME);

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros_sim_time;
  joint_state_msg.name.reserve(std::size(kJointStateNames));
  joint_state_msg.position.reserve(std::size(kJointStateNames));
  joint_state_msg.velocity.reserve(std::size(kJointStateNames));
  for (const auto * joint_name : kJointStateNames)
  {
    const std::string joint_prefix = std::string(joint_name) + "/";
    joint_state_msg.name.emplace_back(joint_name);
    joint_state_msg.position.emplace_back(
      state_interfaces_[find_state_interface_index(joint_prefix + hardware_interface::HW_IF_POSITION)]
        .get_value());
    joint_state_msg.velocity.emplace_back(
      state_interfaces_[find_state_interface_index(joint_prefix + hardware_interface::HW_IF_VELOCITY)]
        .get_value());
  }
  joint_state_publisher_->publish(joint_state_msg);

  geometry_msgs::msg::PoseStamped base_pose_msg;
  base_pose_msg.header.stamp = ros_sim_time;
  base_pose_msg.header.frame_id = world_frame_;
  base_pose_msg.pose.position.x = observation_msg.state.value[0];
  base_pose_msg.pose.position.y = observation_msg.state.value[1];
  base_pose_msg.pose.position.z = 0.0;
  const double half_yaw = static_cast<double>(observation_msg.state.value[2]) * 0.5;
  base_pose_msg.pose.orientation.x = 0.0;
  base_pose_msg.pose.orientation.y = 0.0;
  base_pose_msg.pose.orientation.z = std::sin(half_yaw);
  base_pose_msg.pose.orientation.w = std::cos(half_yaw);
  debug_base_pose_publisher_->publish(base_pose_msg);

  geometry_msgs::msg::TransformStamped root_tf_msg;
  root_tf_msg.header.stamp = ros_sim_time;
  root_tf_msg.header.frame_id = world_frame_;
  root_tf_msg.child_frame_id = base_frame_;
  root_tf_msg.transform.translation.x = observation_msg.state.value[0];
  root_tf_msg.transform.translation.y = observation_msg.state.value[1];
  root_tf_msg.transform.translation.z = 0.0;
  root_tf_msg.transform.rotation.x = 0.0;
  root_tf_msg.transform.rotation.y = 0.0;
  root_tf_msg.transform.rotation.z = std::sin(half_yaw);
  root_tf_msg.transform.rotation.w = std::cos(half_yaw);
  tf_broadcaster_->sendTransform(root_tf_msg);

  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = ros_sim_time;
  clock_publisher_->publish(clock_msg);

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
