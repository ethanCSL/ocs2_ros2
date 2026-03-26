#include "g7_openarm_controllers/g7_openarm_policy_controller.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <pluginlib/class_list_macros.hpp>

namespace
{
constexpr char kMobileBaseName[] = "mobile_base";
constexpr char kForwardVelocityInterface[] = "forward_velocity";
constexpr char kYawRateInterface[] = "yaw_rate";
constexpr char kBaseXInterface[] = "x";
constexpr char kBaseYInterface[] = "y";
constexpr char kBaseYawInterface[] = "yaw";
constexpr char kSimulationName[] = "simulation";
constexpr char kSimTimeStateInterface[] = "sim_time";
constexpr const char * kJointNames[] = {
  "L_1_joint", "L_2_joint", "L_3_joint", "L_4_joint", "L_5_joint", "L_6_joint", "L_7_joint",
  "R_1_joint", "R_2_joint", "R_3_joint", "R_4_joint", "R_5_joint", "R_6_joint", "R_7_joint"};

using ocs2::scalar_t;
using ocs2::size_array_t;

std::shared_ptr<g7_openarm_controllers::G7OpenarmPolicyController::PolicySnapshot>
parse_policy_message(const ocs2_msgs::msg::MpcFlattenedController & msg)
{
  auto snapshot =
    std::make_shared<g7_openarm_controllers::G7OpenarmPolicyController::PolicySnapshot>();

  snapshot->command.mpcInitObservation_ =
    ocs2::ros_msg_conversions::readObservationMsg(msg.init_observation);
  snapshot->command.mpcTargetTrajectories_ =
    ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(msg.plan_target_trajectories);
  snapshot->performance_indices =
    ocs2::ros_msg_conversions::readPerformanceIndicesMsg(msg.performance_indices);

  const size_t trajectory_size = msg.time_trajectory.size();
  if (trajectory_size == 0U)
  {
    throw std::runtime_error("Received empty MPC policy trajectory.");
  }
  if (msg.state_trajectory.size() != trajectory_size || msg.input_trajectory.size() != trajectory_size)
  {
    throw std::runtime_error("Policy state/input trajectory sizes do not match time trajectory.");
  }
  if (msg.data.size() != trajectory_size)
  {
    throw std::runtime_error("Flattened controller data size does not match time trajectory.");
  }

  snapshot->primal_solution.clear();
  snapshot->primal_solution.modeSchedule_ =
    ocs2::ros_msg_conversions::readModeScheduleMsg(msg.mode_schedule);

  size_array_t state_dimensions(trajectory_size);
  size_array_t input_dimensions(trajectory_size);
  snapshot->primal_solution.timeTrajectory_.reserve(trajectory_size);
  snapshot->primal_solution.stateTrajectory_.reserve(trajectory_size);
  snapshot->primal_solution.inputTrajectory_.reserve(trajectory_size);

  for (size_t i = 0; i < trajectory_size; ++i)
  {
    state_dimensions[i] = msg.state_trajectory[i].value.size();
    input_dimensions[i] = msg.input_trajectory[i].value.size();

    snapshot->primal_solution.timeTrajectory_.push_back(msg.time_trajectory[i]);
    snapshot->primal_solution.stateTrajectory_.emplace_back(
      Eigen::Map<const Eigen::VectorXf>(
        msg.state_trajectory[i].value.data(),
        static_cast<Eigen::Index>(state_dimensions[i]))
        .cast<scalar_t>());
    snapshot->primal_solution.inputTrajectory_.emplace_back(
      Eigen::Map<const Eigen::VectorXf>(
        msg.input_trajectory[i].value.data(),
        static_cast<Eigen::Index>(input_dimensions[i]))
        .cast<scalar_t>());
  }

  snapshot->primal_solution.postEventIndices_.reserve(msg.post_event_indices.size());
  for (const auto post_event_index : msg.post_event_indices)
  {
    snapshot->primal_solution.postEventIndices_.push_back(static_cast<size_t>(post_event_index));
  }

  std::vector<std::vector<float> const *> controller_data_ptrs(trajectory_size, nullptr);
  for (size_t i = 0; i < trajectory_size; ++i)
  {
    controller_data_ptrs[i] = &msg.data[i].data;
  }

  switch (msg.controller_type)
  {
    case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD:
    {
      auto controller = ocs2::FeedforwardController::unFlatten(
        snapshot->primal_solution.timeTrajectory_, controller_data_ptrs);
      snapshot->primal_solution.controllerPtr_.reset(
        new ocs2::FeedforwardController(std::move(controller)));
      break;
    }
    case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR:
    {
      auto controller = ocs2::LinearController::unFlatten(
        state_dimensions, input_dimensions, snapshot->primal_solution.timeTrajectory_,
        controller_data_ptrs);
      snapshot->primal_solution.controllerPtr_.reset(
        new ocs2::LinearController(std::move(controller)));
      break;
    }
    default:
      throw std::runtime_error("Unsupported OCS2 controller type in policy message.");
  }

  return snapshot;
}
}  // namespace

namespace g7_openarm_controllers
{

controller_interface::CallbackReturn G7OpenarmPolicyController::on_init()
{
  command_interface_names_.clear();
  command_interface_names_.emplace_back(
    std::string(kMobileBaseName) + "/" + kForwardVelocityInterface);
  command_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kYawRateInterface);

  state_interface_names_.clear();
  state_interface_names_.emplace_back(std::string(kSimulationName) + "/" + kSimTimeStateInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseXInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseYInterface);
  state_interface_names_.emplace_back(std::string(kMobileBaseName) + "/" + kBaseYawInterface);

  for (const auto * joint_name : kJointNames)
  {
    command_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interface_names_.emplace_back(
      std::string(joint_name) + "/" + hardware_interface::HW_IF_POSITION);
  }

  policy_buffer_.initRT(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
G7OpenarmPolicyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;
  return config;
}

controller_interface::InterfaceConfiguration
G7OpenarmPolicyController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}

controller_interface::CallbackReturn G7OpenarmPolicyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  policy_topic_ = auto_declare<std::string>("policy_topic", policy_topic_);
  policy_subscriber_ = get_node()->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
    policy_topic_, rclcpp::QoS(1),
    std::bind(&G7OpenarmPolicyController::policy_callback, this, std::placeholders::_1));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn G7OpenarmPolicyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_policy_.reset();
  zero_command_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn G7OpenarmPolicyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  zero_command_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type G7OpenarmPolicyController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (const auto * buffered_policy = policy_buffer_.readFromRT();
      buffered_policy != nullptr && *buffered_policy != nullptr)
  {
    active_policy_ = *buffered_policy;
  }

  if (active_policy_ == nullptr)
  {
    zero_command_interfaces();
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != state_interface_names_.size() ||
      command_interfaces_.size() != command_interface_names_.size())
  {
    zero_command_interfaces();
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Policy controller interface binding mismatch: got %zu state and %zu command interfaces.",
      state_interfaces_.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  const scalar_t current_time =
    state_interfaces_[find_state_interface_index(std::string(kSimulationName) + "/" + kSimTimeStateInterface)]
      .get_value();

  ocs2::vector_t current_state(static_cast<Eigen::Index>(state_interfaces_.size() - 1));
  for (size_t i = 1; i < state_interfaces_.size(); ++i)
  {
    current_state(static_cast<Eigen::Index>(i - 1)) = state_interfaces_[i].get_value();
  }
  try
  {
    const auto & policy = active_policy_->primal_solution;
    const ocs2::vector_t current_input =
      policy.controllerPtr_->computeInput(current_time, current_state);
    const auto nominal_state = ocs2::LinearInterpolation::interpolate(
      current_time, policy.timeTrajectory_, policy.stateTrajectory_);
    (void)nominal_state;

    if (current_input.size() != static_cast<Eigen::Index>(command_interfaces_.size()))
    {
      zero_command_interfaces();
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Evaluated MPC input has dimension %ld but controller expects %zu commands.",
        current_input.size(), command_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      command_interfaces_[i].set_value(current_input(static_cast<Eigen::Index>(i)));
    }
  }
  catch (const std::exception & error)
  {
    zero_command_interfaces();
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Failed to evaluate current MPC policy: %s", error.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void G7OpenarmPolicyController::policy_callback(
  const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg)
{
  try
  {
    policy_buffer_.writeFromNonRT(parse_policy_message(*msg));
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to parse MPC policy message on %s: %s",
      policy_topic_.c_str(), error.what());
  }
}

void G7OpenarmPolicyController::zero_command_interfaces()
{
  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }
}

size_t G7OpenarmPolicyController::find_state_interface_index(const std::string & name) const
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
  g7_openarm_controllers::G7OpenarmPolicyController,
  controller_interface::ControllerInterface)
