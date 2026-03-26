#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace g7_openarm_controllers
{

class G7OpenarmPolicyController : public controller_interface::ControllerInterface
{
public:
  struct PolicySnapshot
  {
    ocs2::CommandData command;
    ocs2::PrimalSolution primal_solution;
    ocs2::PerformanceIndex performance_indices;
  };

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
  void policy_callback(const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg);
  size_t find_state_interface_index(const std::string & name) const;
  void zero_command_interfaces();

  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;
  std::string policy_topic_ = "/mobile_manipulator_mpc_policy";
  rclcpp::Subscription<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr policy_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<PolicySnapshot>> policy_buffer_;
  std::shared_ptr<PolicySnapshot> active_policy_;
};

}  // namespace g7_openarm_controllers
