#include <ocs2_core/Types.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/srv/reset.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using ocs2::scalar_array_t;
using ocs2::scalar_t;
using ocs2::vector_array_t;
using ocs2::vector_t;
using ocs2::mobile_manipulator::MobileManipulatorInterface;
using ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping;
using ocs2::PinocchioInterface;

class G7OpenarmMpcResetCoordinator final : public rclcpp::Node
{
public:
  G7OpenarmMpcResetCoordinator()
  : Node(
      "g7_openarm_mpc_reset_coordinator",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    task_file_ = get_parameter("taskFile").as_string();
    lib_folder_ = get_parameter("libFolder").as_string();
    urdf_file_ = get_parameter("urdfFile").as_string();
    observation_topic_ = get_parameter_or<std::string>(
      "observation_topic", "/mobile_manipulator_mpc_observation");
    reset_service_name_ = get_parameter_or<std::string>(
      "reset_service", "/mobile_manipulator_mpc_reset");

    interface_ = std::make_unique<MobileManipulatorInterface>(task_file_, lib_folder_, urdf_file_);
    pinocchio_interface_ = std::make_unique<PinocchioInterface>(interface_->getPinocchioInterface());
    pinocchio_mapping_ = std::make_unique<MobileManipulatorPinocchioMapping>(
      interface_->getManipulatorModelInfo());

    auto & model = pinocchio_interface_->getModel();
    ee_frame_id_ = model.getFrameId(interface_->getManipulatorModelInfo().eeFrame);
    ee_frame_1_id_ = model.getFrameId(interface_->getManipulatorModelInfo().eeFrame1);
    if (
      ee_frame_id_ == static_cast<pinocchio::FrameIndex>(-1) ||
      ee_frame_1_id_ == static_cast<pinocchio::FrameIndex>(-1))
    {
      throw std::runtime_error("Failed to resolve end-effector frames for MPC reset coordinator.");
    }

    reset_client_ = create_client<ocs2_msgs::srv::Reset>(reset_service_name_);
    observation_subscription_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
      observation_topic_, rclcpp::QoS(1),
      std::bind(&G7OpenarmMpcResetCoordinator::observation_callback, this, std::placeholders::_1));
  }

private:
  void observation_callback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
  {
    if (reset_requested_ || reset_completed_)
    {
      return;
    }

    if (msg->state.value.size() != 17U)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for a 17D observation before requesting MPC reset.");
      return;
    }

    if (!reset_client_->wait_for_service(std::chrono::milliseconds(50)))
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for MPC reset service %s ...", reset_service_name_.c_str());
      return;
    }

    auto request = std::make_shared<ocs2_msgs::srv::Reset::Request>();
    request->reset = true;
    request->target_trajectories = create_current_target_trajectories(*msg);
    reset_requested_ = true;

    reset_client_->async_send_request(
      request,
      [this](rclcpp::Client<ocs2_msgs::srv::Reset>::SharedFuture future)
      {
        try
        {
          reset_completed_ = future.get()->done;
          if (reset_completed_)
          {
            RCLCPP_INFO(get_logger(), "Initial MPC reset completed.");
          }
          else
          {
            reset_requested_ = false;
            RCLCPP_WARN(get_logger(), "MPC reset service responded with done=false. Retrying.");
          }
        }
        catch (const std::exception & error)
        {
          reset_requested_ = false;
          RCLCPP_ERROR(get_logger(), "Failed to complete MPC reset request: %s", error.what());
        }
      });

    RCLCPP_INFO(
      get_logger(), "Requesting initial MPC reset from current MuJoCo observation at t=%.3f.",
      msg->time);
  }

  ocs2_msgs::msg::MpcTargetTrajectories create_current_target_trajectories(
    const ocs2_msgs::msg::MpcObservation & observation_msg)
  {
    vector_t state(17);
    for (size_t i = 0; i < 17U; ++i)
    {
      state(static_cast<Eigen::Index>(i)) = observation_msg.state.value[i];
    }

    vector_t q_pinocchio = pinocchio_mapping_->getPinocchioJointPosition(state);
    auto & model = pinocchio_interface_->getModel();
    auto & data = pinocchio_interface_->getData();
    pinocchio::forwardKinematics(model, data, q_pinocchio);
    pinocchio::updateFramePlacements(model, data);

    const auto & left_frame = data.oMf[ee_frame_id_];
    const auto & right_frame = data.oMf[ee_frame_1_id_];
    const Eigen::Quaternion<scalar_t> left_orientation(left_frame.rotation());
    const Eigen::Quaternion<scalar_t> right_orientation(right_frame.rotation());

    vector_t target_state(14);
    target_state << left_frame.translation(),
      left_orientation.coeffs(),
      right_frame.translation(),
      right_orientation.coeffs();

    ocs2_msgs::msg::MpcTargetTrajectories target_trajectories_msg;
    target_trajectories_msg.time_trajectory.push_back(observation_msg.time);

    ocs2_msgs::msg::MpcState target_state_msg;
    target_state_msg.value.resize(14U);
    for (size_t i = 0; i < 14U; ++i)
    {
      target_state_msg.value[i] = static_cast<float>(target_state(static_cast<Eigen::Index>(i)));
    }
    target_trajectories_msg.state_trajectory.push_back(std::move(target_state_msg));

    ocs2_msgs::msg::MpcInput zero_input_msg;
    zero_input_msg.value.assign(
      static_cast<size_t>(interface_->getManipulatorModelInfo().inputDim), 0.0f);
    target_trajectories_msg.input_trajectory.push_back(std::move(zero_input_msg));

    return target_trajectories_msg;
  }

  std::string task_file_;
  std::string lib_folder_;
  std::string urdf_file_;
  std::string observation_topic_;
  std::string reset_service_name_;

  bool reset_requested_ = false;
  bool reset_completed_ = false;

  std::unique_ptr<MobileManipulatorInterface> interface_;
  std::unique_ptr<PinocchioInterface> pinocchio_interface_;
  std::unique_ptr<MobileManipulatorPinocchioMapping> pinocchio_mapping_;
  pinocchio::FrameIndex ee_frame_id_{};
  pinocchio::FrameIndex ee_frame_1_id_{};

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_subscription_;
  rclcpp::Client<ocs2_msgs::srv::Reset>::SharedPtr reset_client_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G7OpenarmMpcResetCoordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
