#pragma once

#include <array>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>

namespace g7_openarm_hardware_interface
{

class G7OpenarmMujocoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(G7OpenarmMujocoSystem)

  ~G7OpenarmMujocoSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointData
  {
    std::string name;
    bool commandable = false;
    double position = 0.0;
    double velocity = 0.0;
    double velocity_command = 0.0;
    double velocity_command_state = 0.0;
  };

  struct ServoGains
  {
    double base_linear_kv = 0.0;
    double base_yaw_kv = 0.0;
    double arm_joint_kv = 2.33;
  };

  void load_mujoco_plugins() const;
  void load_mujoco_model();
  void validate_joint_configuration() const;
  void cache_model_handles();
  void cache_joint_and_actuator(
    const char * joint_name, const char * actuator_name,
    int & joint_id, int & qpos_address, int & qvel_address, int & actuator_id) const;
  void reset_simulation_state();
  void update_state_from_mujoco();
  void apply_velocity_servo();
  void set_actuator_control(int actuator_id, double value);
  double get_hardware_parameter(
    const std::string & key, double default_value, bool required = false) const;
  std::string get_hardware_parameter(const std::string & key, bool required = false) const;

  double base_x_ = 0.0;
  double base_y_ = 0.0;
  double base_yaw_ = 0.0;
  double base_forward_velocity_state_ = 0.0;
  double base_yaw_rate_state_ = 0.0;
  double base_forward_velocity_command_ = 0.0;
  double base_yaw_rate_command_ = 0.0;
  double base_forward_velocity_command_state_ = 0.0;
  double base_yaw_rate_command_state_ = 0.0;
  double sim_time_state_ = 0.0;

  std::vector<JointData> joints_;
  std::string mjcf_file_;
  double dt_sim_ = 0.001;
  int sim_substeps_hint_ = 1;
  ServoGains gains_;

  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;

  std::array<int, 3> base_joint_ids_{};
  std::array<int, 3> base_qpos_addresses_{};
  std::array<int, 3> base_qvel_addresses_{};
  std::array<int, 3> base_actuator_ids_{};
  std::array<int, 14> arm_joint_ids_{};
  std::array<int, 14> arm_qpos_addresses_{};
  std::array<int, 14> arm_qvel_addresses_{};
  std::array<int, 14> arm_actuator_ids_{};
};

}  // namespace g7_openarm_hardware_interface
