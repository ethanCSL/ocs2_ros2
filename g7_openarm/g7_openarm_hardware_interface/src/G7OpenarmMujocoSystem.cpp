#include "g7_openarm_hardware_interface/G7OpenarmMujocoSystem.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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

constexpr std::array<const char *, 3> kBaseJointNames = {
  "base_x_joint",
  "base_y_joint",
  "base_yaw_joint",
};

constexpr std::array<const char *, 3> kBaseActuatorNames = {
  "base_x_joint_act",
  "base_y_joint_act",
  "base_yaw_joint_act",
};

constexpr std::array<const char *, 14> kArmJointNames = {
  "L_1_joint", "L_2_joint", "L_3_joint", "L_4_joint", "L_5_joint", "L_6_joint", "L_7_joint",
  "R_1_joint", "R_2_joint", "R_3_joint", "R_4_joint", "R_5_joint", "R_6_joint", "R_7_joint",
};

constexpr std::array<const char *, 14> kArmActuatorNames = {
  "L_1_joint_act", "L_2_joint_act", "L_3_joint_act", "L_4_joint_act", "L_5_joint_act",
  "L_6_joint_act", "L_7_joint_act", "R_1_joint_act", "R_2_joint_act", "R_3_joint_act",
  "R_4_joint_act", "R_5_joint_act", "R_6_joint_act", "R_7_joint_act",
};

#ifdef MUJOCO_PLUGIN_DIR
constexpr const char * kMujocoPluginDir = MUJOCO_PLUGIN_DIR;
#else
constexpr const char * kMujocoPluginDir = "";
#endif
}  // namespace

namespace g7_openarm_hardware_interface
{

G7OpenarmMujocoSystem::~G7OpenarmMujocoSystem()
{
  if (data_ != nullptr)
  {
    mj_deleteData(data_);
  }
  if (model_ != nullptr)
  {
    mj_deleteModel(model_);
  }
}

hardware_interface::CallbackReturn G7OpenarmMujocoSystem::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints)
  {
    JointData joint_data;
    joint_data.name = joint.name;
    joint_data.commandable = !joint.command_interfaces.empty();
    joints_.push_back(std::move(joint_data));
  }

  base_x_ = 0.0;
  base_y_ = 0.0;
  base_yaw_ = 0.0;
  base_forward_velocity_state_ = 0.0;
  base_yaw_rate_state_ = 0.0;
  base_forward_velocity_command_ = 0.0;
  base_yaw_rate_command_ = 0.0;
  base_forward_velocity_command_state_ = 0.0;
  base_yaw_rate_command_state_ = 0.0;
  sim_time_state_ = 0.0;

  mjcf_file_ = get_hardware_parameter("mjcf_file", true);
  dt_sim_ = get_hardware_parameter("dt_sim", 0.001, false);
  gains_.base_linear_kv = get_hardware_parameter("base_linear_kv", 200.0, false);
  gains_.base_yaw_kv = get_hardware_parameter("base_yaw_kv", 200.0, false);
  gains_.arm_joint_kv = get_hardware_parameter("arm_joint_kv", 2.33, false);

  if (dt_sim_ <= 0.0)
  {
    throw std::runtime_error("dt_sim must be positive.");
  }

  load_mujoco_plugins();
  load_mujoco_model();
  validate_joint_configuration();
  cache_model_handles();
  reset_simulation_state();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> G7OpenarmMujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(8 + joints_.size() * 3);

  interfaces.emplace_back(kMobileBaseName, kBaseXInterface, &base_x_);
  interfaces.emplace_back(kMobileBaseName, kBaseYInterface, &base_y_);
  interfaces.emplace_back(kMobileBaseName, kBaseYawInterface, &base_yaw_);
  interfaces.emplace_back(
    kMobileBaseName, kForwardVelocityInterface, &base_forward_velocity_state_);
  interfaces.emplace_back(kMobileBaseName, kYawRateInterface, &base_yaw_rate_state_);
  interfaces.emplace_back(
    kMobileBaseName, kCommandForwardVelocityStateInterface,
    &base_forward_velocity_command_state_);
  interfaces.emplace_back(
    kMobileBaseName, kCommandYawRateStateInterface, &base_yaw_rate_command_state_);
  interfaces.emplace_back(kSimulationName, kSimTimeStateInterface, &sim_time_state_);

  for (auto & joint : joints_)
  {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
    if (joint.commandable)
    {
      interfaces.emplace_back(
        joint.name, kCommandVelocityStateInterface, &joint.velocity_command_state);
    }
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> G7OpenarmMujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(2 + joints_.size());

  interfaces.emplace_back(
    kMobileBaseName, kForwardVelocityInterface, &base_forward_velocity_command_);
  interfaces.emplace_back(kMobileBaseName, kYawRateInterface, &base_yaw_rate_command_);

  for (auto & joint : joints_)
  {
    if (joint.commandable)
    {
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_command);
    }
  }

  return interfaces;
}

hardware_interface::return_type G7OpenarmMujocoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (period.seconds() > 0.0)
  {
    sim_substeps_hint_ = std::max(1, static_cast<int>(std::llround(period.seconds() / dt_sim_)));
  }

  update_state_from_mujoco();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type G7OpenarmMujocoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (model_ == nullptr || data_ == nullptr)
  {
    return hardware_interface::return_type::ERROR;
  }

  const int substeps = period.seconds() > 0.0 ?
    std::max(1, static_cast<int>(std::llround(period.seconds() / dt_sim_))) :
    sim_substeps_hint_;

  apply_velocity_servo();
  for (int i = 0; i < substeps; ++i)
  {
    mj_step(model_, data_);
  }
  sim_time_state_ += static_cast<double>(substeps) * dt_sim_;
  return hardware_interface::return_type::OK;
}

void G7OpenarmMujocoSystem::load_mujoco_plugins() const
{
  static const bool plugins_loaded = [this]()
  {
    if (std::string(kMujocoPluginDir).empty())
    {
      return false;
    }
    mj_loadAllPluginLibraries(kMujocoPluginDir, +[](const char *, int, int) {});
    return true;
  }();

  (void)plugins_loaded;
}

void G7OpenarmMujocoSystem::load_mujoco_model()
{
  std::array<char, 1024> error{};
  model_ = mj_loadXML(mjcf_file_.c_str(), nullptr, error.data(), error.size());
  if (model_ == nullptr)
  {
    throw std::runtime_error("Failed to load MJCF: " + std::string(error.data()));
  }

  data_ = mj_makeData(model_);
  if (data_ == nullptr)
  {
    throw std::runtime_error("Failed to allocate MuJoCo data.");
  }

  model_->opt.timestep = dt_sim_;
  mj_forward(model_, data_);
}

void G7OpenarmMujocoSystem::validate_joint_configuration() const
{
  size_t commandable_index = 0;
  for (const auto & joint : joints_)
  {
    if (!joint.commandable)
    {
      continue;
    }
    if (commandable_index >= kArmJointNames.size())
    {
      throw std::runtime_error("More commandable joints than expected in g7_openarm hardware.");
    }
    if (joint.name != kArmJointNames[commandable_index])
    {
      throw std::runtime_error(
        "Commandable joint ordering mismatch in g7_openarm hardware: expected " +
        std::string(kArmJointNames[commandable_index]) + ", got " + joint.name);
    }
    ++commandable_index;
  }

  if (commandable_index != kArmJointNames.size())
  {
    throw std::runtime_error("Did not find all 14 commandable arm joints in g7_openarm hardware.");
  }
}

void G7OpenarmMujocoSystem::cache_model_handles()
{
  for (size_t i = 0; i < kBaseJointNames.size(); ++i)
  {
    cache_joint_and_actuator(
      kBaseJointNames[i], kBaseActuatorNames[i], base_joint_ids_[i], base_qpos_addresses_[i],
      base_qvel_addresses_[i], base_actuator_ids_[i]);
  }

  for (size_t i = 0; i < kArmJointNames.size(); ++i)
  {
    cache_joint_and_actuator(
      kArmJointNames[i], kArmActuatorNames[i], arm_joint_ids_[i], arm_qpos_addresses_[i],
      arm_qvel_addresses_[i], arm_actuator_ids_[i]);
  }
}

void G7OpenarmMujocoSystem::cache_joint_and_actuator(
  const char * joint_name, const char * actuator_name, int & joint_id, int & qpos_address,
  int & qvel_address, int & actuator_id) const
{
  joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_name);
  actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name);
  if (joint_id < 0 || actuator_id < 0)
  {
    throw std::runtime_error(
      "Failed to resolve MuJoCo joint or actuator: " + std::string(joint_name));
  }

  qpos_address = model_->jnt_qposadr[joint_id];
  qvel_address = model_->jnt_dofadr[joint_id];
}

void G7OpenarmMujocoSystem::reset_simulation_state()
{
  const int home_key_id = mj_name2id(model_, mjOBJ_KEY, "home");
  if (home_key_id >= 0)
  {
    mj_resetDataKeyframe(model_, data_, home_key_id);
  }
  else
  {
    mj_resetData(model_, data_);
  }

  model_->opt.timestep = dt_sim_;
  mju_zero(data_->ctrl, model_->nu);
  base_forward_velocity_command_ = 0.0;
  base_yaw_rate_command_ = 0.0;
  base_forward_velocity_command_state_ = 0.0;
  base_yaw_rate_command_state_ = 0.0;
  for (auto & joint : joints_)
  {
    if (joint.commandable)
    {
      joint.velocity_command = 0.0;
      joint.velocity_command_state = 0.0;
    }
    else
    {
      joint.position = 0.0;
      joint.velocity = 0.0;
    }
  }
  mj_forward(model_, data_);
  update_state_from_mujoco();
}

void G7OpenarmMujocoSystem::update_state_from_mujoco()
{
  base_x_ = data_->qpos[base_qpos_addresses_[0]];
  base_y_ = data_->qpos[base_qpos_addresses_[1]];
  base_yaw_ = data_->qpos[base_qpos_addresses_[2]];

  const double world_x_dot = data_->qvel[base_qvel_addresses_[0]];
  const double world_y_dot = data_->qvel[base_qvel_addresses_[1]];
  base_forward_velocity_state_ =
    std::cos(base_yaw_) * world_x_dot + std::sin(base_yaw_) * world_y_dot;
  base_yaw_rate_state_ = data_->qvel[base_qvel_addresses_[2]];

  size_t command_joint_index = 0;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (joints_[i].commandable)
    {
      if (command_joint_index >= arm_qpos_addresses_.size())
      {
        throw std::runtime_error("Command joint index exceeded cached MuJoCo arm handles.");
      }
      joints_[i].position = data_->qpos[arm_qpos_addresses_[command_joint_index]];
      joints_[i].velocity = data_->qvel[arm_qvel_addresses_[command_joint_index]];
      ++command_joint_index;
    }
    else
    {
      joints_[i].position = 0.0;
      joints_[i].velocity = 0.0;
    }
  }
}

void G7OpenarmMujocoSystem::apply_velocity_servo()
{
  mju_zero(data_->ctrl, model_->nu);

  base_forward_velocity_command_state_ = base_forward_velocity_command_;
  base_yaw_rate_command_state_ = base_yaw_rate_command_;
  for (auto & joint : joints_)
  {
    if (joint.commandable)
    {
      joint.velocity_command_state = joint.velocity_command;
    }
  }

  const double target_base_x_dot = std::cos(base_yaw_) * base_forward_velocity_command_;
  const double target_base_y_dot = std::sin(base_yaw_) * base_forward_velocity_command_;

  set_actuator_control(
    base_actuator_ids_[0],
    gains_.base_linear_kv * (target_base_x_dot - data_->qvel[base_qvel_addresses_[0]]));
  set_actuator_control(
    base_actuator_ids_[1],
    gains_.base_linear_kv * (target_base_y_dot - data_->qvel[base_qvel_addresses_[1]]));
  set_actuator_control(
    base_actuator_ids_[2],
    gains_.base_yaw_kv * (base_yaw_rate_command_ - data_->qvel[base_qvel_addresses_[2]]));

  size_t command_joint_index = 0;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i].commandable)
    {
      continue;
    }
    if (command_joint_index >= arm_actuator_ids_.size())
    {
      break;
    }
    const double joint_velocity_error =
      joints_[i].velocity_command - data_->qvel[arm_qvel_addresses_[command_joint_index]];
    set_actuator_control(
      arm_actuator_ids_[command_joint_index], gains_.arm_joint_kv * joint_velocity_error);
    ++command_joint_index;
  }
}

void G7OpenarmMujocoSystem::set_actuator_control(int actuator_id, double value)
{
  double clamped_value = value;
  if (model_->actuator_ctrllimited[actuator_id] != 0)
  {
    const double lower = model_->actuator_ctrlrange[2 * actuator_id];
    const double upper = model_->actuator_ctrlrange[2 * actuator_id + 1];
    clamped_value = std::clamp(clamped_value, lower, upper);
  }
  data_->ctrl[actuator_id] = clamped_value;
}

double G7OpenarmMujocoSystem::get_hardware_parameter(
  const std::string & key, double default_value, bool required) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end())
  {
    if (required)
    {
      throw std::runtime_error("Missing required hardware parameter: " + key);
    }
    return default_value;
  }

  return std::stod(it->second);
}

std::string G7OpenarmMujocoSystem::get_hardware_parameter(
  const std::string & key, bool required) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end())
  {
    if (required)
    {
      throw std::runtime_error("Missing required hardware parameter: " + key);
    }
    return {};
  }

  return it->second;
}

}  // namespace g7_openarm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  g7_openarm_hardware_interface::G7OpenarmMujocoSystem,
  hardware_interface::SystemInterface)
