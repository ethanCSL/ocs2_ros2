#include <mujoco/mujoco.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using ocs2::CommandData;
using ocs2::PinocchioInterface;
using ocs2::PrimalSolution;
using ocs2::SystemObservation;
using ocs2::TargetTrajectories;
using ocs2::scalar_array_t;
using ocs2::scalar_t;
using ocs2::vector_array_t;
using ocs2::vector_t;
using ocs2::mobile_manipulator::ManipulatorModelInfo;
using ocs2::mobile_manipulator::MobileManipulatorDummyVisualization;
using ocs2::mobile_manipulator::MobileManipulatorInterface;
using ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping;

constexpr const char* kRobotName = "mobile_manipulator";
constexpr std::array<const char*, 3> kBaseJointNames = {
    "base_x_joint",
    "base_y_joint",
    "base_yaw_joint",
};
constexpr std::array<const char*, 3> kBaseActuatorNames = {
    "base_x_joint_act",
    "base_y_joint_act",
    "base_yaw_joint_act",
};
constexpr std::array<const char*, 14> kArmJointNames = {
    "L_1_joint",
    "L_2_joint",
    "L_3_joint",
    "L_4_joint",
    "L_5_joint",
    "L_6_joint",
    "L_7_joint",
    "R_1_joint",
    "R_2_joint",
    "R_3_joint",
    "R_4_joint",
    "R_5_joint",
    "R_6_joint",
    "R_7_joint",
};
constexpr std::array<const char*, 14> kArmActuatorNames = {
    "L_1_joint_act",
    "L_2_joint_act",
    "L_3_joint_act",
    "L_4_joint_act",
    "L_5_joint_act",
    "L_6_joint_act",
    "L_7_joint_act",
    "R_1_joint_act",
    "R_2_joint_act",
    "R_3_joint_act",
    "R_4_joint_act",
    "R_5_joint_act",
    "R_6_joint_act",
    "R_7_joint_act",
};
#ifdef MUJOCO_PLUGIN_DIR
constexpr const char* kMujocoPluginDir = MUJOCO_PLUGIN_DIR;
#else
constexpr const char* kMujocoPluginDir = "";
#endif

struct ServoGains
{
    scalar_t baseLinearKv = 0.0;
    scalar_t baseYawKv = 0.0;
    scalar_t armJointKv = 2.33;
};

class G7OpenarmMujocoMrtNode
{
public:
    G7OpenarmMujocoMrtNode()
        : node_(rclcpp::Node::make_shared(
            "g7_openarm_mujoco_mrt",
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true))),
          taskFile_(node_->get_parameter("taskFile").as_string()),
          libFolder_(node_->get_parameter("libFolder").as_string()),
          urdfFile_(node_->get_parameter("urdfFile").as_string()),
          mjcfFile_(node_->get_parameter("mjcfFile").as_string()),
          dtSim_(node_->get_parameter("dtSim").as_double()),
          dtCtrl_(node_->get_parameter("dtCtrl").as_double()),
          interface_(taskFile_, libFolder_, urdfFile_),
          mrt_(kRobotName),
          visualization_(std::make_shared<MobileManipulatorDummyVisualization>(node_, interface_)),
          pinocchioInterface_(interface_.getPinocchioInterface()),
          pinocchioMapping_(interface_.getManipulatorModelInfo())
    {
        gains_.baseLinearKv = node_->get_parameter("baseLinearKv").as_double();
        gains_.baseYawKv = node_->get_parameter("baseYawKv").as_double();
        gains_.armJointKv = node_->get_parameter("armJointKv").as_double();

        validateTiming();
        validateModelInfo();
        loadInputLimits();
        loadMujocoPlugins();
        loadMujocoModel();
        cacheModelHandles();

        mrt_.launchNodes(node_);
        clockPublisher_ =
            node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
        debugObservationPublisher_ =
            node_->create_publisher<ocs2_msgs::msg::MpcObservation>("/mujoco/debug_observation", 10);
        debugInputPublisher_ =
            node_->create_publisher<std_msgs::msg::Float64MultiArray>("/mujoco/debug_input", 10);
        debugBasePosePublisher_ =
            node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mujoco/debug_base_pose", 10);

        resetService_ = node_->create_service<std_srvs::srv::Trigger>(
            "mujoco_reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                std::scoped_lock<std::mutex> lock(resetMutex_);
                pendingReset_ = true;
                response->success = true;
                response->message = "MuJoCo reset scheduled.";
            });

        performReset();
    }

    ~G7OpenarmMujocoMrtNode()
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

    void run()
    {
        while (rclcpp::ok())
        {
            if (consumeResetRequest())
            {
                performReset();
                continue;
            }

            publishObservationAndWaitForPolicy(currentObservation_);

            vector_t optimizedState;
            vector_t optimizedInput;
            size_t plannedMode = currentObservation_.mode;
            mrt_.evaluatePolicy(
                currentObservation_.time,
                currentObservation_.state,
                optimizedState,
                optimizedInput,
                plannedMode);

            applyVelocityServo(optimizedInput, currentObservation_.state(2));

            for (int i = 0; i < simSubsteps_; ++i)
            {
                mj_step(model_, data_);
            }

            lastAppliedInput_ = optimizedInput;
            currentObservation_ =
                buildObservation(currentObservation_.time + dtCtrl_, plannedMode);
            recordSimulationTime(currentObservation_.time);
            publishClock(currentObservation_.time);
            publishDebugTopics(currentObservation_);
            ++controlCycleCount_;
            if (controlCycleCount_ % 100 == 0)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Closed-loop running: sim_time=%.3f, base=(%.3f, %.3f, %.3f), u_base=(%.3f, %.3f)",
                    currentObservation_.time,
                    currentObservation_.state(0),
                    currentObservation_.state(1),
                    currentObservation_.state(2),
                    lastServoInput_(0),
                    lastServoInput_(1));
            }
            visualization_->update(currentObservation_, mrt_.getPolicy(), mrt_.getCommand());
            mrt_.spinMRT();
        }
    }

private:
    rclcpp::Time toRosTime(scalar_t simulationTime) const
    {
        const auto nanoseconds = static_cast<int64_t>(std::llround(simulationTime * 1.0e9));
        return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
    }

    void publishClock(scalar_t simulationTime) const
    {
        rosgraph_msgs::msg::Clock clockMsg;
        clockMsg.clock = toRosTime(simulationTime);
        clockPublisher_->publish(clockMsg);
    }

    void publishDebugTopics(const SystemObservation& observation) const
    {
        debugObservationPublisher_->publish(
            ocs2::ros_msg_conversions::createObservationMsg(observation));

        std_msgs::msg::Float64MultiArray inputMsg;
        inputMsg.data.resize(static_cast<size_t>(lastServoInput_.size()));
        for (Eigen::Index i = 0; i < lastServoInput_.size(); ++i)
        {
            inputMsg.data[static_cast<size_t>(i)] = lastServoInput_(i);
        }
        debugInputPublisher_->publish(inputMsg);

        geometry_msgs::msg::PoseStamped basePoseMsg;
        basePoseMsg.header.stamp = toRosTime(observation.time);
        basePoseMsg.header.frame_id = "world";
        basePoseMsg.pose.position.x = observation.state(0);
        basePoseMsg.pose.position.y = observation.state(1);
        basePoseMsg.pose.position.z = 0.0;
        const Eigen::Quaternion<scalar_t> baseOrientation(
            Eigen::AngleAxis<scalar_t>(observation.state(2), Eigen::Matrix<scalar_t, 3, 1>::UnitZ()));
        basePoseMsg.pose.orientation.w = baseOrientation.w();
        basePoseMsg.pose.orientation.x = baseOrientation.x();
        basePoseMsg.pose.orientation.y = baseOrientation.y();
        basePoseMsg.pose.orientation.z = baseOrientation.z();
        debugBasePosePublisher_->publish(basePoseMsg);
    }

    scalar_t getNextResetObservationTime() const
    {
        if (!simulationTimeInitialized_)
        {
            return 0.0;
        }
        return latestPublishedSimulationTime_ + dtCtrl_;
    }

    void recordSimulationTime(scalar_t simulationTime)
    {
        latestPublishedSimulationTime_ = simulationTime;
        simulationTimeInitialized_ = true;
    }

    void validateTiming()
    {
        if (dtSim_ <= 0.0 || dtCtrl_ <= 0.0)
        {
            throw std::runtime_error("dtSim and dtCtrl must be positive.");
        }

        const scalar_t ratio = dtCtrl_ / dtSim_;
        const scalar_t roundedRatio = std::round(ratio);
        if (std::abs(ratio - roundedRatio) > 1e-9)
        {
            throw std::runtime_error("dtCtrl must be an integer multiple of dtSim.");
        }
        simSubsteps_ = static_cast<int>(roundedRatio);

        const scalar_t configuredMpcHz = interface_.mpcSettings().mpcDesiredFrequency_;
        const scalar_t requestedMpcHz = 1.0 / dtCtrl_;
        if (std::abs(configuredMpcHz - requestedMpcHz) > 1e-6)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "task.info mpcDesiredFrequency (%.3f Hz) does not match dtCtrl (%.3f Hz).",
                configuredMpcHz,
                requestedMpcHz);
        }
    }

    void validateModelInfo() const
    {
        const ManipulatorModelInfo& modelInfo = interface_.getManipulatorModelInfo();
        if (modelInfo.stateDim != 17 || modelInfo.inputDim != 16 || modelInfo.armDim != 14)
        {
            throw std::runtime_error("Unexpected reduced model dimensions for g7_openarm.");
        }

        for (size_t i = 0; i < kArmJointNames.size(); ++i)
        {
            if (modelInfo.dofNames.at(i) != kArmJointNames[i])
            {
                throw std::runtime_error(
                    "OCS2 arm joint ordering does not match g7_openarm task.info ordering.");
            }
        }

        if (!interface_.dual_arm_)
        {
            throw std::runtime_error("g7_openarm MuJoCo bridge expects dual-arm mode to be enabled.");
        }
    }

    void loadInputLimits()
    {
        const auto& modelInfo = interface_.getManipulatorModelInfo();
        inputLowerBound_ = vector_t::Zero(modelInfo.inputDim);
        inputUpperBound_ = vector_t::Zero(modelInfo.inputDim);

        vector_t baseLowerBound = vector_t::Zero(2);
        vector_t baseUpperBound = vector_t::Zero(2);
        vector_t armLowerBound = vector_t::Zero(modelInfo.armDim);
        vector_t armUpperBound = vector_t::Zero(modelInfo.armDim);

        ocs2::loadData::loadEigenMatrix(
            taskFile_,
            "jointVelocityLimits.lowerBound.base.wheelBasedMobileManipulator",
            baseLowerBound);
        ocs2::loadData::loadEigenMatrix(
            taskFile_,
            "jointVelocityLimits.upperBound.base.wheelBasedMobileManipulator",
            baseUpperBound);
        ocs2::loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.lowerBound.arm", armLowerBound);
        ocs2::loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.upperBound.arm", armUpperBound);

        inputLowerBound_.head(baseLowerBound.size()) = baseLowerBound;
        inputUpperBound_.head(baseUpperBound.size()) = baseUpperBound;
        inputLowerBound_.tail(armLowerBound.size()) = armLowerBound;
        inputUpperBound_.tail(armUpperBound.size()) = armUpperBound;
    }

    void loadMujocoPlugins() const
    {
        static const bool pluginsLoaded = [this]()
        {
            if (std::string(kMujocoPluginDir).empty())
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "MuJoCo plugin directory is not configured; mesh decoder plugins may be unavailable.");
                return false;
            }

            mj_loadAllPluginLibraries(kMujocoPluginDir, +[](const char*, int, int) {});
            RCLCPP_INFO(
                node_->get_logger(),
                "Loaded MuJoCo bundled plugins from %s",
                kMujocoPluginDir);
            return true;
        }();

        (void)pluginsLoaded;
    }

    void loadMujocoModel()
    {
        std::array<char, 1024> error{};
        model_ = mj_loadXML(mjcfFile_.c_str(), nullptr, error.data(), error.size());
        if (model_ == nullptr)
        {
            throw std::runtime_error("Failed to load MJCF: " + std::string(error.data()));
        }

        data_ = mj_makeData(model_);
        if (data_ == nullptr)
        {
            throw std::runtime_error("Failed to allocate MuJoCo data.");
        }

        model_->opt.timestep = dtSim_;
        mj_forward(model_, data_);
    }

    void cacheModelHandles()
    {
        for (size_t i = 0; i < kBaseJointNames.size(); ++i)
        {
            cacheJointAndActuator(
                kBaseJointNames[i],
                kBaseActuatorNames[i],
                baseJointIds_[i],
                baseQposAdr_[i],
                baseQvelAdr_[i],
                baseActuatorIds_[i]);
        }

        for (size_t i = 0; i < kArmJointNames.size(); ++i)
        {
            cacheJointAndActuator(
                kArmJointNames[i],
                kArmActuatorNames[i],
                armJointIds_[i],
                armQposAdr_[i],
                armQvelAdr_[i],
                armActuatorIds_[i]);
        }

        const auto& model = pinocchioInterface_.getModel();
        eeFrameId_ = model.getFrameId(interface_.getManipulatorModelInfo().eeFrame);
        eeFrame1Id_ = model.getFrameId(interface_.getManipulatorModelInfo().eeFrame1);
        if (eeFrameId_ == static_cast<pinocchio::FrameIndex>(-1) ||
            eeFrame1Id_ == static_cast<pinocchio::FrameIndex>(-1))
        {
            throw std::runtime_error("Failed to resolve end-effector frames in Pinocchio.");
        }
    }

    void cacheJointAndActuator(
        const char* jointName,
        const char* actuatorName,
        int& jointId,
        int& qposAdr,
        int& qvelAdr,
        int& actuatorId) const
    {
        jointId = mj_name2id(model_, mjOBJ_JOINT, jointName);
        actuatorId = mj_name2id(model_, mjOBJ_ACTUATOR, actuatorName);
        if (jointId < 0 || actuatorId < 0)
        {
            throw std::runtime_error(
                "Failed to resolve MuJoCo joint or actuator: " + std::string(jointName));
        }

        qposAdr = model_->jnt_qposadr[jointId];
        qvelAdr = model_->jnt_dofadr[jointId];
    }

    bool consumeResetRequest()
    {
        std::scoped_lock<std::mutex> lock(resetMutex_);
        const bool requested = pendingReset_;
        pendingReset_ = false;
        return requested;
    }

    void performReset()
    {
        const int homeKeyId = mj_name2id(model_, mjOBJ_KEY, "home");
        if (homeKeyId >= 0)
        {
            mj_resetDataKeyframe(model_, data_, homeKeyId);
        }
        else
        {
            mj_resetData(model_, data_);
        }

        model_->opt.timestep = dtSim_;
        mju_zero(data_->ctrl, model_->nu);
        mj_forward(model_, data_);

        lastAppliedInput_ = vector_t::Zero(interface_.getManipulatorModelInfo().inputDim);
        lastServoInput_ = lastAppliedInput_;
        currentObservation_ = buildObservation(getNextResetObservationTime(), 0);
        controlCycleCount_ = 0;
        recordSimulationTime(currentObservation_.time);
        publishClock(currentObservation_.time);
        publishDebugTopics(currentObservation_);

        const TargetTrajectories initTargetTrajectories =
            createCurrentTargetTrajectories(currentObservation_.time, currentObservation_.state);

        RCLCPP_INFO(node_->get_logger(), "Resetting MPC with current MuJoCo state.");
        mrt_.resetMpcNode(initTargetTrajectories);

        while (!mrt_.initialPolicyReceived() && rclcpp::ok())
        {
            mrt_.spinMRT();
            mrt_.setCurrentObservation(currentObservation_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (!rclcpp::ok())
        {
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "Initial MPC policy received.");
        publishObservationAndWaitForPolicy(currentObservation_);
        visualization_->update(currentObservation_, mrt_.getPolicy(), mrt_.getCommand());
    }

    SystemObservation buildObservation(scalar_t time, size_t mode) const
    {
        SystemObservation observation;
        observation.time = time;
        observation.mode = mode;
        observation.state = vector_t::Zero(interface_.getManipulatorModelInfo().stateDim);
        observation.input = lastAppliedInput_;

        observation.state(0) = data_->qpos[baseQposAdr_[0]];
        observation.state(1) = data_->qpos[baseQposAdr_[1]];
        observation.state(2) = data_->qpos[baseQposAdr_[2]];

        for (size_t i = 0; i < kArmJointNames.size(); ++i)
        {
            observation.state(static_cast<Eigen::Index>(3 + i)) = data_->qpos[armQposAdr_[i]];
        }

        return observation;
    }

    void publishObservationAndWaitForPolicy(const SystemObservation& observation)
    {
        mrt_.setCurrentObservation(observation);

        const scalar_t timeTolerance = 0.25 * dtCtrl_;
        bool acceptedPolicy = false;
        while (rclcpp::ok() && !acceptedPolicy)
        {
            mrt_.spinMRT();
            if (mrt_.updatePolicy())
            {
                const auto& policy = mrt_.getPolicy();
                if (!policy.timeTrajectory_.empty() &&
                    std::abs(policy.timeTrajectory_.front() - observation.time) <= timeTolerance)
                {
                    acceptedPolicy = true;
                }
            }
            if (!acceptedPolicy)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        if (!acceptedPolicy)
        {
            throw std::runtime_error("Failed to receive an MPC policy for the current simulation time.");
        }

        lastAcceptedPolicyTime_ = observation.time;
    }

    void applyVelocityServo(const vector_t& optimizedInput, scalar_t yaw)
    {
        if (optimizedInput.size() != interface_.getManipulatorModelInfo().inputDim)
        {
            throw std::runtime_error("MPC input dimension does not match reduced model input dimension.");
        }

        vector_t clampedInput = optimizedInput;
        for (Eigen::Index i = 0; i < clampedInput.size(); ++i)
        {
            clampedInput(i) =
                std::clamp(clampedInput(i), inputLowerBound_(i), inputUpperBound_(i));
        }
        lastServoInput_ = clampedInput;

        mju_zero(data_->ctrl, model_->nu);

        const scalar_t bodyForwardVelocity = clampedInput(0);
        const scalar_t yawRate = clampedInput(1);
        const scalar_t targetBaseXDot = std::cos(yaw) * bodyForwardVelocity;
        const scalar_t targetBaseYDot = std::sin(yaw) * bodyForwardVelocity;

        setActuatorControl(
            baseActuatorIds_[0],
            gains_.baseLinearKv * (targetBaseXDot - data_->qvel[baseQvelAdr_[0]]));
        setActuatorControl(
            baseActuatorIds_[1],
            gains_.baseLinearKv * (targetBaseYDot - data_->qvel[baseQvelAdr_[1]]));
        setActuatorControl(
            baseActuatorIds_[2],
            gains_.baseYawKv * (yawRate - data_->qvel[baseQvelAdr_[2]]));

        for (size_t i = 0; i < kArmJointNames.size(); ++i)
        {
            const scalar_t targetJointVelocity = clampedInput(static_cast<Eigen::Index>(2 + i));
            const scalar_t jointVelocityError = targetJointVelocity - data_->qvel[armQvelAdr_[i]];
            setActuatorControl(armActuatorIds_[i], gains_.armJointKv * jointVelocityError);
        }
    }

    void setActuatorControl(int actuatorId, scalar_t value)
    {
        scalar_t clampedValue = value;
        if (model_->actuator_ctrllimited[actuatorId] != 0)
        {
            const scalar_t lower = model_->actuator_ctrlrange[2 * actuatorId];
            const scalar_t upper = model_->actuator_ctrlrange[2 * actuatorId + 1];
            clampedValue = std::clamp(clampedValue, lower, upper);
        }
        data_->ctrl[actuatorId] = clampedValue;
    }

    TargetTrajectories createCurrentTargetTrajectories(
        scalar_t time,
        const vector_t& state)
    {
        vector_t qPinocchio = pinocchioMapping_.getPinocchioJointPosition(state);
        auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, qPinocchio);
        pinocchio::updateFramePlacements(model, data);

        const auto& leftFrame = data.oMf[eeFrameId_];
        const auto& rightFrame = data.oMf[eeFrame1Id_];
        const Eigen::Quaternion<scalar_t> leftOrientation(leftFrame.rotation());
        const Eigen::Quaternion<scalar_t> rightOrientation(rightFrame.rotation());

        vector_t targetState(14);
        targetState << leftFrame.translation(),
            leftOrientation.coeffs(),
            rightFrame.translation(),
            rightOrientation.coeffs();

        const scalar_array_t timeTrajectory{time};
        const vector_array_t stateTrajectory{targetState};
        const vector_array_t inputTrajectory{
            vector_t::Zero(interface_.getManipulatorModelInfo().inputDim)
        };
        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string taskFile_;
    std::string libFolder_;
    std::string urdfFile_;
    std::string mjcfFile_;
    scalar_t dtSim_ = 0.001;
    scalar_t dtCtrl_ = 0.01;
    int simSubsteps_ = 10;
    ServoGains gains_;

    MobileManipulatorInterface interface_;
    ocs2::MRT_ROS_Interface mrt_;
    std::shared_ptr<MobileManipulatorDummyVisualization> visualization_;

    PinocchioInterface pinocchioInterface_;
    MobileManipulatorPinocchioMapping pinocchioMapping_;
    pinocchio::FrameIndex eeFrameId_;
    pinocchio::FrameIndex eeFrame1Id_;

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    std::array<int, 3> baseJointIds_{};
    std::array<int, 3> baseQposAdr_{};
    std::array<int, 3> baseQvelAdr_{};
    std::array<int, 3> baseActuatorIds_{};
    std::array<int, 14> armJointIds_{};
    std::array<int, 14> armQposAdr_{};
    std::array<int, 14> armQvelAdr_{};
    std::array<int, 14> armActuatorIds_{};

    vector_t lastAppliedInput_;
    vector_t lastServoInput_;
    vector_t inputLowerBound_;
    vector_t inputUpperBound_;
    SystemObservation currentObservation_;

    std::mutex resetMutex_;
    bool pendingReset_ = false;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPublisher_;
    rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr debugObservationPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debugInputPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debugBasePosePublisher_;
    size_t controlCycleCount_ = 0;
    scalar_t lastAcceptedPolicyTime_ = 0.0;
    scalar_t latestPublishedSimulationTime_ = 0.0;
    bool simulationTimeInitialized_ = false;
};
} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        G7OpenarmMujocoMrtNode node;
        node.run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("g7_openarm_mujoco_mrt"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
