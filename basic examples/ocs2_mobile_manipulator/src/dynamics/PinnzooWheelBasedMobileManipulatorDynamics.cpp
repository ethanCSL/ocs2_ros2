#include "ocs2_mobile_manipulator/dynamics/PinnzooWheelBasedMobileManipulatorDynamics.h"

#include <Eigen/Geometry>

#include <stdexcept>
#include <unordered_map>

namespace ocs2::mobile_manipulator
{
    namespace
    {
        constexpr double kYawRateEps = 1e-12;
    }

    PinnzooWheelBasedMobileManipulatorDynamics::PinnzooWheelBasedMobileManipulatorDynamics(
        ManipulatorModelInfo modelInfo, PinnzooSettings settings, std::vector<std::string> removedJointNames)
        : modelInfo_(std::move(modelInfo)),
          pinnzooInterface_(std::move(settings)),
          removedJointNames_(std::move(removedJointNames))
    {
        if (modelInfo_.manipulatorModelType != ManipulatorModelType::WheelBasedMobileManipulator)
        {
            throw std::runtime_error("PinnZoo backend currently supports only WheelBasedMobileManipulator.");
        }

        buildJointMappings();
        validateGeneratedModel();
    }

    PinnzooWheelBasedMobileManipulatorDynamics::PinnzooWheelBasedMobileManipulatorDynamics(
        const PinnzooWheelBasedMobileManipulatorDynamics& rhs)
        : ControlledSystemBase(rhs),
          modelInfo_(rhs.modelInfo_),
          pinnzooInterface_(rhs.pinnzooInterface_),
          removedJointNames_(rhs.removedJointNames_),
          armConfigIndices_(rhs.armConfigIndices_),
          armVelocityIndices_(rhs.armVelocityIndices_),
          fixedConfigIndices_(rhs.fixedConfigIndices_),
          fixedConfigValues_(rhs.fixedConfigValues_),
          xConfigIndex_(rhs.xConfigIndex_),
          yConfigIndex_(rhs.yConfigIndex_),
          zConfigIndex_(rhs.zConfigIndex_),
          qwConfigIndex_(rhs.qwConfigIndex_),
          qxConfigIndex_(rhs.qxConfigIndex_),
          qyConfigIndex_(rhs.qyConfigIndex_),
          qzConfigIndex_(rhs.qzConfigIndex_),
          baseLinVxIndex_(rhs.baseLinVxIndex_),
          baseLinVyIndex_(rhs.baseLinVyIndex_),
          baseLinVzIndex_(rhs.baseLinVzIndex_),
          baseAngVxIndex_(rhs.baseAngVxIndex_),
          baseAngVyIndex_(rhs.baseAngVyIndex_),
          baseAngVzIndex_(rhs.baseAngVzIndex_)
    {
    }

    vector_t PinnzooWheelBasedMobileManipulatorDynamics::computeFlowMap(
        scalar_t /*time*/, const vector_t& state, const vector_t& input, const PreComputation& /*preComp*/)
    {
        const Eigen::VectorXd fullState = buildFullState(state, input);
        const Eigen::MatrixXd velocityKinematics = pinnzooInterface_.evaluateVelocityKinematics(fullState);
        const Eigen::VectorXd fullVelocity = fullState.tail(static_cast<long>(pinnzooInterface_.vDim()));
        const Eigen::VectorXd fullQdot = velocityKinematics * fullVelocity;
        return reduceFlowMap(fullState, fullQdot);
    }

    Eigen::VectorXd PinnzooWheelBasedMobileManipulatorDynamics::buildFullState(
        const vector_t& state, const vector_t& input) const
    {
        if (state.size() != static_cast<long>(modelInfo_.stateDim))
        {
            throw std::runtime_error("OCS2 state dimension does not match WheelBasedMobileManipulator model info.");
        }
        if (input.size() != static_cast<long>(modelInfo_.inputDim))
        {
            throw std::runtime_error("OCS2 input dimension does not match WheelBasedMobileManipulator model info.");
        }

        Eigen::VectorXd fullState = Eigen::VectorXd::Zero(static_cast<long>(pinnzooInterface_.xDim()));
        auto q = fullState.head(static_cast<long>(pinnzooInterface_.qDim()));
        auto v = fullState.tail(static_cast<long>(pinnzooInterface_.vDim()));

        q(zConfigIndex_) = pinnzooInterface_.settings().baseHeight;
        q(qwConfigIndex_) = 1.0;

        for (std::size_t i = 0; i < fixedConfigIndices_.size(); ++i)
        {
            q(fixedConfigIndices_[i]) = fixedConfigValues_[i];
        }

        q(xConfigIndex_) = state(0);
        q(yConfigIndex_) = state(1);

        const Eigen::Quaterniond planarBaseOrientation(
            Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()));
        q(qwConfigIndex_) = planarBaseOrientation.w();
        q(qxConfigIndex_) = planarBaseOrientation.x();
        q(qyConfigIndex_) = planarBaseOrientation.y();
        q(qzConfigIndex_) = planarBaseOrientation.z();

        for (std::size_t i = 0; i < armConfigIndices_.size(); ++i)
        {
            q(armConfigIndices_[i]) = state(static_cast<long>(3 + i));
            v(armVelocityIndices_[i]) = input(static_cast<long>(2 + i));
        }

        v(baseLinVxIndex_) = input(0);
        v(baseLinVyIndex_) = 0.0;
        v(baseLinVzIndex_) = 0.0;
        v(baseAngVxIndex_) = 0.0;
        v(baseAngVyIndex_) = 0.0;
        v(baseAngVzIndex_) = input(1);

        return fullState;
    }

    vector_t PinnzooWheelBasedMobileManipulatorDynamics::reduceFlowMap(
        const Eigen::VectorXd& fullState, const Eigen::VectorXd& fullQdot) const
    {
        vector_t flow = vector_t::Zero(static_cast<long>(modelInfo_.stateDim));
        flow(0) = fullQdot(xConfigIndex_);
        flow(1) = fullQdot(yConfigIndex_);

        const Eigen::Vector4d quaternion(
            fullState(qwConfigIndex_), fullState(qxConfigIndex_), fullState(qyConfigIndex_), fullState(qzConfigIndex_));
        const Eigen::Vector4d quaternionDot(
            fullQdot(qwConfigIndex_), fullQdot(qxConfigIndex_), fullQdot(qyConfigIndex_), fullQdot(qzConfigIndex_));
        flow(2) = computeYawRateFromQuaternionDerivative(quaternion, quaternionDot);

        for (std::size_t i = 0; i < armConfigIndices_.size(); ++i)
        {
            flow(static_cast<long>(3 + i)) = fullQdot(armConfigIndices_[i]);
        }

        return flow;
    }

    void PinnzooWheelBasedMobileManipulatorDynamics::buildJointMappings()
    {
        xConfigIndex_ = pinnzooInterface_.configIndex("x");
        yConfigIndex_ = pinnzooInterface_.configIndex("y");
        zConfigIndex_ = pinnzooInterface_.configIndex("z");
        qwConfigIndex_ = pinnzooInterface_.configIndex("q_w");
        qxConfigIndex_ = pinnzooInterface_.configIndex("q_x");
        qyConfigIndex_ = pinnzooInterface_.configIndex("q_y");
        qzConfigIndex_ = pinnzooInterface_.configIndex("q_z");

        baseLinVxIndex_ = pinnzooInterface_.velocityIndex("lin_v_x");
        baseLinVyIndex_ = pinnzooInterface_.velocityIndex("lin_v_y");
        baseLinVzIndex_ = pinnzooInterface_.velocityIndex("lin_v_z");
        baseAngVxIndex_ = pinnzooInterface_.velocityIndex("ang_v_x");
        baseAngVyIndex_ = pinnzooInterface_.velocityIndex("ang_v_y");
        baseAngVzIndex_ = pinnzooInterface_.velocityIndex("ang_v_z");

        armConfigIndices_.clear();
        armVelocityIndices_.clear();
        armConfigIndices_.reserve(modelInfo_.dofNames.size());
        armVelocityIndices_.reserve(modelInfo_.dofNames.size());
        for (const auto& name : modelInfo_.dofNames)
        {
            armConfigIndices_.push_back(pinnzooInterface_.configIndex(name));
            armVelocityIndices_.push_back(pinnzooInterface_.velocityIndex(name));
        }

        std::unordered_map<std::string, double> fixedJointMap;
        for (std::size_t i = 0; i < pinnzooInterface_.settings().fixedJointNames.size(); ++i)
        {
            fixedJointMap[pinnzooInterface_.settings().fixedJointNames[i]] = pinnzooInterface_.settings().fixedJointPositions[i];
        }

        fixedConfigIndices_.clear();
        fixedConfigValues_.clear();
        for (const auto& name : removedJointNames_)
        {
            const auto fixedPositionIt = fixedJointMap.find(name);
            const double fixedPosition = fixedPositionIt != fixedJointMap.end() ? fixedPositionIt->second : 0.0;

            try
            {
                fixedConfigIndices_.push_back(pinnzooInterface_.configIndex(name));
                fixedConfigValues_.push_back(fixedPosition);
            }
            catch (const std::exception&)
            {
            }
        }
    }

    void PinnzooWheelBasedMobileManipulatorDynamics::validateGeneratedModel() const
    {
        Eigen::VectorXd zeroState = Eigen::VectorXd::Zero(static_cast<long>(modelInfo_.stateDim));
        Eigen::VectorXd zeroInput = Eigen::VectorXd::Zero(static_cast<long>(modelInfo_.inputDim));
        const Eigen::VectorXd fullState = buildFullState(zeroState, zeroInput);

        const Eigen::MatrixXd E = pinnzooInterface_.evaluateVelocityKinematics(fullState);
        if (E.rows() != static_cast<long>(pinnzooInterface_.qDim()) ||
            E.cols() != static_cast<long>(pinnzooInterface_.vDim()))
        {
            throw std::runtime_error("PinnZoo velocity kinematics matrix has unexpected dimensions.");
        }

        if (pinnzooInterface_.settings().validateDynamics)
        {
            const Eigen::VectorXd zeroTau = Eigen::VectorXd::Zero(static_cast<long>(pinnzooInterface_.tauDim()));
            const Eigen::VectorXd fullDynamics = pinnzooInterface_.evaluateDynamics(fullState, zeroTau);
            if (fullDynamics.size() != static_cast<long>(pinnzooInterface_.xDim()))
            {
                throw std::runtime_error("PinnZoo dynamics output has unexpected dimension.");
            }
        }
    }

    double PinnzooWheelBasedMobileManipulatorDynamics::computeYawRateFromQuaternionDerivative(
        const Eigen::Vector4d& q, const Eigen::Vector4d& qdot)
    {
        const double n = 2.0 * (q(0) * q(3) + q(1) * q(2));
        const double d = 1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3));
        const double ndot = 2.0 * (qdot(0) * q(3) + q(0) * qdot(3) + qdot(1) * q(2) + q(1) * qdot(2));
        const double ddot = -4.0 * (q(2) * qdot(2) + q(3) * qdot(3));
        const double denom = n * n + d * d;

        if (denom < kYawRateEps)
        {
            return 0.0;
        }

        return (d * ndot - n * ddot) / denom;
    }
}
