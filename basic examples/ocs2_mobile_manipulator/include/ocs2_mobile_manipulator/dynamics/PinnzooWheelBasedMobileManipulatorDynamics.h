#pragma once

#include <ocs2_core/dynamics/ControlledSystemBase.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/dynamics/PinnzooInterface.h"

namespace ocs2::mobile_manipulator
{
    class PinnzooWheelBasedMobileManipulatorDynamics final : public ControlledSystemBase
    {
    public:
        PinnzooWheelBasedMobileManipulatorDynamics(ManipulatorModelInfo modelInfo,
                                                   PinnzooSettings settings,
                                                   std::vector<std::string> removedJointNames);

        ~PinnzooWheelBasedMobileManipulatorDynamics() override = default;

        PinnzooWheelBasedMobileManipulatorDynamics* clone() const override
        {
            return new PinnzooWheelBasedMobileManipulatorDynamics(*this);
        }

        vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input,
                                const PreComputation& preComp) override;

        const PinnzooInterface& getPinnzooInterface() const { return pinnzooInterface_; }

    private:
        PinnzooWheelBasedMobileManipulatorDynamics(const PinnzooWheelBasedMobileManipulatorDynamics& rhs);

        Eigen::VectorXd buildFullState(const vector_t& state, const vector_t& input) const;
        vector_t reduceFlowMap(const Eigen::VectorXd& fullState, const Eigen::VectorXd& fullQdot) const;
        void buildJointMappings();
        void validateGeneratedModel() const;
        static double computeYawRateFromQuaternionDerivative(const Eigen::Vector4d& q, const Eigen::Vector4d& qdot);

        ManipulatorModelInfo modelInfo_;
        PinnzooInterface pinnzooInterface_;
        std::vector<std::string> removedJointNames_;
        std::vector<int> armConfigIndices_;
        std::vector<int> armVelocityIndices_;
        std::vector<int> fixedConfigIndices_;
        std::vector<double> fixedConfigValues_;
        int xConfigIndex_ = -1;
        int yConfigIndex_ = -1;
        int zConfigIndex_ = -1;
        int qwConfigIndex_ = -1;
        int qxConfigIndex_ = -1;
        int qyConfigIndex_ = -1;
        int qzConfigIndex_ = -1;
        int baseLinVxIndex_ = -1;
        int baseLinVyIndex_ = -1;
        int baseLinVzIndex_ = -1;
        int baseAngVxIndex_ = -1;
        int baseAngVyIndex_ = -1;
        int baseAngVzIndex_ = -1;
    };
}
