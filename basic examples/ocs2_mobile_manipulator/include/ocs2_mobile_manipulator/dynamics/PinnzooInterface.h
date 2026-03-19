#pragma once

#include <Eigen/Core>

#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace ocs2::mobile_manipulator
{
    struct PinnzooSettings
    {
        bool enabled = false;
        bool validateDynamics = true;
        double baseHeight = 0.11;
        std::string libraryPath;
        std::string dynamicsSymbol = "dynamics_wrapper";
        std::string velocityKinematicsSymbol = "velocity_kinematics_wrapper";
        std::string configOrderSymbol = "get_config_order";
        std::string velocityOrderSymbol = "get_vel_order";
        std::string torqueOrderSymbol = "get_torque_order";
        std::string generatedUrdfPathSymbol = "get_urdf_path";
        std::vector<std::string> fixedJointNames;
        std::vector<double> fixedJointPositions;
    };

    PinnzooSettings loadPinnzooSettings(const std::string& taskFile);

    class PinnzooInterface
    {
    public:
        explicit PinnzooInterface(PinnzooSettings settings);
        PinnzooInterface(const PinnzooInterface& rhs);
        PinnzooInterface(PinnzooInterface&& rhs) noexcept;
        PinnzooInterface& operator=(const PinnzooInterface&) = delete;
        PinnzooInterface& operator=(PinnzooInterface&& rhs) noexcept;
        ~PinnzooInterface();

        void validate() const;

        Eigen::MatrixXd evaluateVelocityKinematics(const Eigen::VectorXd& fullState) const;
        Eigen::VectorXd evaluateDynamics(const Eigen::VectorXd& fullState, const Eigen::VectorXd& fullTau) const;

        const PinnzooSettings& settings() const { return settings_; }
        const std::vector<std::string>& configOrder() const { return configOrder_; }
        const std::vector<std::string>& velocityOrder() const { return velocityOrder_; }
        const std::vector<std::string>& torqueOrder() const { return torqueOrder_; }
        const std::string& generatedUrdfPath() const { return generatedUrdfPath_; }

        std::size_t qDim() const { return configOrder_.size(); }
        std::size_t vDim() const { return velocityOrder_.size(); }
        std::size_t tauDim() const { return torqueOrder_.size(); }
        std::size_t xDim() const { return qDim() + vDim(); }

        int configIndex(const std::string& name) const;
        int velocityIndex(const std::string& name) const;
        int torqueIndex(const std::string& name) const;

    private:
        using OrderFn = const char** (*)();
        using StringFn = const char* (*)();
        using DynamicsFn = void (*)(double*, double*, double*);
        using VelocityKinematicsFn = void (*)(double*, double*);

        explicit PinnzooInterface(const PinnzooSettings& settings, void* handle);

        void openLibrary();
        void closeLibrary() noexcept;
        void loadSymbols();
        static std::vector<std::string> loadOrder(OrderFn orderFn, const std::string& symbolName);

        template <typename MapType>
        static int lookupIndex(const std::string& name, const MapType& indexMap, const char* label);

        PinnzooSettings settings_;
        std::vector<std::string> configOrder_;
        std::vector<std::string> velocityOrder_;
        std::vector<std::string> torqueOrder_;
        std::string generatedUrdfPath_;
        std::unordered_map<std::string, int> configIndexMap_;
        std::unordered_map<std::string, int> velocityIndexMap_;
        std::unordered_map<std::string, int> torqueIndexMap_;
        void* handle_ = nullptr;
        DynamicsFn dynamicsFn_ = nullptr;
        VelocityKinematicsFn velocityKinematicsFn_ = nullptr;
    };

    template <typename MapType>
    int PinnzooInterface::lookupIndex(const std::string& name, const MapType& indexMap, const char* label)
    {
        const auto it = indexMap.find(name);
        if (it == indexMap.end())
        {
            throw std::runtime_error(
                std::string("PinnZoo ") + label + " '" + name + "' is not present in the generated library order list.");
        }
        return it->second;
    }
}
