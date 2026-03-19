#include "ocs2_mobile_manipulator/dynamics/PinnzooInterface.h"

#include <dlfcn.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>

namespace ocs2::mobile_manipulator
{
    namespace
    {
        template <typename FunctionType>
        FunctionType loadSymbol(void* handle, const std::string& symbolName)
        {
            dlerror();
            auto* fn = reinterpret_cast<FunctionType>(dlsym(handle, symbolName.c_str()));
            const char* dlsymError = dlerror();
            if (dlsymError != nullptr || fn == nullptr)
            {
                throw std::runtime_error(
                    "Failed to resolve PinnZoo symbol '" + symbolName + "': " +
                    (dlsymError != nullptr ? std::string(dlsymError) : std::string("unknown dlsym error")));
            }
            return fn;
        }

        std::string requireEnv(const std::string& name)
        {
            const char* value = std::getenv(name.c_str());
            if (value == nullptr || std::string(value).empty())
            {
                throw std::runtime_error(
                    "Environment variable '" + name + "' is not set, but it is required to resolve the PinnZoo library path.");
            }
            return value;
        }

        std::string expandUserAndEnv(std::string value)
        {
            if (value.rfind("~/", 0) == 0)
            {
                value.replace(0, 1, requireEnv("HOME"));
            }

            std::string expanded;
            expanded.reserve(value.size());

            for (std::size_t i = 0; i < value.size(); ++i)
            {
                if (value[i] != '$')
                {
                    expanded.push_back(value[i]);
                    continue;
                }

                if (i + 1 >= value.size())
                {
                    expanded.push_back(value[i]);
                    continue;
                }

                std::string varName;
                if (value[i + 1] == '{')
                {
                    const auto closing = value.find('}', i + 2);
                    if (closing == std::string::npos)
                    {
                        throw std::runtime_error("Malformed PinnZoo libraryPath: missing '}' in environment variable expression.");
                    }
                    varName = value.substr(i + 2, closing - (i + 2));
                    i = closing;
                }
                else
                {
                    std::size_t j = i + 1;
                    while (j < value.size() && (std::isalnum(static_cast<unsigned char>(value[j])) || value[j] == '_'))
                    {
                        ++j;
                    }
                    if (j == i + 1)
                    {
                        expanded.push_back(value[i]);
                        continue;
                    }
                    varName = value.substr(i + 1, j - (i + 1));
                    i = j - 1;
                }

                expanded += requireEnv(varName);
            }

            return expanded;
        }
    }

    PinnzooSettings loadPinnzooSettings(const std::string& taskFile)
    {
        PinnzooSettings settings;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        loadData::loadPtreeValue(pt, settings.enabled, "pinnzoo.enabled", false);
        loadData::loadPtreeValue(pt, settings.validateDynamics, "pinnzoo.validateDynamics", false);
        loadData::loadPtreeValue(pt, settings.baseHeight, "pinnzoo.baseHeight", false);
        loadData::loadPtreeValue(pt, settings.libraryPath, "pinnzoo.libraryPath", false);
        loadData::loadPtreeValue(pt, settings.dynamicsSymbol, "pinnzoo.dynamicsSymbol", false);
        loadData::loadPtreeValue(pt, settings.velocityKinematicsSymbol, "pinnzoo.velocityKinematicsSymbol", false);
        loadData::loadPtreeValue(pt, settings.configOrderSymbol, "pinnzoo.configOrderSymbol", false);
        loadData::loadPtreeValue(pt, settings.velocityOrderSymbol, "pinnzoo.velocityOrderSymbol", false);
        loadData::loadPtreeValue(pt, settings.torqueOrderSymbol, "pinnzoo.torqueOrderSymbol", false);
        loadData::loadPtreeValue(pt, settings.generatedUrdfPathSymbol, "pinnzoo.generatedUrdfPathSymbol", false);
        if (!settings.libraryPath.empty())
        {
            settings.libraryPath = expandUserAndEnv(settings.libraryPath);
        }

        loadData::loadStdVector<std::string>(
            taskFile, "pinnzoo.fixedJointPositions.jointNames", settings.fixedJointNames, false);

        if (!settings.fixedJointNames.empty())
        {
            Eigen::VectorXd fixedJointValues(static_cast<long>(settings.fixedJointNames.size()));
            loadData::loadEigenMatrix(taskFile, "pinnzoo.fixedJointPositions.values", fixedJointValues);
            settings.fixedJointPositions.assign(fixedJointValues.data(), fixedJointValues.data() + fixedJointValues.size());
        }

        if (settings.fixedJointPositions.size() != settings.fixedJointNames.size())
        {
            throw std::runtime_error(
                "PinnZoo fixedJointPositions.jointNames and fixedJointPositions.values must have the same length.");
        }

        return settings;
    }

    PinnzooInterface::PinnzooInterface(PinnzooSettings settings)
        : settings_(std::move(settings))
    {
        openLibrary();
        loadSymbols();
        validate();
    }

    PinnzooInterface::PinnzooInterface(const PinnzooInterface& rhs)
        : PinnzooInterface(rhs.settings_)
    {
    }

    PinnzooInterface::PinnzooInterface(PinnzooInterface&& rhs) noexcept
        : settings_(std::move(rhs.settings_)),
          configOrder_(std::move(rhs.configOrder_)),
          velocityOrder_(std::move(rhs.velocityOrder_)),
          torqueOrder_(std::move(rhs.torqueOrder_)),
          generatedUrdfPath_(std::move(rhs.generatedUrdfPath_)),
          configIndexMap_(std::move(rhs.configIndexMap_)),
          velocityIndexMap_(std::move(rhs.velocityIndexMap_)),
          torqueIndexMap_(std::move(rhs.torqueIndexMap_)),
          handle_(rhs.handle_),
          dynamicsFn_(rhs.dynamicsFn_),
          velocityKinematicsFn_(rhs.velocityKinematicsFn_)
    {
        rhs.handle_ = nullptr;
        rhs.dynamicsFn_ = nullptr;
        rhs.velocityKinematicsFn_ = nullptr;
    }

    PinnzooInterface& PinnzooInterface::operator=(PinnzooInterface&& rhs) noexcept
    {
        if (this != &rhs)
        {
            closeLibrary();
            settings_ = std::move(rhs.settings_);
            configOrder_ = std::move(rhs.configOrder_);
            velocityOrder_ = std::move(rhs.velocityOrder_);
            torqueOrder_ = std::move(rhs.torqueOrder_);
            generatedUrdfPath_ = std::move(rhs.generatedUrdfPath_);
            configIndexMap_ = std::move(rhs.configIndexMap_);
            velocityIndexMap_ = std::move(rhs.velocityIndexMap_);
            torqueIndexMap_ = std::move(rhs.torqueIndexMap_);
            handle_ = rhs.handle_;
            dynamicsFn_ = rhs.dynamicsFn_;
            velocityKinematicsFn_ = rhs.velocityKinematicsFn_;
            rhs.handle_ = nullptr;
            rhs.dynamicsFn_ = nullptr;
            rhs.velocityKinematicsFn_ = nullptr;
        }
        return *this;
    }

    PinnzooInterface::~PinnzooInterface()
    {
        closeLibrary();
    }

    void PinnzooInterface::validate() const
    {
        if (settings_.libraryPath.empty())
        {
            throw std::runtime_error("PinnZoo libraryPath is empty.");
        }
        if (handle_ == nullptr)
        {
            throw std::runtime_error("PinnZoo library handle is null.");
        }
        if (dynamicsFn_ == nullptr)
        {
            throw std::runtime_error("PinnZoo dynamics function is not loaded.");
        }
        if (velocityKinematicsFn_ == nullptr)
        {
            throw std::runtime_error("PinnZoo velocity kinematics function is not loaded.");
        }
        if (configOrder_.empty() || velocityOrder_.empty() || torqueOrder_.empty())
        {
            throw std::runtime_error("PinnZoo generated library order metadata is incomplete.");
        }
    }

    Eigen::MatrixXd PinnzooInterface::evaluateVelocityKinematics(const Eigen::VectorXd& fullState) const
    {
        if (static_cast<std::size_t>(fullState.size()) != xDim())
        {
            throw std::runtime_error("PinnZoo fullState size does not match library x dimension.");
        }

        Eigen::VectorXd stateCopy = fullState;
        std::vector<double> data(qDim() * vDim(), 0.0);
        velocityKinematicsFn_(stateCopy.data(), data.data());

        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> E(
            data.data(), static_cast<long>(qDim()), static_cast<long>(vDim()));
        return Eigen::MatrixXd(E);
    }

    Eigen::VectorXd PinnzooInterface::evaluateDynamics(const Eigen::VectorXd& fullState, const Eigen::VectorXd& fullTau) const
    {
        if (static_cast<std::size_t>(fullState.size()) != xDim())
        {
            throw std::runtime_error("PinnZoo fullState size does not match library x dimension.");
        }
        if (static_cast<std::size_t>(fullTau.size()) != tauDim())
        {
            throw std::runtime_error("PinnZoo fullTau size does not match library tau dimension.");
        }

        Eigen::VectorXd stateCopy = fullState;
        Eigen::VectorXd tauCopy = fullTau;
        Eigen::VectorXd xdot(static_cast<long>(xDim()));
        dynamicsFn_(stateCopy.data(), tauCopy.data(), xdot.data());
        return xdot;
    }

    int PinnzooInterface::configIndex(const std::string& name) const
    {
        return lookupIndex(name, configIndexMap_, "config");
    }

    int PinnzooInterface::velocityIndex(const std::string& name) const
    {
        return lookupIndex(name, velocityIndexMap_, "velocity");
    }

    int PinnzooInterface::torqueIndex(const std::string& name) const
    {
        return lookupIndex(name, torqueIndexMap_, "torque");
    }

    void PinnzooInterface::openLibrary()
    {
        handle_ = dlopen(settings_.libraryPath.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (handle_ == nullptr)
        {
            throw std::runtime_error(
                "Failed to dlopen PinnZoo library '" + settings_.libraryPath + "': " + dlerror());
        }
    }

    void PinnzooInterface::closeLibrary() noexcept
    {
        dynamicsFn_ = nullptr;
        velocityKinematicsFn_ = nullptr;
        if (handle_ != nullptr)
        {
            dlclose(handle_);
            handle_ = nullptr;
        }
    }

    void PinnzooInterface::loadSymbols()
    {
        dynamicsFn_ = loadSymbol<DynamicsFn>(handle_, settings_.dynamicsSymbol);
        velocityKinematicsFn_ = loadSymbol<VelocityKinematicsFn>(handle_, settings_.velocityKinematicsSymbol);

        const auto configOrderFn = loadSymbol<OrderFn>(handle_, settings_.configOrderSymbol);
        const auto velocityOrderFn = loadSymbol<OrderFn>(handle_, settings_.velocityOrderSymbol);
        const auto torqueOrderFn = loadSymbol<OrderFn>(handle_, settings_.torqueOrderSymbol);
        const auto generatedUrdfPathFn = loadSymbol<StringFn>(handle_, settings_.generatedUrdfPathSymbol);

        configOrder_ = loadOrder(configOrderFn, settings_.configOrderSymbol);
        velocityOrder_ = loadOrder(velocityOrderFn, settings_.velocityOrderSymbol);
        torqueOrder_ = loadOrder(torqueOrderFn, settings_.torqueOrderSymbol);
        generatedUrdfPath_ = generatedUrdfPathFn != nullptr && generatedUrdfPathFn() != nullptr ? generatedUrdfPathFn() : "";

        configIndexMap_.clear();
        velocityIndexMap_.clear();
        torqueIndexMap_.clear();

        for (std::size_t i = 0; i < configOrder_.size(); ++i)
        {
            configIndexMap_.emplace(configOrder_[i], static_cast<int>(i));
        }
        for (std::size_t i = 0; i < velocityOrder_.size(); ++i)
        {
            velocityIndexMap_.emplace(velocityOrder_[i], static_cast<int>(i));
        }
        for (std::size_t i = 0; i < torqueOrder_.size(); ++i)
        {
            torqueIndexMap_.emplace(torqueOrder_[i], static_cast<int>(i));
        }
    }

    std::vector<std::string> PinnzooInterface::loadOrder(OrderFn orderFn, const std::string& symbolName)
    {
        const char** rawOrder = orderFn();
        if (rawOrder == nullptr)
        {
            throw std::runtime_error("PinnZoo order symbol '" + symbolName + "' returned a null pointer.");
        }

        std::vector<std::string> order;
        for (std::size_t i = 0; rawOrder[i] != nullptr; ++i)
        {
            order.emplace_back(rawOrder[i]);
        }
        return order;
    }
}
