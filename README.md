# OCS2_ROS2 Toolbox

## Quick Start For This Fork

This fork adds a `g5_openarm` workflow on top of the ROS 2 OCS2 port, including:

- `g5_openarm` robot description, launch files, and RViz config
- a switchable PinnZoo-backed wheel-based dynamics backend for `ocs2_mobile_manipulator`
- a ready-to-use `g5_openarm_pinnzoo.launch.py` entrypoint

Companion PinnZoo repository:

- `https://github.com/ethanCSL/PinnZoo.git`
- branch: `g7-openarm`

Fastest path to run `g5_openarm`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ethanCSL/ocs2_ros2.git
git clone -b g7-openarm https://github.com/ethanCSL/PinnZoo.git ~/PinnZoo
cd ocs2_ros2
git submodule update --init --recursive
cd ~/PinnZoo
mkdir -p build
cd build
cmake ..
cmake --build . --target g7_openarm_quat
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  ocs2_mobile_manipulator \
  ocs2_mobile_manipulator_ros \
  g5_openarm_description \
  g5_openarm_ocs2 \
  g5_openarm_ros
source ~/ros2_ws/install/setup.bash
export PINNZOO_LIBRARY_PATH=~/PinnZoo/build/libg7_openarm_quat.so
ros2 launch g5_openarm_ros g5_openarm_pinnzoo.launch.py
```

## 1. Summary

OCS2_ROS2 is developed based on [OCS2](https://github.com/leggedrobotics/ocs2), it was refactored to be compatible with ROS2 and modern cmake.

### What's New (2025.08)

**Pinocchio 3 Dependency Optimization**
- Upgraded to Pinocchio 3 version for better performance and stability
- Support for installing Pinocchio from ROS sources, avoiding complex third-party package management

**Dual-Arm Mobile Manipulator Support**
- Added Dual-Arm Mobile Manipulator functionality
- Enhanced interactive markers for better user operation experience

The IDE I used is CLion, you can follow the [guide](https://www.jetbrains.com/help/clion/ros2-tutorial.html) to set up
the IDE.

### Tested Platform

* Intel Nuc X15 (i7-11800H):
    * Ubuntu 22.04 ROS2 Humble  (WSL2 included)
    * Ubuntu 24.04 ROS2 Jazzy   (WSL2 included)
* Lenovo P16v (i7-13800H):
    * Ubuntu 24.04 ROS2 Jazzy
* Jetson Orin Nano
    * Ubuntu 22.04 ROS2 Humble (JetPack 6.1)

## 2. Installation

### 2.1 Prerequisites

The OCS2 library is written in C++17. It is tested under Ubuntu with library versions as provided in the package
sources.

Tested system and ROS2 version:

* Ubuntu 24.04 ROS2 Jazzy
* Ubuntu 22.04 ROS2 Humble

### 2.2 Dependencies

* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

> **Note:** Latest version used pinocchio from ros source to simplified install steps. If you install pinocchio from robot-pkgs, you can uninstall it by
> ```bash
> sudo apt remove robotpkg-*
> ```

### 2.3 Clone Repositories

* Create a new workspace or clone the project to your workspace

```bash
cd ~
mkdir -p ros2_ws/src
```

* Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/ethanCSL/ocs2_ros2.git
cd ocs2_ros2
git submodule update --init --recursive
```

* rosdep

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2.4 g5_openarm + PinnZoo quick start

This fork includes a `g5_openarm` integration that can switch the wheel-based mobile manipulator dynamics from the native OCS2 implementation to a PinnZoo-generated shared library.

Clone and build PinnZoo separately:

```bash
cd ~
git clone -b g7-openarm https://github.com/ethanCSL/PinnZoo.git
cd PinnZoo
mkdir -p build
cd build
cmake ..
cmake --build . --target g7_openarm_quat
```

Then build this workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 PYTHON_EXECUTABLE=/usr/bin/python3 \
  colcon build --packages-select \
    ocs2_mobile_manipulator \
    ocs2_mobile_manipulator_ros \
    g5_openarm_description \
    g5_openarm_ocs2 \
    g5_openarm_ros
source ~/ros2_ws/install/setup.bash
```

Export the PinnZoo library path and launch:

```bash
export PINNZOO_LIBRARY_PATH=~/PinnZoo/build/libg7_openarm_quat.so
ros2 launch g5_openarm_ros g5_openarm_pinnzoo.launch.py
```

The detailed robot-specific guide is in `g5_openarm/README.md`.