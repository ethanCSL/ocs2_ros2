# g7_openarm OCS2 + PinnZoo Integration Guide

## 0. MuJoCo Closed-Loop Quick Start

This repository now includes a first closed-loop MuJoCo plant for `g7_openarm`.
The controller remains OCS2/MPC, while MuJoCo provides the plant dynamics.

### 0.1 Build

```bash
cd ~/ocs2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  ocs2_mobile_manipulator_ros \
  g7_openarm_description \
  g7_openarm_ocs2 \
  g7_openarm_ros \
  --symlink-install
source ~/ocs2_ws/install/setup.bash
```

### 0.2 Launch

Standard OCS2 + MuJoCo closed-loop:

```bash
ros2 launch g7_openarm_ros g7_openarm.launch.py
```

Headless mode:

```bash
ros2 launch g7_openarm_ros g7_openarm.launch.py rviz:=false
```

PinnZoo backend + MuJoCo closed-loop:

```bash
export PINNZOO_LIBRARY_PATH=~/PinnZoo/build/libg7_openarm_quat.so
ros2 launch g7_openarm_ros g7_openarm_pinnzoo.launch.py
```

Reset the simulator and MPC together:

```bash
ros2 service call /mujoco_reset std_srvs/srv/Trigger "{}"
```

### 0.3 Default Timing

The current first-pass closed-loop uses:

- `dtSim = 0.001` (1000 Hz MuJoCo stepping)
- `dtCtrl = 0.01` (100 Hz OCS2 update)
- synchronous stepping
- zero-order hold between control updates

These are exposed as launch arguments:

```bash
ros2 launch g7_openarm_ros g7_openarm.launch.py \
  dtSim:=0.001 \
  dtCtrl:=0.01
```

### 0.4 Servo Gain Tuning

The MuJoCo side currently uses a simple velocity servo:

- `baseLinearKv`
- `baseYawKv`
- `armJointKv`

Example:

```bash
ros2 launch g7_openarm_ros g7_openarm.launch.py \
  baseLinearKv:=200.0 \
  baseYawKv:=150.0 \
  armJointKv:=15.0
```

Recommended tuning order:

1. Keep `dtSim=0.001` and `dtCtrl=0.01` fixed.
2. Tune `baseLinearKv` until the base tracks forward velocity without obvious overshoot.
3. Tune `baseYawKv` until yaw tracking is responsive but not oscillatory.
4. Tune `armJointKv` last, because arm inertia and coupling make it the most sensitive.

Practical rules:

- if the base jitters or overshoots, lower `baseLinearKv` / `baseYawKv`
- if the base feels too soft or lags badly, raise them gradually
- if the arm chatters, lower `armJointKv`
- if the arm is too sluggish, raise `armJointKv` gradually

Important:

- OCS2 command targets are clamped to the velocity limits in `task.info`
- the current MuJoCo base is a planar abstraction, not wheel-ground contact dynamics
- visual meshes are loaded from `g7_openarm_description/meshes`, while collision/stability still rely on the simplified first-pass setup

### 0.5 Mesh Decoder Note

When using the Python-packaged MuJoCo, STL/OBJ decoders are provided by bundled plugins.
The C++ MuJoCo node now explicitly loads:

- `${python_mujoco_package}/plugin`

before calling `mj_loadXML()`.

This means the original STL/OBJ assets are usable again, and the previous
`no decoder found for mesh file ...` error should not reappear unless the
MuJoCo plugin directory is missing from the runtime installation.


## 1. Overview

The goal of this integration is not just to display the robot in RViz. The full pipeline is:

- `g7_openarm.urdf` is used consistently by `robot_state_publisher`, Pinocchio, and OCS2
- the `WheelBasedMobileManipulator` dynamics can switch between native OCS2 dynamics and a PinnZoo-generated dynamics library
- `g7_openarm_pinnzoo.launch.py` starts RViz, the interactive marker, MPC, and the dummy MRT loop together
- the interactive marker controls the gripper center, not a wrist frame or a single finger link

There are currently two runtime modes:

- standard OCS2 wheel-based dynamics
- PinnZoo-backed wheel-based dynamics

The switch is done through the task file, not by hardcoding a different C++ build:

- [`task.info`](g7_openarm_ocs2/config/g7_openarm/task.info): native OCS2 dynamics
- [`task_pinnzoo.info`](g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info): PinnZoo backend enabled

Important note:

- the OCS2 code changes described in Section 3 are only needed for the PinnZoo backend
- the default flow still works with the original OCS2/Pinocchio-based dynamics path

For experienced OCS2 users:

- if a new robot still fits the existing `ocs2_mobile_manipulator` model, the usual work is `URDF + task file + launch/config`; upstream OCS2 source changes are typically unnecessary
- this project changes OCS2 because it adds a new dynamics backend, not just a new robot
- the added OCS2 code is limited to loading a PinnZoo-generated `.so`, mapping reduced OCS2 state/input to the full PinnZoo model, and switching between native and PinnZoo-backed dynamics at runtime

## 2. Package Layout

### `g7_openarm_description`

Purpose:

- stores the robot URDF
- stores all visual and collision meshes
- provides the shared robot description for `robot_state_publisher`, RViz, and Pinocchio

Main files:

- [`g7_openarm_description/urdf/g7_openarm.urdf`](g7_openarm_description/urdf/g7_openarm.urdf)
- [`g7_openarm_description/CMakeLists.txt`](g7_openarm_description/CMakeLists.txt)
- [`g7_openarm_description/package.xml`](g7_openarm_description/package.xml)
- `g7_openarm_description/meshes/*`

### `g7_openarm_ocs2`

Purpose:

- stores the `g7_openarm` task and configuration files
- stores the `auto_generated` OCS2 helper-library folder used by `libFolder`
- no longer contains the PinnZoo adapter implementation itself; the runtime PinnZoo integration now lives in upstream [`ocs2_mobile_manipulator`](../basic%20examples/ocs2_mobile_manipulator)

Main files:

- [`g7_openarm_ocs2/config/g7_openarm/task.info`](g7_openarm_ocs2/config/g7_openarm/task.info)
- [`g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info`](g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info)
- [`g7_openarm_ocs2/auto_generated/g7_openarm/README.md`](g7_openarm_ocs2/auto_generated/g7_openarm/README.md)
- [`g7_openarm_ocs2/CMakeLists.txt`](g7_openarm_ocs2/CMakeLists.txt)
- [`g7_openarm_ocs2/package.xml`](g7_openarm_ocs2/package.xml)

### `g7_openarm_ros`

Purpose:

- provides the `g7_openarm` ROS 2 launch entrypoints
- provides the RViz layout
- wires the `g7_openarm` URDF, task file, and interactive-marker parameters into `ocs2_mobile_manipulator_ros`

Main files:

- [`g7_openarm_ros/launch/g7_openarm.launch.py`](g7_openarm_ros/launch/g7_openarm.launch.py)
- [`g7_openarm_ros/launch/g7_openarm_pinnzoo.launch.py`](g7_openarm_ros/launch/g7_openarm_pinnzoo.launch.py)
- [`g7_openarm_ros/launch/include/g7_openarm_bringup.launch.py`](g7_openarm_ros/launch/include/g7_openarm_bringup.launch.py)
- [`g7_openarm_ros/rviz/g7_openarm.rviz`](g7_openarm_ros/rviz/g7_openarm.rviz)
- [`g7_openarm_ros/CMakeLists.txt`](g7_openarm_ros/CMakeLists.txt)
- [`g7_openarm_ros/package.xml`](g7_openarm_ros/package.xml)

## 3. OCS2 Code Changes

This section describes the changes made to upstream OCS2 packages outside the `g7_openarm` folder.
All links point to the exact files in this fork so that users can inspect the implementation directly.

### 3.0 Code Change Map

Main upstream files changed for the PinnZoo integration:

- [`ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/dynamics/PinnzooInterface.h`](../basic%20examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/dynamics/PinnzooInterface.h): shared-library wrapper API and task-file settings
- [`ocs2_mobile_manipulator/src/dynamics/PinnzooInterface.cpp`](../basic%20examples/ocs2_mobile_manipulator/src/dynamics/PinnzooInterface.cpp): config parsing, environment-variable expansion, and `dlopen` / `dlsym` loading
- [`ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/dynamics/PinnzooWheelBasedMobileManipulatorDynamics.h`](../basic%20examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/dynamics/PinnzooWheelBasedMobileManipulatorDynamics.h): PinnZoo-backed wheel-based dynamics class declaration
- [`ocs2_mobile_manipulator/src/dynamics/PinnzooWheelBasedMobileManipulatorDynamics.cpp`](../basic%20examples/ocs2_mobile_manipulator/src/dynamics/PinnzooWheelBasedMobileManipulatorDynamics.cpp): reduced-to-full state mapping and flow-map evaluation through PinnZoo
- [`ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp`](../basic%20examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp): runtime backend switch between native OCS2 and PinnZoo
- [`ocs2_mobile_manipulator/CMakeLists.txt`](../basic%20examples/ocs2_mobile_manipulator/CMakeLists.txt): build-system wiring for the new sources and `dl`
- [`ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp`](../basic%20examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp): safer launch-parameter handling for the interactive target
- [`ocs2_mobile_manipulator_ros/launch/include/mobile_manipulator.launch.py`](../basic%20examples/ocs2_mobile_manipulator_ros/launch/include/mobile_manipulator.launch.py): shared launch behavior used by the interactive-target flow

### 3.1 Changes in `ocs2_mobile_manipulator`

#### Added [`PinnzooInterface.h`](../basic%20examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/dynamics/PinnzooInterface.h)

What it does:

- defines `PinnzooSettings`
- declares `loadPinnzooSettings()`
- wraps the generated PinnZoo shared library interface
- exposes config-order, velocity-order, and torque-order lookup utilities

This file is the C++ wrapper interface around the external PinnZoo `.so`.

#### Added [`PinnzooInterface.cpp`](../basic%20examples/ocs2_mobile_manipulator/src/dynamics/PinnzooInterface.cpp)

What it does:

- reads the `pinnzoo { ... }` block from the task file
- loads the generated PinnZoo symbols with `dlopen` / `dlsym`
- resolves the following generated entry points:
  - `dynamics_wrapper`
  - `velocity_kinematics_wrapper`
  - `get_config_order`
  - `get_vel_order`
  - `get_torque_order`
  - `get_urdf_path`
- expands `PINNZOO_LIBRARY_PATH` and other environment-variable references in `libraryPath`

This is what makes the task file portable instead of hardcoding a local absolute path.

#### Added [`PinnzooWheelBasedMobileManipulatorDynamics.h`](../basic%20examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/dynamics/PinnzooWheelBasedMobileManipulatorDynamics.h)

What it does:

- declares the PinnZoo-backed wheel-based dynamics class
- defines the interface for mapping the reduced OCS2 state/input to the full PinnZoo state

#### Added [`PinnzooWheelBasedMobileManipulatorDynamics.cpp`](../basic%20examples/ocs2_mobile_manipulator/src/dynamics/PinnzooWheelBasedMobileManipulatorDynamics.cpp)

What it does:

- maps the reduced OCS2 state
  - `x, y, yaw, L_1..L_7, R_1..R_7`
  into the full floating-base PinnZoo state
- converts the mobile-base heading into a quaternion
- fixes the joints that are removed from the reduced model
- uses `velocity_kinematics_wrapper` to build the flow map used by OCS2
- validates the generated model dimensions and wrapper availability at startup

This file is the core of the PinnZoo backend integration.

#### Modified [`MobileManipulatorInterface.cpp`](../basic%20examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp)

What changed:

- reads `pinnzoo.enabled`
- only allows the PinnZoo backend for `WheelBasedMobileManipulator`
- when `pinnzoo.enabled == true`
  - constructs `PinnzooWheelBasedMobileManipulatorDynamics`
  - wraps it with `SystemDynamicsLinearizer`
- when `pinnzoo.enabled == false`
  - keeps using the original `WheelBasedMobileManipulatorDynamics`

This preserves the original OCS2 behavior while adding a switchable alternative backend.

#### Modified [`CMakeLists.txt`](../basic%20examples/ocs2_mobile_manipulator/CMakeLists.txt)

What changed:

- compiles the new PinnZoo dynamics sources into `ocs2_mobile_manipulator`
- links `dl`

### 3.2 Changes in `ocs2_mobile_manipulator_ros`

#### Modified [`MobileManipulatorTarget.cpp`](../basic%20examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp)

What changed:

- added safer handling for `enableJoystick`
- added safer handling for `enableAutoPosition`
- added fallback handling when the parameter type is not what the node expects

Why this matters:

- without this change, the interactive target could crash under some launch parameter combinations
- with this change, the target node is more robust and the interactive marker is less likely to die at startup

#### Modified [`mobile_manipulator.launch.py`](../basic%20examples/ocs2_mobile_manipulator_ros/launch/include/mobile_manipulator.launch.py)

What changed:

- added display-aware terminal-prefix handling
- preserved the wiring of `enableJoystick` and `enableAutoPosition`

This is not `g7_openarm`-specific, but it affects the behavior of the shared interactive-target flow.

## 4. What Each `g7_openarm` File Does

This section only covers files under [`g7_openarm`](.).

### 4.1 `g7_openarm_description`

#### [`g7_openarm_description/urdf/g7_openarm.urdf`](g7_openarm_description/urdf/g7_openarm.urdf)

Defines:

- the full kinematic tree of the dual-arm mobile manipulator
- `AMR_base_link`
- the fixed `openarm_base_link`
- the left and right 7-DoF arms
- the left and right gripper finger joints
- the virtual tool-center frames `L_ee_link` and `R_ee_link`




### 4.2 `g7_openarm_ocs2`

#### [`g7_openarm_ocs2/config/g7_openarm/task.info`](g7_openarm_ocs2/config/g7_openarm/task.info)

Purpose:

- standard OCS2 task file
- uses the native OCS2 wheel-based dynamics
- defines reduced-model joint order, costs, limits, and end-effector frames

#### [`g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info`](g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info)

Purpose:

- PinnZoo-enabled task file
- sets `pinnzoo.enabled true`
- defines the generated wrapper symbol names
- defines `baseHeight`
- references the external PinnZoo `.so` through `PINNZOO_LIBRARY_PATH`

This is the single switch that enables the PinnZoo backend.

### 4.3 `g7_openarm_ros`

#### [`g7_openarm_ros/launch/g7_openarm.launch.py`](g7_openarm_ros/launch/g7_openarm.launch.py)

Purpose:

- standard `g7_openarm` launch entrypoint
- uses [`task.info`](g7_openarm_ocs2/config/g7_openarm/task.info) by default
- passes `rviz`, `debug`, `urdfFile`, `taskFile`, `libFolder`, and interactive-target parameters downward

#### [`g7_openarm_ros/launch/g7_openarm_pinnzoo.launch.py`](g7_openarm_ros/launch/g7_openarm_pinnzoo.launch.py)

Purpose:

- PinnZoo launch entrypoint
- uses [`task_pinnzoo.info`](g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info) by default
- this is the main public launch file for the PinnZoo-backed workflow

#### [`g7_openarm_ros/launch/include/g7_openarm_bringup.launch.py`](g7_openarm_ros/launch/include/g7_openarm_bringup.launch.py)

Purpose:

- shared bringup used by the top-level launch files
- starts:
  - `robot_state_publisher`
  - `mobile_manipulator_mpc_node`
  - `mobile_manipulator_dummy_mrt_node`
  - `mobile_manipulator_target`
- wires `enableJoystick`, `enableAutoPosition`, and `enableDynamicFrame` into the interactive-target node

## 5. Reduced-Order OCS2 Model Assumptions

- `manipulatorModelType = 1`
- `baseFrame = AMR_base_link`
- `eeFrame = L_ee_link`
- `eeFrame1 = R_ee_link`
- reduced state order:
  - `x, y, yaw, L_1..L_7, R_1..R_7`
- reduced input order:
  - `forward velocity, yaw velocity, L_1..L_7 velocity, R_1..R_7 velocity`
- wheel joints and gripper finger joints are removed from the reduced OCS2 model
- the wheel and gripper joints still exist in the full PinnZoo model, but they are fixed during reduced-to-full state mapping

## 6. Build and Launch

### Build

```bash
cd /root/ocs2_ws
source /opt/ros/humble/setup.bash
AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 PYTHON_EXECUTABLE=/usr/bin/python3 \
  colcon build --packages-select \
    ocs2_mobile_manipulator \
    ocs2_mobile_manipulator_ros \
    g7_openarm_description \
    g7_openarm_ocs2 \
    g7_openarm_ros
source /root/ocs2_ws/install/setup.bash
```

### Build PinnZoo

```bash
cd ~/workspace
git clone -b g7-openarm https://github.com/ethanCSL/PinnZoo.git
cd PinnZoo
mkdir -p build
cd build
cmake ..
cmake --build . --target g7_openarm_quat
export PINNZOO_LIBRARY_PATH=~/workspace/PinnZoo/build/libg7_openarm_quat.so
```

### Launch

Standard OCS2 dynamics:

```bash
ros2 launch g7_openarm_ros g7_openarm.launch.py
```

PinnZoo-backed dynamics:

```bash
ros2 launch g7_openarm_ros g7_openarm_pinnzoo.launch.py
```

## 7. URDF Changes

This section lists the URDF-level changes that are important for the OCS2 integration.

### 7.1 Fixed mounting between the mobile base and the arm body

Key items:

- `openarm_base_link`
- `openarm_base_link_joint`
- `chest_link`
- `chest_link_joint`

Why it was added:

- to mount the upper-body arm assembly onto `AMR_base_link`
- to keep the full robot as a single URDF tree
- to align the robot description with `baseFrame = AMR_base_link`

### 7.2 Kept the gripper finger joints in the full URDF

Key joints:

- `gripper_LL_joint`
- `gripper_LR_joint`
- `gripper_RL_joint`
- `gripper_RR_joint`

Why this matters:

- the gripper geometry is preserved for visualization and the full model
- the finger joints are still removed from the reduced OCS2 optimization model

### 7.3 Added virtual tool-center frames

Key links and joints:

- `L_ee_link`
- `L_ee_joint`
- `R_ee_link`
- `R_ee_joint`

Why this matters:

- a fixed tool-center frame is placed at the center of each gripper opening
- `eeFrame` / `eeFrame1` now point to the gripper center instead of a wrist link or a single finger
- the interactive marker controls the actual gripper center

Current offsets:

- left arm: `L_link7 -> L_ee_link`, `xyz="0 -0.03175 -0.119"`
- right arm: `R_link7 -> R_ee_link`, `xyz="0 0.03175 -0.119"`

### 7.4 Frame naming aligned with the task files

The URDF naming is aligned with the task configuration:

- `baseFrame = AMR_base_link`
- `eeFrame = L_ee_link`
- `eeFrame1 = R_ee_link`

This alignment is required for Pinocchio frame lookup, end-effector constraints, and interactive-marker targeting to work consistently.

## 8. Important Usage Notes

- if you want the PinnZoo backend, you must export `PINNZOO_LIBRARY_PATH` first
- [`task_pinnzoo.info`](g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info) expects the external PinnZoo `.so`, not the `auto_generated/g7_openarm` folder
- if RViz shows the robot but no draggable marker, check whether `mobile_manipulator_target` started correctly
- if the interactive marker should control the gripper center, `eeFrame` / `eeFrame1` must remain `L_ee_link` / `R_ee_link`
