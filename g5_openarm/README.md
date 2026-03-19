# g5_openarm Project Guide

This workspace contains the `g5_openarm` description, OCS2 task files, and ROS 2 launch entrypoints for the wheel-based dual-arm mobile manipulator. The project now supports two runtime modes:

- standard OCS2 wheel-based dynamics
- PinnZoo-backed wheel-based dynamics selected through `task_pinnzoo.info`

All launch entrypoints share the same RViz layout and interactive end-effector target workflow.

## Package layout

### `g5_openarm_description`

- `urdf/g5_openarm.urdf`: robot model used by `robot_state_publisher`, Pinocchio, and RViz
- `meshes/`: visual and collision meshes referenced by the URDF

Edit this package when link geometry, meshes, frame names, or joint names change.

### `g5_openarm_ocs2`

- `config/g5_openarm/task_startup.info`: conservative startup tuning
- `config/g5_openarm/task.info`: normal tuning
- `config/g5_openarm/task_pinnzoo.info`: PinnZoo-backed tuning and library path
- `auto_generated/g5_openarm/`: generated OCS2 helper library folder used by `libFolder`

This package now acts as a clean config asset package. The actual PinnZoo runtime integration lives in `ocs2_mobile_manipulator`, while `g5_openarm_ocs2` only owns task/config data and generated-library assets.

### `g5_openarm_ros`

- `launch/g5_openarm.launch.py`: compatibility entry, defaults to `task_startup.info`
- `launch/g5_openarm_startup.launch.py`: explicit startup-safe entry
- `launch/g5_openarm_normal.launch.py`: explicit normal-tuning entry
- `launch/g5_openarm_pinnzoo.launch.py`: PinnZoo-backed entry
- `launch/include/g5_openarm_bringup.launch.py`: shared internal bring-up
- `rviz/g5_openarm.rviz`: RViz config with robot model, trajectories, and interactive markers

All top-level launch files now expose the same user-facing arguments:

- `rviz`
- `debug`
- `urdfFile`
- `taskFile`
- `libFolder`
- `rvizconfig`
- `enableJoystick`
- `enableAutoPosition`
- `enableDynamicFrame`

## Build

Rebuild the OCS2 mobile manipulator packages together with `g5_openarm`, because the PinnZoo dynamics backend lives in `ocs2_mobile_manipulator`:

```bash
cd /root/ocs2_ws
source /opt/ros/humble/setup.bash
AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 PYTHON_EXECUTABLE=/usr/bin/python3 \
  colcon build --packages-select \
    ocs2_mobile_manipulator \
    ocs2_mobile_manipulator_ros \
    g5_openarm_description \
    g5_openarm_ocs2 \
    g5_openarm_ros
```

After building:

```bash
source /root/ocs2_ws/install/setup.bash
```

## PinnZoo setup

Clone and build PinnZoo separately:

```bash
cd ~/workspace
git clone -b g7-openarm https://github.com/ethanCSL/PinnZoo.git
cd PinnZoo
mkdir -p build
cd build
cmake ..
cmake --build . --target g7_openarm_quat
```

Export the generated shared library path before launching the PinnZoo workflow:

```bash
export PINNZOO_LIBRARY_PATH=~/workspace/PinnZoo/build/libg7_openarm_quat.so
```

## Launch


Auto-generated launch:

```bash
ros2 launch g5_openarm_ros g5_openarm.launch.py
```

PinnZoo launch:

```bash
ros2 launch g5_openarm_ros g5_openarm_pinnzoo.launch.py
```

## Current model assumptions

- `manipulatorModelType = 1`
- `baseFrame = AMR_base_link`
- `eeFrame = L_ee_link`
- `eeFrame1 = R_ee_link`
- reduced state order is `x, y, yaw, L_1..L_7, R_1..R_7`
- wheel joints and gripper finger joints are removed from the reduced OCS2 model

The URDF provides `L_ee_link` and `R_ee_link` as tool-center frames, so dragging the interactive target controls the gripper center rather than a wrist link or a single finger.

## PinnZoo notes

- `task_pinnzoo.info` enables the PinnZoo backend with `pinnzoo.enabled true`
- the external PinnZoo dynamics library path is read from `PINNZOO_LIBRARY_PATH`
- `libFolder` still points to `g5_openarm_ocs2/auto_generated/g5_openarm`, which is the OCS2 helper-library folder, not the PinnZoo `.so`

If the external PinnZoo library location changes, update `PINNZOO_LIBRARY_PATH` before launch.
