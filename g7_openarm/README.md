# g7_openarm

`g7_openarm` is the mobile-manipulator integration layer for this ROS 2 Humble + OCS2 workspace.

The runtime path in this package is:

`OCS2 MPC -> ros2_control -> g7_openarm_hardware_interface -> MuJoCo`

This README is intended to match the workspace under `/root/ocs2_ws/src/g7_openarm`.

## Package Layout

- `g7_openarm_description`
  Robot URDF, MuJoCo MJCF, viewer scene, meshes, and utility scripts.
- `g7_openarm_ocs2`
  OCS2 task configuration and auto-generated helper libraries.
- `g7_openarm_ros`
  Top-level launch files, RViz config, and the MuJoCo viewer script.
- `g7_openarm_ros2_control`
  `ros2_control` bringup, overlay URDF, controller config, and MPC reset coordinator.
- `g7_openarm_hardware_interface`
  MuJoCo-backed hardware plugin used by `ros2_control`.
- `g7_openarm_controllers`
  Policy controller and observation broadcaster.

## Quick Start

```bash
cd /root/ocs2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to g7_openarm_ros g7_openarm_ros2_control
source install/setup.bash
ros2 launch g7_openarm_ros g7_openarm.launch.py
```

After startup settles, the usual signs of life are:

- `/mobile_manipulator_mpc_observation` is publishing.
- `/joint_states`, `/tf`, and `/clock` keep updating.
- RViz shows the robot model.
- If `mujocoViewer:=true`, the MuJoCo viewer follows the observation state.

## Common Launch Commands

Build:
```
cd /root/ocs2_ws
colcon build --packages-select g7_openarm_description g7_openarm_ros g7_openarm_ros2_control g7_openarm_controllers g7_openarm_hardware_interface --symlink-install
source /root/ocs2_ws/install/setup.bash
```

Default bringup:

```bash
ros2 launch g7_openarm_ros g7_openarm.launch.py
```

Headless bringup:

```bash
ros2 launch g7_openarm_ros g7_openarm.launch.py rviz:=false mujocoViewer:=false
```

PinnZoo-backed task(test):

```bash
export PINNZOO_LIBRARY_PATH=/abs/path/to/libg7_openarm_quat.so
ros2 launch g7_openarm_ros g7_openarm_pinnzoo.launch.py
```

## Runtime Architecture

1. `ocs2_mobile_manipulator_ros/mobile_manipulator_mpc_node` publishes `/mobile_manipulator_mpc_policy`.
2. `controller_manager/ros2_control_node` loads `g7_openarm_policy_controller` and `g7_openarm_observation_broadcaster`.
3. `g7_openarm_controllers/G7OpenarmPolicyController` evaluates the latest MPC policy.
4. `g7_openarm_hardware_interface/G7OpenarmMujocoSystem` advances the MuJoCo simulation.
5. `g7_openarm_controllers/G7OpenarmObservationBroadcaster` publishes:
   - `/mobile_manipulator_mpc_observation`
   - `/joint_states`
   - `/tf`
   - `/clock`
   - `/mujoco/debug_observation`
   - `/mujoco/debug_input`
   - `/mujoco/debug_base_pose`
6. `g7_openarm_ros2_control/g7_openarm_mpc_reset_coordinator` waits for the first valid observation and calls `/mobile_manipulator_mpc_reset`.
7. If `rviz:=true`, `ocs2_mobile_manipulator_ros/mobile_manipulator_target` provides the interactive target tool in RViz.
8. If `mujocoViewer:=true`, `g7_openarm_mujoco_viewer.py` opens a passive viewer driven by `/mobile_manipulator_mpc_observation`.

## Key Files

- `g7_openarm_ros/launch/g7_openarm.launch.py`
  Default bringup entry point.
- `g7_openarm_ros/launch/g7_openarm_pinnzoo.launch.py`
  Bringup entry point using `task_pinnzoo.info`.
- `g7_openarm_ros/launch/include/g7_openarm_bringup.launch.py`
  Shared launch graph used by both top-level launch files.
- `g7_openarm_ros2_control/launch/g7_openarm_ros2_control.launch.py`
  Starts `ros2_control_node` and spawns the controllers.
- `g7_openarm_description/urdf/g7_openarm.urdf`
  Robot model used by `robot_state_publisher` and OCS2.
- `g7_openarm_description/urdf/g7_openarm.MJCF`
  MuJoCo model used by the runtime simulation.
- `g7_openarm_description/urdf/g7_openarm_viewer_scene.xml`
  Viewer scene that includes `g7_openarm.MJCF`.
- `g7_openarm_ros/scripts/g7_openarm_mujoco_viewer.py`
  Observation-driven MuJoCo viewer.

## Important Topics And Services

- Policy input: `/mobile_manipulator_mpc_policy`
- Observation feedback: `/mobile_manipulator_mpc_observation`
- Reset service: `/mobile_manipulator_mpc_reset`
- Sim time: `/clock`
- RViz joint state input: `/joint_states`
- Debug observation: `/mujoco/debug_observation`
- Debug input: `/mujoco/debug_input`
- Debug base pose: `/mujoco/debug_base_pose`
- Base TF root: `world -> AMR_base_link`

## Launch Arguments You Will Likely Touch

From `g7_openarm.launch.py`:

- `rviz`
  Open RViz.
- `mujocoViewer`
  Open the observation-driven MuJoCo viewer.
- `taskFile`
  OCS2 task config file.
- `viewerMjcfFile`
  Viewer scene MJCF, defaulting to `g7_openarm_viewer_scene.xml`.
- `dtSim`
  MuJoCo simulation step size used by `ros2_control`.
- `baseLinearKv`
  Velocity-servo gain for the mobile base planar joints.
- `baseYawKv`
  Velocity-servo gain for base yaw.
- `armJointKv`
  Velocity-servo gain for arm joints.
- `enableJoystick`
  Enable joystick control for the interactive target tool.
- `enableAutoPosition`
  Auto-follow the end-effector pose with the target marker.
- `enableDynamicFrame`
  Choose the target marker frame from the task file.

## Build And Environment Notes

Expected environment:

- Ubuntu 22.04
- ROS 2 Humble
- Python MuJoCo package available in the same shell used for build and launch
- `controller_manager`, `robot_state_publisher`, and `rviz2` installed

Useful setup flow:

```bash
cd /root/ocs2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -c "import mujoco; print(mujoco.__file__)"
```

If the MuJoCo import fails, `g7_openarm_ros` and `g7_openarm_hardware_interface` will not configure correctly.

## PinnZoo Notes

`g7_openarm_pinnzoo.launch.py` uses:

- `g7_openarm_ocs2/config/g7_openarm/task_pinnzoo.info`
- `PINNZOO_LIBRARY_PATH`

Expected usage:

```bash
export PINNZOO_LIBRARY_PATH=/abs/path/to/libg7_openarm_quat.so
ros2 launch g7_openarm_ros g7_openarm_pinnzoo.launch.py
```

If `PINNZOO_LIBRARY_PATH` is unset or points to the wrong library, the PinnZoo path will not start correctly.

## Viewer Notes

`g7_openarm_mujoco_viewer.py` does not run the simulation itself. It subscribes to `/mobile_manipulator_mpc_observation` and updates a separate viewer scene from that state.

That means:

- `g7_openarm.MJCF` is the runtime simulation model.
- `g7_openarm_viewer_scene.xml` is the visualization scene for the passive viewer.
- If `/mobile_manipulator_mpc_observation` stops updating, the viewer will appear frozen.

## Troubleshooting

If RViz is open but the robot looks frozen:

- Check `/clock`, `/joint_states`, and `/tf`.
- Check whether `/mobile_manipulator_mpc_observation` is still publishing.

If the MuJoCo viewer opens but does not move:

- Check whether the viewer is using the expected `viewerMjcfFile`.
- Check whether `/mobile_manipulator_mpc_observation` has valid messages.

If the first motion never starts:

- Watch `g7_openarm_mpc_reset_coordinator`.
- Make sure `/mobile_manipulator_mpc_reset` is available.

If build fails around generated libraries:

- Rebuild with `--packages-up-to g7_openarm_ros g7_openarm_ros2_control`.
- The first launch can take longer because OCS2 may generate and compile helper libraries under `g7_openarm_ocs2/auto_generated/g7_openarm`.
