
# g7_openarm

`g7_openarm` is the mobile-manipulator bringup in this ROS 2 OCS2 workspace.

Runtime path:

`OCS2 MPC -> ros2_control -> g7_openarm_hardware_interface -> MuJoCo`

## Supported Environment

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MuJoCo Python package installed in the same Python environment used for build and launch

## Workspace Setup

Create a workspace and clone the repository:

```bash
cd ~
mkdir -p ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ethanCSL/ocs2_ros2.git
cd ocs2_ros2
git submodule update --init --recursive
```

Install dependencies:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install -y python3-pip
python3 -m pip install --user mujoco
python3 -c "import mujoco; print(mujoco.__version__)"
```

The `mujoco` Python package is required. Without it, `g7_openarm_hardware_interface` will not configure or build.

## Build

Build the packages needed for the default bringup:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to g7_openarm_ros g7_openarm_ros2_control --symlink-install
source ~/ros2_ws/install/setup.bash
```

If you only want to rebuild the bringup-related packages after the workspace already built once:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select \
  g7_openarm_hardware_interface \
  g7_openarm_controllers \
  g7_openarm_ros2_control \
  g7_openarm_ros \
  --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Launch

Default bringup:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch g7_openarm_ros g7_openarm.launch.py
```

Headless bringup:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch g7_openarm_ros g7_openarm.launch.py rviz:=false mujocoViewer:=false
```

The headless command is the safest first validation on a new machine.

## Expected Bringup Result

After startup settles, the following should happen:

- `g7_openarm_policy_controller` loads and activates
- `g7_openarm_observation_broadcaster` loads and activates
- `g7_openarm_mpc_reset_coordinator` completes the initial MPC reset
- `/mobile_manipulator_mpc_observation` is publishing
- `/joint_states`, `/tf`, and `/clock` keep updating
- RViz shows the robot model when `rviz:=true`
- The passive MuJoCo viewer follows the observation state when `mujocoViewer:=true`

Useful checks:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | rg "mobile_manipulator_mpc_observation|joint_states|clock"
ros2 control list_controllers
```

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
- `g7_openarm_ros/launch/include/g7_openarm_bringup.launch.py`
  Shared launch graph used by the top-level launch file.
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

## Launch Arguments You Will Likely Touch

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

## Notes

- The first launch can take longer because OCS2 may generate and compile helper libraries under `g7_openarm_ocs2/auto_generated/g7_openarm`.
- `g7_openarm_mujoco_viewer.py` is only a viewer. It does not run the simulation itself.
- `g7_openarm.MJCF` is the runtime simulation model.
- `g7_openarm_viewer_scene.xml` is the visualization scene used by the passive viewer.

## Troubleshooting

If build fails before compiling `g7_openarm_hardware_interface`:

- Check `python3 -c "import mujoco; print(mujoco.__file__)"`
- Make sure the same shell can both `import mujoco` and run `colcon build`

If bringup starts but motion never begins:

- Check whether `g7_openarm_observation_broadcaster` is active
- Check whether `/mobile_manipulator_mpc_observation` is publishing
- Check whether `/mobile_manipulator_mpc_reset` is available

If RViz opens but the robot looks frozen:

- Check `/clock`, `/joint_states`, and `/tf`
- Check whether `/mobile_manipulator_mpc_observation` is still publishing

If the MuJoCo viewer opens but does not move:

- Check whether `/mobile_manipulator_mpc_observation` is updating
- Check whether the viewer is using the expected `viewerMjcfFile`