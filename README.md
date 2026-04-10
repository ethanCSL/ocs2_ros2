ROS 2 port of [OCS2](https://github.com/leggedrobotics/ocs2) with modern CMake support.

This fork adds a ready-to-run `g7_openarm` workflow, including robot description, launch files, RViz config, a MuJoCo-backed `ros2_control` hardware interface, and a closed-loop OCS2 bringup entrypoint.

## Supported Platforms

- Ubuntu 22.04 + ROS 2 Humble
- Ubuntu 24.04 + ROS 2 Jazzy

## Quick Start for `g7_openarm`

```bash
cd ~/ros2_ws/src
git clone https://github.com/ethanCSL/ocs2_ros2.git
cd ocs2_ros2
git submodule update --init --recursive

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install -y python3-pip
python3 -m pip install --user mujoco

colcon build --packages-up-to g7_openarm_ros g7_openarm_ros2_control --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 launch g7_openarm_ros g7_openarm.launch.py