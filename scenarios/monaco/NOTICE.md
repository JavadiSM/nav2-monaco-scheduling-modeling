# Upstream attribution

`racecar.sdf` and `racecar.urdf` are generated from the installed
`nav2_minimal_tb3_sim` package (version 1.0.1), specifically
`urdf/gz_waffle.sdf.xacro` and `urdf/turtlebot3_waffle.urdf`.
Their visual geometry is modified; sensors, inertial data, collisions, joints
and drive plugins are retained. Upstream package license: Apache-2.0.
Source: https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation

`nav2_params.yaml` is generated from `nav2_bringup/params/nav2_params.yaml`
(Nav2 1.3.13), with scenario-specific parameter changes documented in
`docs/monaco-scenario.md`. Upstream license: Apache-2.0.
Source: https://github.com/ros-navigation/navigation2

`config/monaco-gui.config` is adapted from the Gazebo Sim 8 GUI template
provided by `ros-jazzy-gz-sim-vendor`. Camera, window and panel settings are
modified. Gazebo source is maintained by Open Source Robotics Foundation
and contributors under Apache-2.0.
Source: https://github.com/gazebosim/gz-sim

The full Apache-2.0 license is included in `LICENSE.upstream`.
These upstream-derived files are not covered by the root project MIT license.
The original track design, map, checkpoint placement and project scripts are
covered by the root MIT license.

`navigate_through_poses.xml` is derived from the installed Navigation2 `nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` (Apache-2.0). Only its replanning RateController parameter is changed to 1 Hz; the upstream navigation and recovery nodes are retained.
