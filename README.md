# Nav2 NeuPAN Controller Plugin

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/nav2_neupan_controller/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/LihanChen2004/nav2_neupan_controller/actions/workflows/build_and_test.yml)

This package provides a Nav2 local controller plugin based on NeuPAN. It implements the `nav2_core::Controller` interface and can run inside `controller_server` as a local trajectory planner.

The controller delegates trajectory generation to NeuPAN (Python side), converts planner outputs to ROS velocity commands, and publishes visualization topics for debugging and analysis.

> [!NOTE]
>
> This project has only been tested in **ROS 2 Jazzy** with **Python 3.12**.

## Features

- NeuPAN integration through embedded Python interpreter.
- Supports multiple robot models:
  - `diff`
  - `acker`
  - `omni`
- RViz-friendly visualization of:
  - optimized states
  - reference states
  - initial path
  - DUNE/NRMP points

## Install NeuPAN First

Please install NeuPAN before using this plugin:

```bash
cd ~/Programs
git clone https://github.com/LihanChen2004/NeuPAN.git
cd NeuPAN
```

Then install NeuPAN into your Python 3.12 environment according to the NeuPAN repository instructions.

```bash
pip install -e . --break-system-packages
```

## Configuration

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `max_linear_velocity` | double | `1.0` | Maximum absolute linear velocity for x/y clamp. |
| `max_angular_velocity` | double | `1.5` | Maximum absolute angular velocity clamp. |
| `robot_type` | string | `acker` | Robot kinematics mode: `diff`, `acker`, `omni`. |
| `neupan_config_path` | string | `""` | Path to NeuPAN YAML config file (required). |
| `dune_model_path` | string | `""` | DUNE model checkpoint path. |
| `marker_size` | double | `0.05` | Marker cube size for point visualization. |
| `marker_z` | double | `1.0` | Robot marker Z scale. |

### Example Controller Server Config

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "nav2_neupan_controller::NeuPANController"
      dune_model_path: ""         # $(find-pkg-share MY_ROS_PACKAGE)/params/model_5000.pth
      neupan_config_path: ""      # $(find-pkg-share MY_ROS_PACKAGE)/params/planner.yaml
      max_linear_velocity: 1.0
      max_angular_velocity: 1.5
      robot_type: "acker"
      marker_size: 0.05
      marker_z: 1.0
```

## Topics

| Topic | Type | Description |
| --- | --- | --- |
| `neupan_plan` | `nav_msgs/msg/Path` | Optimized states returned by NeuPAN (`opt_state_list`). |
| `neupan_ref_state` | `nav_msgs/msg/Path` | Reference states from NeuPAN (`ref_state_list`). |
| `neupan_initial_path` | `nav_msgs/msg/Path` | NeuPAN initial path used by planner. |
| `dune_point_markers` | `visualization_msgs/msg/MarkerArray` | DUNE points visualization. |
| `nrmp_point_markers` | `visualization_msgs/msg/MarkerArray` | NRMP points visualization. |
| `robot_marker` | `visualization_msgs/msg/Marker` | Robot body marker (rectangle model). |

## Notes for Users

- `neupan_config_path` must be set, otherwise initialization fails.
- The plugin dynamically updates parameters at runtime for all listed parameters.
- If no obstacle points are extracted from costmap, the controller still runs and tracks path.
- For Ackermann mode, angular velocity is computed from steering angle and wheelbase:

  \[
  \omega = \frac{v}{L} \tan(\delta)
  \]

- Velocity outputs are always clamped by configured limits before publishing.

## Build

In your workspace root:

```bash
./build.sh
```

## Copyright and Licensing

nav2_neupan_controller is provided under Apache License 2.0.

The upstream [NeuPAN](ttps://github.com/hanruihua/NeuPAN.git) vendor repository uses GPL-3.0. All rights and license terms of that upstream project are reserved and must be respected.
