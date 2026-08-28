# RescueBot 🤖

Autonomous 4-wheeled search-and-rescue robot built on **ROS2 Humble**, with a custom `ros2_control` hardware interface talking to Arduino firmware over a serial protocol. The stack combines mecanum-drive locomotion, EKF sensor fusion, SLAM-based mapping, Nav2 autonomous navigation, frontier exploration, and AprilTag-based target detection — with the goal of autonomously mapping unknown environments and locating tagged targets within them.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Package Reference](#package-reference)
- [Hardware](#hardware)
- [Requirements](#requirements)
- [Build](#build)
- [Usage](#usage)
- [Topics, Frames & Data Flow](#topics-frames--data-flow)
- [Target Detection & Logging](#target-detection--logging)
- [Known Issues / Work in Progress](#known-issues--work-in-progress)
- [License](#license)

---

## Overview

RescueBot is a competition/research robot designed to autonomously explore and map unknown environments, then locate and record the position of tagged targets (AprilTags) within them — a simplified stand-in for locating survivors or objects of interest in a search-and-rescue scenario.

The system is split into one ROS2 package per subsystem so each layer (locomotion, sensing, mapping, navigation, perception) can be developed, tested, and launched independently, then composed together via the top-level bringup launch files.

## System Architecture

```
 Arduino (motor + encoders)
        │  serial protocol
        ▼
 mecanum_firmware (ros2_control SystemInterface, C++)
        │
        ▼
 ros2_control: mecanum_drive_controller ──► /mecanum_drive_controller/odometry
        │                                          │
        ▼                                          ▼
 twist_mux (joystick vs nav cmd_vel arbitration)   robot_localization (EKF)
                                                     + /imu
                                                          │
                                                          ▼
                                                  odom -> map TF (slam_toolbox)
                                                          │
                                    ┌─────────────────────┼─────────────────────┐
                                    ▼                     ▼                     ▼
                              Nav2 stack          frontier_exploration    apriltag_ros
                       (planner/controller/BT)     (autonomous mapping)   + tag_marker_node
                                                                           (locks target
                                                                            positions in map
                                                                            frame, logs YAML)
```

## Package Reference

| Package | Purpose | Key contents |
|---|---|---|
| `rescuebot_bringup` | Top-level entry points | `real_robot.launch.py`, `simulated_robot.launch.py`, RPLidar A1 config |
| `rescuebot_description` | Robot model | `rescuebot.xacro`, `rescuebot_controller.xacro`, `rescuebot_gazebo.xacro`, meshes, Gazebo worlds, RViz configs |
| `rescuebot_firmware` | Hardware interface + Arduino firmware | `mecanum_firmware` (custom `hardware_interface::SystemInterface` plugin, C++), Arduino `.ino` sketches (`simple_motor`, `simple_encoder_reader`, `simple_serial_transmitter`, `simple_serial_receiver`) |
| `rescuebot_controller` | Drive control & teleop | `khushi_controller.yaml` (`mecanum_drive_controller` config), `twist_mux_*.yaml` (joystick/nav arbitration), `drive_teleop.py`, `twist_relay.py`, `joystick_teleop.launch.py` |
| `rescuebot_mapping` | Mapping & localization | `ekf.yaml` (`robot_localization`, fuses wheel odom + IMU), `slam_toolbox.yaml`, `mapping_with_known_poses.py` |
| `rescuebot_navigation` | Autonomous navigation | Nav2 `behavior_server`, `bt_navigator`, `controller_server`, `costmap`, `planner_server`, `smoother_server` configs + custom behavior trees |
| `rescuebot_planning` | Standalone path planners | `a_star_planner.py`, `dijkstra_planner.py` (independent of the Nav2 planner_server — experimental/alternative planners over `OccupancyGrid`) |
| `rescuebot_perception` | Target detection | `camera_publisher.py` (webcam → `/camera/image_raw`), `apriltag.launch.py` (wraps `apriltag_ros`), `tag_marker_node.py` (locks + logs confirmed tag positions), `tags.yaml` |
| `frontier-exploration-ros2` | Autonomous exploration | Frontier-based exploration package (external, embedded as a git submodule) |

## Hardware

- **Drivetrain:** 4-wheel mecanum drive, driven via a custom `mecanum_firmware` `ros2_control` hardware interface plugin communicating with an Arduino over a defined serial protocol (see `mecanum_interface.cpp` — wheel order is `front_right, front_left, back_right, back_left`, matched on both the firmware and URDF sides)
- **Compute/control:** `ros2_control_node` running the `mecanum_drive_controller` (`mecanum_drive_controller/MecanumDriveController`) and `joint_state_broadcaster`
- **Lidar:** RPLidar A1 (`rplidar_ros`), used for SLAM and Nav2 costmaps
- **IMU:** fused into the EKF alongside wheel odometry (`/imu` topic, yaw + yaw-rate only — see `ekf.yaml`)
- **Camera:** USB webcam via OpenCV (`camera_publisher.py`) feeding `apriltag_ros` for target detection — camera intrinsics in the current publisher are a placeholder and need real calibration for accurate tag pose estimates
- **Input:** Xbox/generic joystick via `joy_teleop.yaml` / `joy_config.yaml`, arbitrated against autonomous `cmd_vel` through `twist_mux`

## Requirements

- Ubuntu 22.04, ROS2 Humble
- [`ros2_control`](https://control.ros.org/) / `controller_manager`
- [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros)
- [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox)
- [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization)
- [`nav2`](https://github.com/ros-planning/navigation2) (incl. `nav2_map_server`, `nav2_lifecycle_manager`)
- [`apriltag_ros`](https://github.com/christianrauch/apriltag_ros)
- `twist_mux`, `cv_bridge`, `python3-serial`, `python3-smbus`, `libserial-dev`
- Gazebo (for simulation)

## Build

```bash
mkdir -p ~/RescueBot/src   # if starting fresh
cd ~/RescueBot
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

> Note: `src/frontier-exploration-ros2` is tracked as its own git repository inside this one. If cloning fresh and it doesn't need to stay linked to upstream, either `git submodule add` it properly or vendor it in — see the package's own instructions if it's set up as a submodule.

## Usage

**Simulation (Gazebo, with SLAM, RViz, navigation, exploration, and AprilTag detection):**
```bash
ros2 launch rescuebot_bringup simulated_robot.launch.py use_slam:=true
```

**On real hardware** (hardware interface, RPLidar, controllers, joystick, SLAM, navigation):
```bash
ros2 launch rescuebot_bringup real_robot.launch.py use_slam:=true
```
> Camera publishing, AprilTag detection, and frontier exploration are currently commented out in `real_robot.launch.py` — uncomment them once the camera/calibration is ready to enable target detection on hardware.

**Mapping only:**
```bash
ros2 launch rescuebot_mapping slam.launch.py
```

**Useful launch arguments (`rescuebot_controller/controller.launch.py`):**
| Arg | Default | Purpose |
|---|---|---|
| `use_slam` | `true` | Bring up `slam_toolbox` alongside navigation |
| `use_simple_controller` | `false` | Swap the `mecanum_drive_controller` for a simplified Python/C++ velocity controller |
| `use_python` | varies | Use the Python vs C++ simple controller node (only relevant when `use_simple_controller:=true`) |
| `wheel_radius` / `wheel_separation` | `0.033` / `0.17` | Kinematic parameters for the simple controller path |

## Topics, Frames & Data Flow

- **TF tree:** `map -> odom -> base_link -> ... -> camera_link_optical -> tag_<id>` — `slam_toolbox` publishes `map -> odom`, `robot_state_publisher` publishes the rest from the URDF.
- **Odometry:** `mecanum_drive_controller` publishes `/mecanum_drive_controller/odometry`, fused with `/imu` by the EKF (`robot_localization`) into `odom -> base_link`.
- **Perception:** `camera_publisher` → `/camera/image_raw` + `/camera/camera_info` → `apriltag_ros` → per-tag TF frames (`tag_0`, `tag_1`, ...) → `tag_marker_node` → `/detected_tags_markers` (RViz) + logged YAML.

## Target Detection & Logging

`tag_marker_node` watches TF for AprilTag frames (`apriltag_ros` output), transforms each detection into the map frame, and requires several consistent readings (default: 3, within 0.15 m of each other) before "confirming" and permanently locking a tag's position — protecting against a single noisy detection mis-placing a marker. Confirmed tag positions are visualized as markers in RViz and logged to a fresh YAML file per run under `~/rescuebot_runs/`.

## Known Issues / Work in Progress

- Camera intrinsics in `camera_publisher.py` are a rough placeholder (not a real calibration) — AprilTag pose estimates will be inaccurate until the camera is properly calibrated.
- Camera publishing, AprilTag detection, and frontier exploration are disabled by default in `real_robot.launch.py` pending hardware camera setup.
- `dijkstra_planner.py` / `a_star_planner.py` are standalone planners separate from the Nav2 `planner_server` — not yet wired into the main navigation pipeline.
- `simple_controller` / `diff_drive_controller` paths exist in config as alternates to the primary `mecanum_drive_controller` but aren't the default flow.

## License

_Add a license file to specify how this project can be used/modified._
