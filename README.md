# RescueBot 🤖

Autonomous 4-wheeled search-and-rescue robot built on **ROS2 Humble**, with a custom `ros2_control` hardware interface talking to Arduino firmware over a defined serial protocol. The stack combines mecanum-drive locomotion, EKF sensor fusion, SLAM-based mapping, Nav2 autonomous navigation, frontier exploration, and AprilTag-based perception.

## Overview

RescueBot is designed to autonomously map and navigate unknown environments, with the long-term goal of supporting search-and-rescue tasks such as locating tagged targets in unmapped spaces. Locomotion, sensing, mapping, navigation, and perception are each split into their own ROS2 packages so pieces can be developed, tested, and launched independently.

## Package Layout

| Package | Description |
|---|---|
| `rescuebot_bringup` | Top-level launch files that bring the full system (or subsystems) online |
| `rescuebot_description` | URDF/xacro robot description, meshes, and RViz configs |
| `rescuebot_controller` | `ros2_control` hardware interface and controller configuration for the drivetrain |
| `rescuebot_firmware` | Arduino firmware and serial protocol for low-level motor/sensor communication |
| `rescuebot_mapping` | SLAM (`slam_toolbox`) and EKF sensor fusion (`robot_localization`) configuration |
| `rescuebot_navigation` | Nav2 configuration for autonomous path planning and navigation |
| `rescuebot_planning` | Higher-level task/behavior planning |
| `rescuebot_perception` | AprilTag detection and other perception nodes |
| `frontier-exploration-ros2` | Frontier-based autonomous exploration for unmapped environments |

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox)
- [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization)
- [`nav2`](https://github.com/ros-planning/navigation2)
- Gazebo (for simulation)

## Build

```bash
mkdir -p ~/RescueBot/src   # if starting fresh
cd ~/RescueBot
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Usage

**Simulation (Gazebo, with SLAM, navigation, and exploration):**
```bash
ros2 launch rescuebot_bringup bringup.launch.py use_slam:=true
```

**Mapping only:**
```bash
ros2 launch rescuebot_mapping slam.launch.py
```

**On real hardware:** bring up the `ros2_control` hardware interface and controllers via `rescuebot_controller`, then launch navigation/perception as needed — see each package's `launch/` directory for the individual launch files and arguments.

## Status

Actively in development for Robocon competition and general autonomous navigation work. Package-level READMEs (where present) have more detail on individual subsystems.

## License

_Add a license file to specify how this project can be used/modified._
