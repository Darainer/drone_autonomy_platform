# Drone Autonomy Platform

[![CI](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Orin-green)](https://developer.nvidia.com/embedded/jetson-orin)

> AI-powered drone autonomy platform optimized for NVIDIA Jetson Orin

## Architecture

This platform is a modular ROS2-based system for controlling autonomous drones. It is designed to be extensible and to support a variety of missions. The architecture is composed of the following modules:

- **Communication:** Handles communication with the flight controller (e.g., PX4) via MAVROS. It translates high-level commands from the autonomy module into MAVLink messages and sends them to the flight controller. It also receives telemetry data from the flight controller and publishes it as ROS2 messages.
- **Perception:** Processes sensor data from cameras, LiDAR, and other sensors. It uses NVIDIA Isaac ROS for GPU-accelerated perception, including visual SLAM and DNN-based object detection.
- **Navigation:** Responsible for localization, mapping, and path planning. It uses the Nav2 stack for high-level navigation and path planning.
- **Control:** Computes the control commands (e.g., attitude, velocity) required to follow the desired trajectory.
- **Autonomy:** Contains the high-level mission logic. It uses a behavior tree to define the drone's behavior in different situations.
- **Safety:** Monitors the drone's state and takes action to prevent accidents. It includes a failsafe system that can take over control of the drone in case of a software failure.
- **Common:** Contains common data structures, utilities, and configuration files used by other modules.

## Getting Started

To get started with this platform, you will need to have ROS2 Humble and the NVIDIA Jetson Orin SDK installed. You will also need to have a flight controller (e.g., PX4) and a drone.

To build the platform, clone this repository and then run the following commands:

```bash
colcon build
```

## Usage

To launch the platform, run the following command:

```bash
ros2 launch drone_autonomy_platform platform.launch.py
```

## Contributing

Contributions are welcome! Please see the [contributing guidelines](CONTRIBUTING.md) for more information.
