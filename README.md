# Drone Autonomy Platform

[![CI](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Orin-green)](https://developer.nvidia.com/embedded/jetson-orin)

> AI-powered drone autonomy platform optimized for NVIDIA Jetson Orin

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    NVIDIA JETSON ORIN AGX/NX                    │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │              CUDA / TensorRT / cuDNN / ISAAC ROS          │  │
│  └───────────────────────────────────────────────────────────┘  │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────┐  │
│  │  GPU Cores  │ │ DLA Engines │ │  Vision Acc │ │  Tensor   │  │
│  │  (Ampere)   │ │   (2x DLA)  │ │    (PVA)    │ │   Cores   │  │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └─────┬─────┘  │
│         └───────────────┴───────────────┴───────────────┘       │
└─────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│  PERCEPTION   │   │  NAVIGATION   │   │   CONTROL     │
│ Camera/LiDAR  │   │ Planning/SLAM │   │  PID/MPC      │
└───────┬───────┘   └───────┬───────┘   └───────┬───────┘
        └───────────────────┼───────────────────┘
                            ▼
                   ┌───────────────┐
                   │   AUTONOMY    │
                   │ Mission/BT/SM │
                   └───────┬───────┘
                           │
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐
│    SAFETY     │ │ COMMUNICATION │ │  PX4/Ardupilot│
│ Failsafe/Geo  │ │ MAVLink/Telem │ │  Integration  │
└───────────────┘ └───────────────┘ └───────────────┘
```

## Supported Hardware

| Variant            | GPU       | CPU        | Memory | Use Case               |
|--------------------|-----------|------------|--------|------------------------|
| Orin AGX 64GB      | 2048 CUDA | 12-core Arm| 64GB   | Full autonomy stack     |
| Orin AGX 32GB      | 1792 CUDA | 8-core Arm | 32GB   | Standard deployment     |
| Orin NX 16GB       | 1024 CUDA | 8-core Arm | 16GB   | Lightweight missions    |
| Orin Nano 8GB      | 512 CUDA  | 6-core Arm | 8GB    | Basic perception        |

## Requirements

- Docker (tested on 20.10+)
- Git

No ROS2 local installation required — everything runs inside the container.

## Quick Start

```bash
git clone https://github.com/Darainer/drone_autonomy_platform.git
cd drone_autonomy_platform

# Build the image (first build ~5 min, cached rebuilds ~30s)
docker build -t drone_autonomy_platform .
```

## Launch

### Core platform (all nodes except perception)

```bash
docker run -it --rm drone_autonomy_platform \
  ros2 launch /ws/src/drone_autonomy_platform/launch/platform_core.launch.py
```

### Full platform (requires NVIDIA Isaac ROS — Jetson only)

```bash
docker run -it --rm drone_autonomy_platform \
  ros2 launch /ws/src/drone_autonomy_platform/launch/platform.launch.py
```

### Interactive shell

```bash
docker run -it --rm drone_autonomy_platform bash
# Inside the container, ROS2 and the workspace are already sourced:
ros2 node list
ros2 topic list
```

### Smoke test

Verifies all core nodes start and register with the ROS2 graph:

```bash
docker run --rm drone_autonomy_platform \
  bash /ws/src/drone_autonomy_platform/scripts/smoke_test.sh
```

## Development

Mount the source into a running container to iterate without rebuilding the image:

```bash
docker run -it --rm \
  -v $(pwd):/ws/src/drone_autonomy_platform \
  drone_autonomy_platform bash

# Inside the container, rebuild after changes:
cd /ws
colcon build --merge-install \
  --base-paths src/drone_autonomy_platform/msgs src/drone_autonomy_platform/src \
  --packages-ignore common perception \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## Project Structure

```
drone_autonomy_platform/
├── Dockerfile               # Build environment (ros:humble base)
├── msgs/                    # Custom ROS2 message definitions
├── src/
│   ├── autonomy/            # Mission management, behavior trees
│   ├── communication/       # MAVLink, telemetry, GCS interface
│   ├── control/             # Flight control, trajectory tracking
│   ├── navigation/          # Path planning, mapping, localization
│   ├── safety/              # Failsafes, geofencing, emergency
│   ├── perception/          # Camera/LiDAR, sensor fusion (Isaac ROS — Jetson only)
│   └── common/              # Shared headers
├── launch/
│   ├── platform_core.launch.py   # Core nodes (CI + non-Jetson)
│   └── platform.launch.py        # Full stack including perception
├── scripts/
│   └── smoke_test.sh        # Node startup smoke test
└── docs/                    # Architecture and standards documentation
```

## Notes on `perception`

The `perception` package depends on [NVIDIA Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS) packages
(`isaac_ros_visual_slam_interfaces`, `isaac_ros_dnn_inference`) which are only available via
NVIDIA's Jetson apt registry. It is excluded from the standard Docker build and CI. To build it,
add the Isaac ROS apt sources and remove `perception` from `--packages-ignore` in the Dockerfile.

## AI Agent Workforce

| Agent          | Purpose                              | Trigger              |
|----------------|--------------------------------------|----------------------|
| Issue Triage   | Categorize & route issues            | New issue            |
| Safety Review  | Analyze safety-critical code         | PR to control/safety |
| Test Generation| Generate test cases                  | Feature ready        |

## Safety-Critical Development

- DO-178C principles applied to safety and control code
- Mandatory safety review for `src/control/` and `src/safety/`
- Simulation-first testing policy
- Comprehensive failsafe mechanisms

## License

Apache 2.0 — see [LICENSE](LICENSE)
