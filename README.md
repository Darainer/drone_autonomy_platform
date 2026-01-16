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

## Quick Start

```bash
# Clone and enter
git clone https://github.com/Darainer/drone_autonomy_platform.git
cd drone_autonomy_platform

# Start dev container
docker compose -f docker/docker-compose.yml up -d dev
docker compose exec dev bash

# Build
colcon build --symlink-install
source install/setup.bash
```

## Project Structure

```
drone_autonomy_platform/
├── src/
│   ├── perception/      # Camera, LiDAR, sensor fusion (ISAAC ROS)
│   ├── navigation/      # Path planning, mapping, localization
│   ├── control/         # Flight control, trajectory tracking
│   ├── autonomy/        # Mission management, behavior trees
│   ├── communication/   # MAVLink, telemetry, GCS interface
│   ├── safety/          # Failsafes, geofencing, emergency
│   └── common/          # Shared utilities
├── config/              # Vehicle, sensor, mission configs
├── launch/              # ROS2 launch files
├── docker/              # Development containers
├── test/                # Unit, integration, simulation tests
├── tools/               # Analysis, calibration, deployment
└── docs/                # Documentation
```

## AI Agent Workforce

| Agent         | Purpose                                         | Trigger          |
|---------------|-------------------------------------------------|------------------|
| Issue Triage  | Categorize & route issues                      | New issue        |
| Safety Review | Analyze safety-critical code                    | PR to control/safety |
| Test Generation| Generate test cases                             | Feature ready     |

## Safety-Critical Development

- DO-178C principles
- Mandatory safety review for control/safety code
- Simulation-first testing
- Comprehensive failsafe mechanisms

## License

Apache 2.0 - see [LICENSE](LICENSE)