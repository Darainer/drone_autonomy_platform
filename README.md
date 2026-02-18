# Drone Autonomy Platform

[![CI](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Orin-green)](https://developer.nvidia.com/embedded/jetson-orin)

> AI-powered drone autonomy platform optimized for NVIDIA Jetson Orin

# 🛸 Agri-Map Autonomous UAV

An industrial-grade autonomous quadcopter designed for agricultural mapping and real-time crop analysis. This system leverages the **Pixhawk 6X** for flight stability and the **NVIDIA Jetson Orin Nano** for on-board Spatial AI and computer vision.

---

## 🏗 System Architecture

### 1. Flight Control (The Pilot)
* **Frame:** Tarot 650 Sport (Carbon Fiber)
* **Autopilot:** Holybro Pixhawk 6X (v2A)
* **Firmware:** ArduPilot (ArduCopter 4.5+)
* **RC Link:** RadioMaster TX16S MKII (4-in-1) with R88 Receiver
* **Telemetry:** SiK Telemetry Radio V3 (433MHz) for GCS/MAVLink

### 2. Companion Computer (The Brain)
* **Compute:** NVIDIA Jetson Orin Nano Developer Kit (JetPack 6.x)
* **Primary Storage:** 1TB Lexar NM790 NVMe SSD (RootFS migrated)
* **Boot Storage:** 128GB SanDisk Extreme A2 microSD
* **Vision Hardware:** Luxonis OAK-D (Spatial AI / Stereo Depth)
* **Data Bridge:** UAV-DEV USB2SERIAL (Telem 2 ↔ USB 3.1)

### 3. Power System (6S)
* **Battery:** SLS XTRON 10,000mAh 6S1P (22.2V)
* **Power Splitter:** XT90-S Anti-Spark Parallel Y-Cable
* **Regulation:** Matek UBEC Duo (12V/4A to Jetson | 5V/4A to Peripherals)
* **Charging:** ISDT K4 Smart Duo + XT60-to-XT90 Adapters

---

## 💻 Software Environment

### Containerization
The system runs a containerized microservice architecture using **Docker** to ensure GPU-accelerated consistency across JetPack versions.

* **ROS2 Humble:** Core robotics middleware.
* **MAVROS:** Communication bridge between ArduPilot and ROS2.
* **DepthAI:** Luxonis library for OAK-D VPU offloading.
* **NVIDIA Isaac ROS:** Hardware-accelerated Visual SLAM and perception.


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