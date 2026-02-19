# Drone Autonomy Platform

[![CI](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Orin-green)](https://developer.nvidia.com/embedded/jetson-orin)

> AI-powered drone autonomy platform optimized for NVIDIA Jetson Orin — industrial-grade autonomous quadcopter designed for agricultural mapping, ISR, and real-time perception.

This system leverages the **Pixhawk 6X** for flight stability and the **NVIDIA Jetson Orin** for on-board Spatial AI and computer vision.

---

## Hardware Configuration

### Flight Control

| Component | Specification |
|-----------|---------------|
| Frame | Tarot 650 Sport (Carbon Fiber) |
| Autopilot | Holybro Pixhawk 6X (v2A) |
| Firmware | ArduPilot (ArduCopter 4.5+) |
| RC Link | RadioMaster TX16S MKII (4-in-1) + R88 Receiver |
| Telemetry | SiK Telemetry Radio V3 (433MHz) |

### Companion Computer

| Component | Specification |
|-----------|---------------|
| Compute | NVIDIA Jetson Orin Nano (JetPack 6.x) |
| Primary Storage | 1TB Lexar NM790 NVMe SSD |
| Boot Storage | 128GB SanDisk Extreme A2 microSD |
| Vision | Luxonis OAK-D (Spatial AI / Stereo Depth) |
| Data Bridge | UAV-DEV USB2SERIAL (Telem 2 ↔ USB 3.1) |

### Power System (6S)

| Component | Specification |
|-----------|---------------|
| Battery | SLS XTRON 10,000mAh 6S1P (22.2V) |
| Power Splitter | XT90-S Anti-Spark Parallel Y-Cable |
| Regulation | Matek UBEC Duo (12V/4A → Jetson, 5V/4A → Peripherals) |
| Charging | ISDT K4 Smart Duo + XT60-to-XT90 Adapters |

---

## Software Stack

The system runs a containerized microservice architecture using **Docker** to ensure GPU-accelerated consistency across JetPack versions.

| Layer | Technology |
|-------|------------|
| Middleware | ROS2 Humble |
| Flight Bridge | MAVROS (ArduPilot ↔ ROS2) |
| Vision | DepthAI (OAK-D VPU offload) |
| Perception | NVIDIA Isaac ROS (TensorRT accelerated) |

---

## System Architecture

### High-Level Overview

```
                              ┌─────────────────────────────────────┐
                              │          GROUND STATION             │
                              │  ┌─────────┐       ┌─────────────┐  │
                              │  │   GCS   │       │  Telemetry  │  │
                              │  │ Display │◄──────│   Receiver  │  │
                              │  └─────────┘       └──────▲──────┘  │
                              └───────────────────────────┼─────────┘
                                                          │ MAVLink (433MHz)
                              ┌───────────────────────────┼─────────┐
                              │                    DRONE  │         │
  ┌───────────────────────────┼───────────────────────────┼───────┐ │
  │ COMPANION COMPUTER        │                           │       │ │
  │ (Jetson Orin)             │                           │       │ │
  │ ┌─────────────────────────┼───────────────────────────┼─────┐ │ │
  │ │                    ROS2 │Humble                     │     │ │ │
  │ │  ┌─────────┐  ┌─────────┴───┐  ┌───────────┐  ┌─────┴───┐ │ │ │
  │ │  │Perception│  │ Navigation │  │  Control  │  │ Comms   │ │ │ │
  │ │  │ (Isaac) │  │(SLAM/Plan) │  │ (Traj)    │  │(MAVLink)│ │ │ │
  │ │  └────┬────┘  └─────┬──────┘  └─────┬─────┘  └────┬────┘ │ │ │
  │ │       └─────────────┴───────────────┴─────────────┘      │ │ │
  │ │                         │                                │ │ │
  │ │  ┌──────────────────────┴──────────────────────────────┐ │ │ │
  │ │  │               AUTONOMY (Mission/BT)                 │ │ │ │
  │ │  └──────────────────────┬──────────────────────────────┘ │ │ │
  │ │                         │                                │ │ │
  │ │  ┌──────────────────────┴──────────────────────────────┐ │ │ │
  │ │  │                SAFETY (Failsafe/Geo)                │ │ │ │
  │ │  └─────────────────────────────────────────────────────┘ │ │ │
  │ └──────────────────────────────────────────────────────────┘ │ │
  │                            │ UART/USB                        │ │
  └────────────────────────────┼─────────────────────────────────┘ │
                               │                                   │
  ┌────────────────────────────┼─────────────────────────────────┐ │
  │ FLIGHT CONTROLLER          │                                 │ │
  │ (Pixhawk 6X)               ▼                                 │ │
  │  ┌─────────────────────────────────────────────────────────┐ │ │
  │  │              ArduPilot (ArduCopter 4.5+)                │ │ │
  │  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────┐ │ │ │
  │  │  │  EKF3   │  │  Mixer  │  │   PID   │  │ Failsafes   │ │ │ │
  │  │  └─────────┘  └─────────┘  └─────────┘  └─────────────┘ │ │ │
  │  └─────────────────────────────────────────────────────────┘ │ │
  │                            │ PWM                             │ │
  └────────────────────────────┼─────────────────────────────────┘ │
                               ▼                                   │
                    ┌─────────────────────┐                        │
                    │   ESCs + Motors     │                        │
                    └─────────────────────┘                        │
                              └─────────────────────────────────────┘
```

### Compute Architecture

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

### ROS2 Node Communication

```
                           ┌─────────────────────────────────────────┐
   ┌───────────┐           │              /autonomy_node             │
   │  OAK-D    │           │  ┌─────────────────────────────────┐   │
   │  Camera   │           │  │      Mission State Machine      │   │
   └─────┬─────┘           │  │      Behavior Tree Engine       │   │
         │                 │  └─────────────────────────────────┘   │
         ▼                 └──────────────┬──────────────────────────┘
┌─────────────────┐                       │
│/perception_node │        /mission ──────┘
│ ┌─────────────┐ │        /mission_status
│ │  RT-DETR    │ │                       │
│ │  Detector   │ │                       ▼
│ ├─────────────┤ │        ┌─────────────────────────────────────────┐
│ │  Tracker    │ │        │            /navigation_node             │
│ └─────────────┘ │        │  ┌─────────────────────────────────┐   │
└────────┬────────┘        │  │    Visual SLAM  │  Path Planner │   │
         │                 │  └─────────────────────────────────┘   │
         │ /detections     └──────────────┬──────────────────────────┘
         │ /depth_image                   │
         │ /camera/image_raw              │ /trajectory
         │                                │ /pose
         ▼                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                          ROS2 DDS (Cyclone DDS)                     │
└─────────────────────────────────────────────────────────────────────┘
         │                                │
         │ /sensor_data                   │ /attitude_cmd
         │                                │ /velocity_cmd
         ▼                                ▼
┌─────────────────┐        ┌─────────────────────────────────────────┐
│  /safety_node   │        │             /control_node               │
│ ┌─────────────┐ │        │  ┌─────────────────────────────────┐   │
│ │ Geofence    │ │        │  │  Trajectory Tracker  │   MPC    │   │
│ │ Failsafe    │ │        │  └─────────────────────────────────┘   │
│ │ Watchdog    │ │        └──────────────┬──────────────────────────┘
│ └─────────────┘ │                       │
└────────┬────────┘                       │ /mavros/setpoint_*
         │                                ▼
         │ /safety_status  ┌─────────────────────────────────────────┐
         └────────────────▶│          /communication_node            │
                           │  ┌─────────────────────────────────┐   │
                           │  │   MAVROS   │   Telemetry Tx     │   │
                           │  └─────────────────────────────────┘   │
                           └──────────────┬──────────────────────────┘
                                          │ MAVLink
                                          ▼
                                   ┌─────────────┐
                                   │  Pixhawk 6X │
                                   └─────────────┘
```

### Data Flow & Latency Budget

End-to-end latency target: **≤250ms** (sensor to action)

```
┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐
│  Camera   │───▶│   Image   │───▶│ Perception│───▶│ Decision  │───▶│  Control  │
│  Capture  │    │Processing │    │ (RT-DETR) │    │  Making   │    │  Command  │
└───────────┘    └───────────┘    └───────────┘    └───────────┘    └───────────┘
     30ms            50ms             100ms            20ms             10ms
              ◀───────────────────────────────────────────────────────────────▶
                                    Total: 210ms
```

| Stage | Budget | Description |
|-------|--------|-------------|
| Camera | 30ms | Capture → MIPI-CSI bus |
| Image Processing | 50ms | Debayer, color correction |
| Perception | 100ms | RT-DETR inference (TensorRT) |
| Decision | 20ms | Mission manager response |
| Control | 10ms | Command generation |
| Flight Controller | 10ms | PWM output |

---


## Quick Start

### Prerequisites

- Docker with NVIDIA Container Toolkit
- ROS2 Humble (for native builds)
- NVIDIA Jetson Orin (deployment) or x86 with NVIDIA GPU (development)

### Development Container (Recommended)

```bash
# Clone the repository
git clone https://github.com/Darainer/drone_autonomy_platform.git
cd drone_autonomy_platform

# Start development container with GPU support
docker compose -f docker/docker-compose.yml up -d dev
docker compose exec dev bash

# Build inside container
colcon build --symlink-install
source install/setup.bash

# Launch the platform
ros2 launch launch/platform.launch.py
```

### Native Build

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Launch individual nodes
ros2 launch src/perception/launch/perception.launch.py
```

---

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

---

## AI Agent Workforce

| Agent | Purpose | Trigger |
|-------|---------|---------|
| Issue Triage | Categorize & route issues | New issue opened |
| Safety Review | Analyze safety-critical code | PR to `control/` or `safety/` |
| Test Generation | Generate test cases | Feature ready |

---

## Safety-Critical Development

This platform follows **DO-178C** principles for safety-critical avionics software:

- Mandatory safety review for `control/` and `safety/` code changes
- Simulation-first testing before hardware deployment
- Comprehensive failsafe mechanisms (geofencing, return-to-home, motor cutoff)
- Traceability from requirements to implementation

---

## License

Apache 2.0 — see [LICENSE](LICENSE)