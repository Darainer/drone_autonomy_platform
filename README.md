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
| Flight Bridge | MAVROS (PX4 ↔ ROS2) |
| Vision | DepthAI (OAK-D VPU offload) |
| Perception | NVIDIA Isaac ROS (TensorRT accelerated) |
| Telemetry | MAVLink (433MHz SiK Radio) |
| Video Downlink | GStreamer + WiFi / WFB-ng ([details](docs/architecture/telemetry.md)) |

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
  │ │  ┌──────────┐  ┌─────────┴──┐  ┌───────────┐  ┌─────┴───┐ │ │ │
  │ │  │Perception│  │ Navigation │  │  Control  │  │ Comms   │ │ │ │
  │ │  │ (Isaac)  │  │(SLAM/Plan) │  │ (Traj)    │  │(MAVLink)│ │ │ │
  │ │  └────┬─────┘  └────┬───────┘  └────┬──────┘  └────┬────┘ │ │ │
  │ │       └─────────────┴───────────────┴──────────────┘      │ │ │
  │ │                         │                                 │ │ │
  │ │  ┌──────────────────────┴──────────────────────────────┐  │ │ │
  │ │  │               AUTONOMY (Mission/BT)                 │  │ │ │
  │ │  └──────────────────────┬──────────────────────────────┘  │ │ │
  │ │                         │                                 │ │ │
  │ │  ┌──────────────────────┴──────────────────────────────┐  │ │ │
  │ │  │                SAFETY (Failsafe/Geo)                │  │ │ │
  │ │  └─────────────────────────────────────────────────────┘  │ │ │
  │ └────────────────────────────────────────────────────────── ┘ │ │
  │                            │ UART/USB                         │ │
  └────────────────────────────┼─────────────────────────────── ──┘ │
                               │                                    │
  ┌────────────────────────────┼─────────────────────────────── ──┐ │
  │ FLIGHT CONTROLLER          │                                  │ │
  │ (Pixhawk 6X)               ▼                                  │ │
  │  ┌───────────────────────────────────────────────────────── ┐ │ │
  │  │              ArduPilot (ArduCopter 4.5+)                 │ │ │
  │  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────┐  │ │ │
  │  │  │  EKF3   │  │  Mixer  │  │   PID   │  │ Failsafes   │  │ │ │
  │  │  └─────────┘  └─────────┘  └─────────┘  └─────────────┘  │ │ │
  │  └───────────────────────────────────────────────────────── ┘ │ │
  │                            │ PWM                              │ │
  └────────────────────────────┼─────────────────────────────── ──┘ │
                               ▼                                    │
                    ┌─────────────────────┐                         │
                    │   ESCs + Motors     │                         │
                    └─────────────────────┘                         │
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
         │ /oak/stereo/image_raw          │
         │ /oak/rgb/image_raw             │ /trajectory
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


## Requirements

- Docker (tested on 20.10+)
- Git

No ROS2 local installation required — everything runs inside the container.

## Quick Start

### Prerequisites

- Docker with NVIDIA Container Toolkit
- ROS2 Humble (for native builds)
- NVIDIA Jetson Orin (deployment) or x86 with NVIDIA GPU (development)

### Development Container (Recommended)

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
  --packages-ignore common \
  --cmake-args -DBUILD_TESTING=OFF
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

The `perception_node` binary builds on x86 and is included in the standard Docker build and CI.
The full RT-DETR inference pipeline and OAK-D camera driver are runtime-only dependencies
(`exec_depend` in `package.xml`) that require NVIDIA Isaac ROS packages
(`isaac_ros_dnn_image_encoder`, `isaac_ros_rtdetr`, `isaac_ros_tensor_rt`) and `depthai_ros_driver`,
available only on Jetson via NVIDIA's apt registry. Use `platform.launch.py` on the Orin to
launch the complete pipeline; `platform_core.launch.py` excludes perception for environments
without these runtime dependencies.

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
