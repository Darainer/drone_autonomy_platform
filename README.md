# Drone Autonomy Platform

[![CI](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Orin-green)](https://developer.nvidia.com/embedded/jetson-orin)

Autonomous drone platform built on ROS2 Humble and NVIDIA Isaac ROS. Runs real-time RF-DETR object detection and visual SLAM on a Jetson Orin Nano with a Luxonis OAK-D camera, communicating with a Pixhawk 6X flight controller over MAVLink.

---

## Physical Architecture

```
  GROUND STATION                          RC CONTROLLER
┌───────────────────────────┐       ┌───────────────────────┐
│  Laptop (QGroundControl)  │       │   RadioMaster Boxer   │
│  ┌─────────┐  ┌─────────┐│        │       (ELRS TX)       │
│  │ Mission │  │  Video  ││        └───────────┬───────────┘
│  │ Planner │  │ Display ││                    │
│  └─────────┘  └─────────┘│                    │ ELRS 2.4 GHz
└──────┬─────────────┬──────┘                   │
       │             │                          │
  MAVLink 433 MHz    │ WiFi / WFB-ng            │
       │             │ (video downlink)         │
═══════╪═════════════╪══════════════════════════╪════  AIR
       │             │                          │
┌──────┴─────────────┴──────────────────────────┴──────────┐
│  DRONE                                                   │
│                                                          │
│  ┌─────────────────────────────────────────────────────┐ │
│  │  Jetson Orin Nano                                   │ │
│  │  ┌────────────────────┐                             │ │
│  │  │    ROS2 Humble     │                             │ │
│  │  │    (6 packages)    │                             │ │
│  │  └─────────┬──────────┘                             │ │
│  │       USB 3.1│                                      │ │
│  │  ┌─────────┴──────────┐                             │ │
│  │  │   OAK-D Camera     │                             │ │
│  │  │   RGB + Stereo     │                             │ │
│  │  │   + IMU            │                             │ │
│  │  └────────────────────┘                             │ │
│  └──────────┬──────────────────────────────────────────┘ │
│         UART│(Telem 2)                                   │
│  ┌──────────┴──────────────────────────────────────────┐ │
│  │  Pixhawk 6X (PX4)                                   │ │
│  │    ├── Telem 1 ──▶ SiK 433 MHz antenna              │ │
│  │    ├── Telem 2 ──▶ Jetson Orin companion link       │ │
│  │    ├── Telem 3 ◀── R88 ELRS receiver (CRSF)         │ │
│  │    ├── GPS     ◀── GPS module + antenna             │ │
│  │    └── PWM Out ──▶ ESCs ──▶ Motors (x4)             │ │
│  └─────────────────────────────────────────────────────┘ │
│                                                          │
│  WiFi antenna (Jetson) ──▶ video downlink to GCS         │
└──────────────────────────────────────────────────────────┘
```

---

## Software Architecture

### ROS2 Packages

```
src/
├── perception/      OAK-D → RF-DETR detection + cuVSLAM odometry (Jetson only)
├── navigation/      Path planning, SLAM, localization (Nav2)
├── control/         Trajectory tracking, MAVROS bridge to Pixhawk
├── autonomy/        Mission logic, behavior trees
├── communication/   MAVLink telemetry, GCS interface
├── safety/          Geofence, failsafe, battery monitor, watchdog
└── common/          Shared headers
```

### Perception Pipeline (Jetson Orin)

```
OAK-D Camera                     Jetson Orin
┌──────────┐      /oak/rgb/image_raw     ┌──────────────┐      /detections
│ depthai  │────────────────────────────▶│ RF-DETR-S    │────────────────────┐
│ _ros_    │                             │ TensorRT     │                    │
│ driver   │                             │ inference    │                    ▼
└────┬─────┘                             └──────────────┘          ┌──────────────────┐
     │                                                            │  perception_node │
     │ /oak/left/image_raw                                        │  (sensor fusion) │
     │ /oak/right/image_raw                                       └──────────────────┘
     │ /oak/imu/data
     │
     └────────────────────────────────────────────────────────────▶┌──────────────┐
                                                (stereo + IMU)    │   cuVSLAM    │
                                                                  │ Visual SLAM  │
                                                                  └──────────────┘
```

### Node Topics

```
                    ┌──────────────────┐
  OAK-D ──────────▶│ perception_node  │──▶ ~/sensor_data
  /detections ────▶│                  │
                    └──────────────────┘

                    ┌──────────────────┐
  ~/mission ──────▶│ navigation_node  │──▶ ~/trajectory
                    └──────────────────┘

                    ┌──────────────────┐
  ~/trajectory ──▶│  control_node    │──▶ /mavros/setpoint_*
                    └──────────────────┘

                    ┌──────────────────┐
  /mavros/* ──────▶│ communication    │──▶ MAVLink → GCS
  /safety_status ─▶│     _node        │
                    └──────────────────┘

                    ┌──────────────────┐
  /mavros/state ──▶│  safety_node     │──▶ /safety_status
                    └──────────────────┘

                    ┌──────────────────┐
  /mavros/battery ▶│ battery_monitor  │──▶ RTL via /mavros/set_mode
                    └──────────────────┘

                    ┌──────────────────┐
                    │ autonomy_node   │──▶ ~/mission
                    │ (BehaviorTree)  │    ~/mission_status
                    └──────────────────┘
```

---

## Quick Start

### x86 Development (Docker)

```bash
git clone https://github.com/Darainer/drone_autonomy_platform.git
cd drone_autonomy_platform

# Build — includes all packages except perception runtime deps
docker build -t drone_autonomy_platform .

# Launch core stack (no Jetson required)
docker run -it --rm drone_autonomy_platform \
  ros2 launch /ws/src/drone_autonomy_platform/launch/platform_core.launch.py
```

### Jetson Orin (Full Stack)

```bash
# Build with Isaac ROS dev container
docker build -f docker/Dockerfile.dev -t drone_dev .

# Generate TensorRT engine (once per device)
# See docs/architecture/perception_architecture.md for details

# Launch full platform with perception
docker run -it --rm --privileged --network host \
  --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all \
  --device /dev/ttyUSB0 \
  drone_dev \
  ros2 launch /ws/src/drone_autonomy_platform/launch/platform.launch.py
```

### Perception Launch Modes

The `perception` package exposes three Jetson launch modes:

```bash
# 1. Perception only: OAK-D RGB + RF-DETR + perception_node
ros2 launch perception perception_only.launch.py

# 2. VSLAM only: OAK-D stereo/IMU + cuVSLAM
ros2 launch perception vslam_only.launch.py

# 3. Full stack: RF-DETR + cuVSLAM + perception_node
ros2 launch perception full_stack.launch.py
```

For live annotated output during perception-only runs:

```bash
ros2 launch perception detection_viz.launch.py
```

### Development Loop

```bash
docker run -it --rm \
  -v $(pwd):/ws/src/drone_autonomy_platform \
  drone_autonomy_platform bash

# Inside the container:
cd /ws && colcon build --merge-install \
  --base-paths src/drone_autonomy_platform/msgs src/drone_autonomy_platform/src \
  --packages-ignore common \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch launch/platform_core.launch.py
```

---

## Project Structure

```
drone_autonomy_platform/
├── Dockerfile                 # x86 build (ros:humble, all packages except perception runtime)
├── docker/
│   ├── Dockerfile.dev         # Jetson Orin build (Isaac ROS + OAK-D + MAVROS)
│   ├── docker-compose.yml     # Agent workforce (Temporal + workers)
│   └── local-agent/           # Local Ollama LLM for agent dev
├── src/                       # ROS2 packages (see above)
├── msgs/                      # Custom message definitions (SensorData, Mission, etc.)
├── launch/
│   ├── platform.launch.py     # Full platform (Jetson — includes perception/full_stack.launch.py)
│   └── platform_core.launch.py # Core stack (x86/CI — excludes perception)
├── agents/                    # AI agent workforce (Temporal-based)
├── scripts/                   # submit_task.py, utilities
└── docs/                      # Architecture docs, use cases, standards
```

## Notes on Perception

The `perception_node` binary builds on x86 — it only depends on `rclcpp`, `sensor_msgs`,
`vision_msgs`, and `drone_autonomy_msgs`. The full Jetson perception stack (RF-DETR TensorRT,
cuVSLAM, depthai_ros_driver) runs only on Jetson Orin. These are declared as `exec_depend`
in `package.xml` so the package compiles anywhere but launches fully only on Jetson.

See [`docs/architecture/perception_architecture.md`](docs/architecture/perception_architecture.md)
for model selection, TensorRT engine generation, and latency analysis.

## Hardware Reference

Full frame specs, wiring, power system, and antenna placement are in
[`docs/architecture/drone_hardware.md`](docs/architecture/drone_hardware.md).
Telemetry and video downlink details are in
[`docs/architecture/telemetry.md`](docs/architecture/telemetry.md).
PX4 flight-controller port assignments and companion-link parameters are in
[`docs/architecture/px4_setup.md`](docs/architecture/px4_setup.md).

| Component | Spec |
|-----------|------|
| Frame | Tarot 650 Sport |
| Autopilot | Pixhawk 6X — PX4 |
| Companion | Jetson Orin Nano (JetPack 6.x) |
| Camera | Luxonis OAK-D (USB 3.1) |
| RC | RadioMaster Boxer + R88 (ELRS) |
| Telemetry | SiK V3 433 MHz |
| Battery | 6S 10,000 mAh LiPo |

## License

Apache 2.0 — see [LICENSE](LICENSE)
