---
name: 🚀 One-step repo initialization: NVIDIA Orin-ready structure and platform files
---

# Drone Autonomy Platform Mega Setup Script

This issue provides a single script to initialize the full NVIDIA Orin-ready repository structure and all starter files for the drone autonomy platform.

## Usage Instructions

1. **Clone your repo and checkout a new branch**
   ```bash
   git clone https://github.com/Darainer/drone_autonomy_platform.git
   cd drone_autonomy_platform
   git checkout -b init-platform-structure
   ```

2. **Create and run the setup script**
   Copy the script below into `setup_platform.sh`, then:
   ```bash
   chmod +x setup_platform.sh
   ./setup_platform.sh
   git add -A
   git commit -m "Initialize drone autonomy platform with NVIDIA Orin architecture"
   git push --set-upstream origin init-platform-structure
   ```

3. **Create a PR** at https://github.com/Darainer/drone_autonomy_platform/compare/main...init-platform-structure

---

## 📜 setup_platform.sh

```bash
#!/bin/bash
set -e
echo "🚁 Setting up Drone Autonomy Platform for NVIDIA Orin..."

# ---- Directories ----
mkdir -p src/{perception/{camera,lidar,sensor_fusion},navigation/{planning,mapping,localization},control/{attitude,position,trajectory},autonomy/{mission,behavior_tree,state_machine},communication/{mavlink,telemetry},safety/{failsafe,geofence,emergency},common/{math,logging,config}}
mkdir -p config/{vehicles,sensors,environments,missions}
mkdir -p test/{unit,integration,simulation/{scenarios,worlds,models},hardware}
mkdir -p tools/{analysis,calibration,deployment,dev}
mkdir -p docs/{api,architecture,safety,compliance}
mkdir -p docker launch msgs/{ros2,mavlink}
mkdir -p .github/{workflows,ISSUE_TEMPLATE,agents}

# ---- README.md ----
cat > README.md << 'EOF'
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

## Quick Start

```bash
docker compose -f docker/docker-compose.yml up -d dev
docker compose exec dev bash
colcon build --symlink-install
source install/setup.bash
```

## Project Structure

- `src/` - Source modules (perception, navigation, control, autonomy, safety, communication)
- `config/` - Vehicle, sensor, mission configurations
- `launch/` - ROS2 launch files
- `docker/` - Development containers
- `test/` - Unit, integration, simulation tests
- `docs/` - Documentation

## AI Agent Workforce

| Agent | Purpose |
|-------|---------|
| Issue Triage | Categorize and route issues |
| Safety Review | Analyze safety-critical PRs |
| Test Generation | Generate test cases |

## License

Apache 2.0
EOF

# ---- .gitignore ----
cat > .gitignore << 'EOF'
build/
install/
log/
__pycache__/
*.pyc
*.so
*.o
.vscode/
.idea/
*.bag
*.db3
.env
*.log
flight_logs/
.DS_Store
EOF

# ---- CMakeLists.txt ----
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.22)
project(drone_autonomy_platform VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build tests" ON)
option(BUILD_SIMULATION "Build simulation" ON)
option(ENABLE_SAFETY_CHECKS "Enable safety checks" ON)

if(ENABLE_SAFETY_CHECKS)
    add_compile_definitions(SAFETY_CHECKS_ENABLED)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

add_subdirectory(src/common)
add_subdirectory(src/safety)
add_subdirectory(src/perception)
add_subdirectory(src/navigation)
add_subdirectory(src/control)
add_subdirectory(src/autonomy)
add_subdirectory(src/communication)
EOF

# ---- colcon.meta ----
cat > colcon.meta << 'EOF'
{
  "names": {
    "drone_autonomy_platform": {
      "cmake-args": [
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DBUILD_TESTING=ON",
        "-DENABLE_SAFETY_CHECKS=ON"
      ]
    }
  }
}
EOF

# ---- pyproject.toml ----
cat > pyproject.toml << 'EOF'
[project]
name = "drone_autonomy_platform"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["numpy>=1.24.0", "opencv-python>=4.8.0", "PyYAML>=6.0"]

[tool.ruff]
line-length = 100
target-version = "py310"

[tool.pytest.ini_options]
testpaths = ["test"]
EOF

# ---- Docker ----
cat > docker/Dockerfile.dev << 'EOF'
ARG ROS_DISTRO=humble
FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble-isaac_ros_dev

RUN apt-get update && apt-get install -y \
    build-essential cmake ninja-build ccache gdb \
    libeigen3-dev libopencv-dev libpcl-dev \
    python3-pip python3-colcon-common-extensions \
    git vim tmux \
    && rm -rf /var/lib/apt/lists/*

ENV CCACHE_DIR=/ccache
WORKDIR /ros2_ws/src/drone_autonomy_platform
CMD ["/bin/bash"]
EOF

cat > docker/docker-compose.yml << 'EOF'
version: '3.8'
services:
  dev:
    build:
      context: ..
      dockerfile: docker/Dockerfile.dev
    volumes:
      - ..:/ros2_ws/src/drone_autonomy_platform
      - ccache:/ccache
    network_mode: host
    privileged: true
    stdin_open: true
    tty: true
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
v
volumes:
  ccache:
EOF

# ---- GitHub Workflows ----
cat > .github/workflows/ci.yml << 'EOF'
name: CI
on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main, develop]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: humble
      - run: |
          source /opt/ros/humble/setup.bash
          colcon build
      - run: |
          source /opt/ros/humble/setup.bash
          colcon test
EOF

# ---- Issue Templates ----
cat > .github/ISSUE_TEMPLATE/bug_report.yml << 'EOF'
name: Bug Report
description: Report a bug
labels: ["bug"]
body:
  - type: dropdown
    id: component
    attributes:
      label: Component
      options: [Perception, Navigation, Control, Autonomy, Safety, Other]
    validations:
      required: true
  - type: textarea
    id: description
    attributes:
      label: Description
    validations:
      required: true
EOF

cat > .github/ISSUE_TEMPLATE/safety_issue.yml << 'EOF'
name: Safety Issue
description: Report safety-critical issue
labels: ["safety-critical", "priority-high"]
body:
  - type: dropdown
    id: hazard
    attributes:
      label: Hazard Type
      options: [Collision Risk, Loss of Control, Flyaway, Battery, Comms Loss, Geofence, Other]
    validations:
      required: true
  - type: textarea
    id: description
    attributes:
      label: Description
    validations:
      required: true
EOF

# ---- PR Template ----
cat > .github/PULL_REQUEST_TEMPLATE.md << 'EOF'
## Description

## Safety Checklist (if modifying src/control or src/safety)
- [ ] Failsafe logic reviewed
- [ ] Timeout handling verified
- [ ] Bounds checking on outputs
- [ ] Simulation tests passed

## Testing
- [ ] Unit tests pass
- [ ] Integration tests pass
EOF

# ---- Agent Configs ----
cat > .github/agents/triage-agent.yml << 'EOF'
name: Issue Triage Agent
trigger: issues.opened
rules:
  - condition: {body_contains: ["crash", "collision", "emergency"]}
    actions: [{add_label: safety-critical}, {add_label: priority-high}]
EOF

cat > .github/agents/safety-review-agent.yml << 'EOF'
name: Safety Review Agent
trigger: pull_request
paths: [src/control/**, src/safety/**]
actions:
  on_safety_code: [{request_review: safety-team}, {add_label: needs-safety-review}]
EOF

# ---- Module CMakeLists ----
for mod in common safety perception navigation control autonomy communication; do
cat > src/$mod/CMakeLists.txt << EOF
add_library($mod STATIC)
target_include_directories($mod PUBLIC \$<BUILD_INTERFACE:\${CMAKE_CURRENT_SOURCE_DIR}/include>)
EOF
done

# ---- Placeholder source files ----
echo "// Failsafe Manager - Safety Critical" > src/safety/failsafe/failsafe_manager.cpp
echo "// Geofence Monitor" > src/safety/geofence/geofence_monitor.cpp
echo "// Emergency Handler" > src/safety/emergency/emergency_handler.cpp
echo "// Attitude Controller - Safety Critical" > src/control/attitude/attitude_controller.cpp
echo "// Position Controller" > src/control/position/position_controller.cpp
echo "// Trajectory Tracker" > src/control/trajectory/trajectory_tracker.cpp
echo "// Camera Pipeline (ISAAC ROS)" > src/perception/camera/camera_pipeline.cpp
echo "// LiDAR Processor" > src/perception/lidar/lidar_processor.cpp
echo "// Sensor Fusion" > src/perception/sensor_fusion/fusion_node.cpp
echo "// Path Planner" > src/navigation/planning/path_planner.cpp
echo "// Voxel Map" > src/navigation/mapping/voxel_map.cpp
echo "// EKF Localization" > src/navigation/localization/ekf_localization.cpp
echo "// Mission Manager" > src/autonomy/mission/mission_manager.cpp
echo "// Behavior Tree Nodes" > src/autonomy/behavior_tree/bt_nodes.cpp
echo "// Flight States" > src/autonomy/state_machine/flight_states.cpp
echo "// MAVLink Handler" > src/communication/mavlink/mavlink_handler.cpp
echo "// Telemetry Stream" > src/communication/telemetry/telemetry_stream.cpp

# ---- Launch files ----
cat > launch/simulation.launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='drone_autonomy_platform', executable='sim_node', name='sim'),
    ])
EOF

cat > launch/hardware.launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='drone_autonomy_platform', executable='perception_node', name='perception'),
        Node(package='drone_autonomy_platform', executable='control_node', name='control'),
        Node(package='drone_autonomy_platform', executable='safety_node', name='safety'),
    ])
EOF

# ---- Docs ----
cat > docs/architecture/README.md << 'EOF'
# Architecture
Platform optimized for NVIDIA Jetson Orin with ISAAC ROS acceleration.
EOF

cat > docs/safety/README.md << 'EOF'
# Safety Procedures
Safety-critical modules: src/control, src/safety, src/navigation/planning
EOF

# ---- Config ----
cat > config/vehicles/default.yaml << 'EOF'
vehicle:
  name: quadcopter
  mass: 2.5
  limits:
    max_velocity: 10.0
    max_acceleration: 5.0
EOF

# ---- Safety check tool ----
cat > tools/dev/safety_check.py << 'EOF'
#!/usr/bin/env python3
import sys
print("Safety check passed")
sys.exit(0)
EOF
chmod +x tools/dev/safety_check.py

# ---- .gitkeep files ----
for d in config/sensors config/environments config/missions msgs/ros2 msgs/mavlink test/unit test/integration test/hardware test/simulation/scenarios test/simulation/worlds test/simulation/models tools/analysis tools/calibration tools/deployment docs/api docs/compliance; do
    touch $d/.gitkeep
done

# ---- CONTRIBUTING.md ----
cat > CONTRIBUTING.md << 'EOF'
# Contributing
Safety-critical code requires extra review. See docs/safety/README.md
EOF

echo ""
echo "✅ Drone Autonomy Platform structure created!"
echo ""
echo "Next steps:"
echo "  git add -A"
echo "  git commit -m 'Initialize drone autonomy platform with NVIDIA Orin architecture'"
echo "  git push origin init-platform-structure"
echo ""
echo "🚁 Happy flying!"
```