# 🚁 Drone Autonomy Platform

[![CI Pipeline](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml/badge.svg)](https://github.com/Darainer/drone_autonomy_platform/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)

AI-powered drone autonomy development platform with Claude agent workforce for safety-critical UAV software development.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           DRONE AUTONOMY PLATFORM                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ PERCEPTION  │  │ NAVIGATION  │  │  CONTROL    │  │  AUTONOMY   │        │
│  │             │  │             │  │             │  │             │        │
│  │ • Camera    │  │ • Planning  │  │ • Attitude  │  │ • Mission   │        │
│  │ • LiDAR     │  │ • Mapping   │  │ • Position  │  │ • Behavior  │        │
│  │ • Fusion    │  │ • Localize  │  │ • Trajectory│  │ • State     │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────────┐         │
│  │COMMUNICATION│  │   SAFETY    │  │          COMMON             │         │
│  │             │  │             │  │                             │         │
│  │ • MAVLink   │  │ • Failsafe  │  │ • Math  • Logging  • Config │         │
│  │ • Telemetry │  │ • Geofence  │  │                             │         │
│  └─────────────┘  └─────────────┘  └─────────────────────────────┘         │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🤖 AI Agent Workforce

This platform is designed to work with an AI agent workforce:

| Agent | Purpose |
|-------|---------|
| **Issue Triage** | Auto-categorize and route issues |
| **Safety Review** | Analyze safety-critical code changes |
| **Code Review** | Automated first-pass code review |
| **Test Generation** | Generate test cases for edge scenarios |
| **Documentation** | Keep docs synchronized with code |
| **Compliance** | Verify regulatory compliance |

## 🚀 Quick Start

### Prerequisites

- Docker & Docker Compose
- Git

### Development Environment

```bash
# Clone the repository
git clone https://github.com/Darainer/drone_autonomy_platform.git
cd drone_autonomy_platform

# Start development container
docker compose -f docker/docker-compose.yml up -d dev

# Enter the container
docker compose -f docker/docker-compose.yml exec dev bash

# Build the project
colcon build --symlink-install
```

### Running Simulation

```bash
# Start simulation environment
docker compose -f docker/docker-compose.yml up simulation
```

## 📁 Project Structure

```
drone_autonomy_platform/
├── .github/                 # CI/CD, templates, agent configs
├── src/                     # Source code modules
│   ├── perception/          # Sensor processing
│   ├── navigation/          # Path planning
│   ├── control/             # Flight control
│   ├── autonomy/            # Decision making
│   ├── communication/       # GCS & telemetry
│   ├── safety/              # Failsafes
│   └── common/              # Shared utilities
├── config/                  # Runtime configurations
├── docker/                  # Container definitions
├── docs/                    # Documentation
├── launch/                  # ROS2 launch files
├── msgs/                    # Message definitions
├── test/                    # Test suites
└── tools/                   # Developer utilities
```

## ����️ Safety-First Development

This platform enforces safety-critical development practices:

- ⚠️ Safety-critical code requires additional review
- 🧪 Comprehensive simulation testing before hardware
- 📋 Safety checklists in all PRs
- 🔒 Automated static analysis for safety issues

## 🔧 Building

```bash
# Standard build
colcon build

# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Build with safety checks enabled
colcon build --cmake-args -DENABLE_SAFETY_CHECKS=ON

# Run tests
colcon test
colcon test-result --verbose
```

## 📖 Documentation

- [Architecture Overview](docs/architecture/README.md)
- [Safety Procedures](docs/safety/README.md)
- [API Reference](docs/api/README.md)
- [Compliance Guide](docs/compliance/README.md)

## 🤝 Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.