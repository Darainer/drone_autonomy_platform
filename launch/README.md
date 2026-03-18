# Launch Files — Drone Autonomy Platform

This directory contains the top-level ROS 2 launch files for the
**drone\_autonomy\_platform**.  They are the single entry-point for starting
the full ROS 2 node graph; individual subsystem launch files live inside each
package under `src/<subsystem>/launch/` and are composed here.

> **Cross-reference:** For quick-start Docker usage examples and the project's
> overall architecture see the [Launch section of the root README](../README.md#launch).

---

## Contents

| File | Short description |
|------|-------------------|
| [`platform.launch.py`](platform.launch.py) | **Full stack** — all six subsystems including GPU-accelerated perception |
| [`platform_core.launch.py`](platform_core.launch.py) | **Core stack** — five subsystems; perception excluded (see [note below](#why-perception-is-excluded-from-platform_corelaunchpy)) |

---

## Overview of the Launch System

Both files use the standard ROS 2
[`IncludeLaunchDescription`](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
pattern to delegate to each subsystem's own launch file, located in the
installed share directory of that package:

```
<pkg_share>/launch/<pkg>.launch.py
```

This means the top-level files are deliberately thin — they contain no node
parameters or remappings.  All subsystem-specific configuration lives in the
respective subsystem launch file, keeping concerns separated and making it
possible to bring up any single subsystem in isolation.

Execution order within a single `LaunchDescription` is not guaranteed by
ROS 2; subsystems must therefore tolerate peer nodes being momentarily
unavailable at startup and rely on QoS durability / latching where
strict ordering is required.

### Subsystem composition

`platform.launch.py` explicitly includes each of the six subsystems in
sequence. The perception subsystem entry point is
`perception/full_stack.launch.py`.

```python
# platform.launch.py (abridged)
for pkg in ['autonomy', 'communication', 'control',
            'navigation', 'perception', 'safety']:
    ld.add_action(IncludeLaunchDescription(...))
```

`platform_core.launch.py` iterates over the same list **minus** `perception`:

```python
# platform_core.launch.py (abridged)
for pkg in ['autonomy', 'communication', 'control', 'navigation', 'safety']:
    ld.add_action(IncludeLaunchDescription(...))
```

---

## Comparison: `platform.launch.py` vs `platform_core.launch.py`

| Property | `platform.launch.py` | `platform_core.launch.py` |
|---|---|---|
| **Purpose** | Full production deployment on target hardware | Development, CI, and hardware-independent testing |
| **autonomy** | ✅ included | ✅ included |
| **communication** | ✅ included | ✅ included |
| **control** | ✅ included | ✅ included |
| **navigation** | ✅ included | ✅ included |
| **perception** | ✅ included | ❌ excluded |
| **safety** | ✅ included | ✅ included |
| **Platform requirement** | NVIDIA Jetson (JetPack 6.x) + Isaac ROS apt sources | Any platform (x86 dev machine, CI runner, Jetson) |
| **CI suitable** | ❌ — Isaac ROS packages unavailable in generic CI images | ✅ — no hardware-specific apt dependencies |
| **Typical invocation** | On-drone at mission time | `colcon test`, GitHub Actions, local dev loops |

---

## Usage Examples

The examples below are taken verbatim from the
[Launch section of the root README](../README.md#launch)
and are reproduced here for convenience.

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

### Native / sourced workspace

```bash
# Source the workspace first
source install/setup.bash

# Core stack
ros2 launch launch/platform_core.launch.py

# Full stack
ros2 launch launch/platform.launch.py
```

---

## Why Perception Is Excluded from `platform_core.launch.py`

The `perception` package depends on
[NVIDIA Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS) packages —
specifically `isaac_ros_dnn_image_encoder`, `isaac_ros_rtdetr`,
`isaac_ros_tensor_rt`, and `isaac_ros_visual_slam` — that are distributed exclusively through
**NVIDIA's Jetson-specific apt registry**
(`repo.download.nvidia.com/jetson`).

These packages:

* are **not available** on standard Ubuntu / ROS 2 apt mirrors,
* require a **JetPack 6.x** environment and Jetson-specific kernel headers
  to install, and
* cannot be satisfied during a generic CI build on x86 runners or plain
  Docker images that do not include the Jetson apt source.

`platform_core.launch.py` deliberately omits the `perception` package so
that autonomy, communication, control, navigation, and safety can all be
built, launched, and tested in any environment without Jetson hardware or
the Isaac ROS apt source.

To enable the full perception pipeline at runtime:

1. Build on Jetson with the Isaac ROS dev container (`docker/Dockerfile.dev`), which
   includes `depthai_ros_driver` and the Isaac ROS apt sources.
2. Generate the TensorRT engine (`RF-DETR-SMALL.engine`) on the target Jetson device
   (see [`docs/architecture/perception_architecture.md`](../docs/architecture/perception_architecture.md)).
3. Use `platform.launch.py` instead of `platform_core.launch.py`.

Within the `perception` package, the main launch entry points are:

* `perception_only.launch.py` — RGB-only RF-DETR perception
* `vslam_only.launch.py` — stereo/IMU VSLAM only
* `full_stack.launch.py` — combined RF-DETR + VSLAM + fusion

For full details on the perception pipeline, model selection, and TensorRT
engine generation see
[`docs/architecture/perception_architecture.md`](../docs/architecture/perception_architecture.md).

---

## Cross-References

* **Root README — Launch section:** [`../README.md#launch`](../README.md#launch)
* **Root README — Project Structure:** [`../README.md#project-structure`](../README.md#project-structure)
* **Root README — Notes on `perception`:** [`../README.md#notes-on-perception`](../README.md#notes-on-perception)
* **Perception architecture:** [`../docs/architecture/perception_architecture.md`](../docs/architecture/perception_architecture.md)
* **Latency requirements:** [`../docs/architecture/latency_requirements.md`](../docs/architecture/latency_requirements.md)
