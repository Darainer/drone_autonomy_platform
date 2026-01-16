# Intelligence, Surveillance, and Reconnaissance (ISR)

## Use Case Description

Gathering information about enemy forces and terrain. This is the most common use case for military drones. An effective ISR drone can operate autonomously for extended periods, identify and track targets of interest, and provide a real-time stream of data to a ground control station.

## Perception Modules and Algorithms

A robust ISR capability requires a multi-sensor perception stack that can fuse data from different a variety of sensors to create a comprehensive understanding of the environment.

### Camera-Based Perception

| Functionality | Algorithms | Open-Source ROS2 Nodes |
|---|---|---|
| **Object Detection** | YOLOv8, Faster R-CNN, RetinaNet | [isaac_ros_yolov8](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_yolov8) |
| **Object Tracking** | DeepSORT, ByteTrack | [isaac_ros_deepsort](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_tracking) |
| **Semantic Segmentation** | U-Net, DeepLabv3+ | [isaac_ros_unet](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_unet) |
| **Change Detection** | Siam-NestedUNet, BIT | [ros-change-detection](https://github.com/TUI-ProDrones/ros-change-detection) |
| **Visual SLAM** | ORB-SLAM3, VINS-Fusion | [isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) |

### LiDAR-Based Perception

LiDAR provides accurate depth information, which is essential for 3D mapping and obstacle avoidance.

| Functionality | Algorithms | Open-Source ROS2 Nodes |
|---|---|---|
| **LiDAR Odometry and Mapping (LOAM)** | LOAM, LEGO-LOAM | [lio-sam-ros2](https://github.com/chao-qu/lio-sam-ros2) |
| **3D Object Detection** | PointPillars, VoxelNet | [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) |
| **Obstacle Avoidance** | Voxel-based methods | [navigation2](https://github.com/ros-planning/navigation2) |

### Other Sensors

| Sensor | Purpose |
|---|---|
| **GPS/INS** | Provides accurate localization information. |
| **Thermal Camera** | Allows for detection of heat signatures, which is useful for finding people and vehicles at night. |
| **SIGINT/ELINT** | Can be used to detect and locate enemy communication and electronic systems. |

## Decision Layer Logic (Behavior Tree)

The decision layer for an ISR mission needs to be more sophisticated than a simple search pattern. It needs to be able to react to new information and dynamically re-task the drone.

```
(root)
└── (selector)
    ├── (sequence)
    │   ├── (condition) Is there a high-priority target?
    │   ├── (action) Fly to the target's last known location
    │   ├── (action) Loiter over the target and gather data
    │   └── (action) Transmit the data to the ground control station
    ├── (sequence)
    │   ├── (condition) Is there a new area of interest?
    │   ├── (action) Generate a search pattern for the new area
    │   ├── (action) Execute the search pattern
    │   └── (action) Report any new targets of interest
    └── (action) Return to base
```

### Explanation

This behavior tree defines a more realistic ISR mission.

1.  **High-Priority Target:** The drone will first check if there is a high-priority target. If there is, it will fly to the target's last known location, loiter over the target to gather data, and then transmit the data to the ground control station.
2.  **New Area of Interest:** If there are no high-priority targets, the drone will check if there is a new area of interest. If there is, it will generate a search pattern for the new area, execute the search pattern, and then report any new targets of interest.
3.  **Return to Base:** If there are no high-priority targets and no new areas of interest, the drone will return to base.
