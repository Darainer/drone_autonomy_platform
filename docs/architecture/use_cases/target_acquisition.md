# Target Acquisition and Designation

## Use Case Description

Identifying and marking targets for attack by other assets (e.g., aircraft, artillery).

## Perception Requirements

- **High-Resolution Object Detection:** To positively identify targets from a long distance.
- **Object Tracking:** To maintain a lock on the target as it moves.
- **3D Object Pose Estimation:** To determine the precise location and orientation of the target.

## Decision Layer Logic (Behavior Tree)

```
(root)
└── (selector)
    ├── (sequence)
    │   ├── (condition) Is there a valid target?
    │   ├── (action) Lock on to the target
    │   ├── (action) Designate the target for attack
    │   └── (action) Transmit the target information to the ground control station
    └── (action) Search for a new target
```

### Explanation

This behavior tree defines a simple target acquisition and designation mission. The drone will first check if there is a valid target. If there is, it will lock on to the target, designate it for attack, and then transmit the target information to the ground control station. If there is no valid target, it will search for a new one.
