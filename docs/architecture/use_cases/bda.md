# Battle Damage Assessment (BDA)

## Use Case Description

Assessing the effectiveness of an attack.

## Perception Requirements

- **Change Detection:** To compare the scene before and after the attack.
- **3D Reconstruction:** To create a 3D model of the target area to assess the extent of the damage.

## Decision Layer Logic (Behavior Tree)

```
(root)
└── (sequence)
    ├── (action) Fly to the target area
    ├── (action) Loiter over the target area and gather data
    ├── (action) Transmit the data to the ground control station for analysis
    └── (action) Return to base
```

### Explanation

This behavior tree defines a simple BDA mission. The drone will fly to the target area, loiter over it to gather data, transmit the data to the ground control station for analysis, and then return to base.
