# Convoy Escort and Protection

## Use Case Description

Protecting a convoy of vehicles from attack.

## Perception Requirements

- **Object Detection and Tracking:** To identify and track potential threats to the convoy.
- **Lane Detection:** To ensure the drone stays in its lane and doesn't collide with other vehicles.
- **Obstacle Avoidance:** To avoid colliding with obstacles on the road.

## Decision Layer Logic (Behavior Tree)

```
(root)
└── (sequence)
    ├── (action) Fly ahead of the convoy
    ├── (selector)
    │   ├── (sequence)
    │   │   ├── (condition) Is there a threat to the convoy?
    │   │   └── (action) Neutralize the threat
    │   └── (action) Continue to scan for threats
    └── (action) Return to base when the convoy reaches its destination
```

### Explanation

This behavior tree defines a simple convoy escort and protection mission. The drone will fly ahead of the convoy and scan for threats. If a threat is detected, the drone will neutralize it. If no threats are detected, the drone will continue to scan for threats. When the convoy reaches its destination, the drone will return to base.
