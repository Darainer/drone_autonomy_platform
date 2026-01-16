# Swarm Operations

## Use Case Description

A large number of drones working together to achieve a common goal.

## Perception Requirements

- **Relative Positioning:** To know the position of other drones in the swarm.
- **Collision Avoidance:** To avoid colliding with other drones in the swarm.
- **Visual Servoing:** To maintain a formation with other drones in the swarm.

## Decision Layer Logic (Behavior Tree)

```
(root)
└── (sequence)
    ├── (action) Join the swarm
    ├── (selector)
    │   ├── (sequence)
    │   │   ├── (condition) Is there a task for the swarm to perform?
    │   │   └── (action) Perform the task as part of the swarm
    │   └── (action) Maintain formation with the swarm
    └── (action) Leave the swarm when the mission is complete
```

### Explanation

This behavior tree defines a simple swarm operations mission. The drone will join the swarm and then maintain formation with it. If there is a task for the swarm to perform, the drone will perform the task as part of the swarm. When the mission is complete, the drone will leave the swarm.
