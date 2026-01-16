# Intelligence, Surveillance, and Reconnaissance (ISR)

## Use Case Description

Gathering information about enemy forces and terrain. This is the most common use case for military drones.

## Perception Requirements

- **Object Detection and Tracking:** To identify and track enemy vehicles, soldiers, and equipment.
- **Semantic Segmentation:** To classify different types of terrain (e.g., roads, buildings, forests).
- **Change Detection:** To identify changes in the environment over time (e.g., new construction, troop movements).

## Decision Layer Logic (Behavior Tree)

```
(root)
└── (selector)
    ├── (sequence)
    │   ├── (condition) Is there a high-priority target?
    │   └── (action) Loiter over the target and gather data
    └── (sequence)
        ├── (condition) Is there a new area to explore?
        ├── (action) Generate a search pattern for the new area
        └── (action) Execute the search pattern
```

### Explanation

This behavior tree defines a simple ISR mission. The drone will first check if there are any high-priority targets. If there are, it will loiter over the target and gather data. If there are no high-priority targets, it will check if there are any new areas to explore. If there are, it will generate a search pattern for the new area and then execute it.
