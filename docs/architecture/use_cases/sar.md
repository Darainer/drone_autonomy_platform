# Search and Rescue (SAR)

## Use Case Description

Finding and assisting missing or injured persons (e.g., lost hikers, flood or
disaster victims) in the field.

## Perception Requirements

- **Human Detection:** To find persons on the ground.
- **Thermal Imaging:** To find persons at night or in dense foliage.
- **Object Tracking:** To track the location of the person once they are found.

## Decision Layer Logic (Behavior Tree)

```
(root)
└── (selector)
    ├── (sequence)
    │   ├── (condition) Is there a person in distress?
    │   ├── (action) Fly to the person's location
    │   ├── (action) Loiter over the person and provide assistance
    │   └── (action) Transmit the person's location to the ground control station
    └── (sequence)
        ├── (condition) Is there a new area to search?
        ├── (action) Generate a search pattern for the new area
        └── (action) Execute the search pattern
```

### Explanation

This behavior tree defines a simple SAR mission. The drone will first check if there is a person in distress. If there is, it will fly to the person's location, loiter over them to provide assistance, and then transmit their location to the ground control station. If there is no person in distress, it will check if there is a new area to search. If there is, it will generate a search pattern for the new area and then execute it.
