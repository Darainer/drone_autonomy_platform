# Search and Rescue (SAR)

## Use Case Description

Finding and rescuing soldiers in distress.

## Perception Requirements

- **Human Detection:** To find soldiers on the ground.
- **Thermal Imaging:** To find soldiers at night or in dense foliage.
- **Object Tracking:** To track the location of the soldier once they are found.

## Decision Layer Logic (Behavior Tree)

```
(root)
└── (selector)
    ├── (sequence)
    │   ├── (condition) Is there a soldier in distress?
    │   ├── (action) Fly to the soldier's location
    │   ├── (action) Loiter over the soldier and provide assistance
    │   └── (action) Transmit the soldier's location to the ground control station
    └── (sequence)
        ├── (condition) Is there a new area to search?
        ├── (action) Generate a search pattern for the new area
        └── (action) Execute the search pattern
```

### Explanation

This behavior tree defines a simple SAR mission. The drone will first check if there is a soldier in distress. If there is, it will fly to the soldier's location, loiter over them to provide assistance, and then transmit their location to the ground control station. If there is no soldier in distress, it will check if there is a new area to search. If there is, it will generate a search pattern for the new area and then execute it.
