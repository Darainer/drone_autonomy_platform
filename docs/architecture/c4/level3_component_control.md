<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/control`

```mermaid
C4Component
    title Component View — src/control

    Container(communication_node, "communication_node", "ROS2 node", "src/communication")
    Container(navigation_node, "navigation_node", "ROS2 node", "src/navigation")

    Container_Boundary(control, "src/control") {
        Component(control_node, "control_node", "ROS2 node (C++)", "1 pub / 1 sub")
    }

    Rel(control_node, communication_node, "/attitude_command", "drone_autonomy_msgs/AttitudeCommand")
    Rel(navigation_node, control_node, "/trajectory", "drone_autonomy_msgs/Trajectory")
```

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `control_node` | publishes | `/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` |
| `control_node` | subscribes | `/trajectory` | `drone_autonomy_msgs/Trajectory` |
