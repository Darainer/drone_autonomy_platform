<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/communication`

```mermaid
C4Component
    title Component View — src/communication

    Container(control_node, "control_node", "ROS2 node", "src/control")
    System_Ext(mavros, "MAVROS / PX4 Autopilot", "MAVLink bridge to the PX4 flight controller")

    Container_Boundary(communication, "src/communication") {
        Component(communication_node, "communication_node", "ROS2 node (C++)", "0 pub / 2 sub")
    }

    Rel(control_node, communication_node, "attitude_command (needs remap)", "drone_autonomy_msgs/AttitudeCommand")
    Rel(mavros, communication_node, "/mavros/state", "mavros_msgs/State")
```

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `communication_node` | subscribes | `/communication_node/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` |
| `communication_node` | subscribes | `/mavros/state` | `mavros_msgs/State` |
