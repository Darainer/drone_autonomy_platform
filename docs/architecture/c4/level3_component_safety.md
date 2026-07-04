<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/safety`

```mermaid
C4Component
    title Component View — src/safety

    System_Ext(mavros, "MAVROS / PX4 Autopilot", "MAVLink bridge to the PX4 flight controller")

    Container_Boundary(safety, "src/safety") {
        Component(battery_monitor, "battery_monitor", "ROS2 node (C++)", "0 pub / 1 sub / 1 srv client")
        Component(safety_node, "safety_node", "ROS2 node (C++)", "0 pub / 1 sub")
    }

    Rel(mavros, battery_monitor, "/mavros/battery", "sensor_msgs/BatteryState")
    Rel(mavros, safety_node, "/mavros/state", "mavros_msgs/State")
    Rel(battery_monitor, mavros, "srv: /mavros/set_mode", "mavros_msgs/SetMode")
```

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `battery_monitor` | subscribes | `/mavros/battery` | `sensor_msgs/BatteryState` |
| `battery_monitor` | service client | `/mavros/set_mode` | `mavros_msgs/SetMode` |
| `safety_node` | subscribes | `/mavros/state` | `mavros_msgs/State` |
