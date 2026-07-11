<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/safety`

![C4 Level 3 — Component View: `src/safety`](level3_component_safety.svg)

Diagram source: [`level3_component_safety.puml`](level3_component_safety.puml) (C4-PlantUML, rendered with Graphviz).

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `battery_monitor` | subscribes | `/mavros/battery` | `sensor_msgs/BatteryState` |
| `battery_monitor` | service client | `/mavros/set_mode` | `mavros_msgs/SetMode` |
| `safety_node` | subscribes | `/mavros/state` | `mavros_msgs/State` |
