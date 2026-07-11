<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/communication`

![C4 Level 3 — Component View: `src/communication`](level3_component_communication.svg)

Diagram source: [`level3_component_communication.puml`](level3_component_communication.puml) (C4-PlantUML, rendered with Graphviz).

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `communication_node` | subscribes | `/attitude_command` | `drone_autonomy_msgs/AttitudeCommand` |
| `communication_node` | subscribes | `/mavros/state` | `mavros_msgs/State` |
