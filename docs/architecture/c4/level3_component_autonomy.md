<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/autonomy`

![C4 Level 3 — Component View: `src/autonomy`](level3_component_autonomy.svg)

Diagram source: [`level3_component_autonomy.puml`](level3_component_autonomy.puml) (C4-PlantUML, rendered with Graphviz).

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `autonomy_node` | publishes | `/mission` | `drone_autonomy_msgs/Mission` |
| `autonomy_node` | publishes | `/mission_status` | `drone_autonomy_msgs/MissionStatus` |
| `autonomy_node` | subscribes | `/survey_request` | `drone_autonomy_msgs/Mission` |
