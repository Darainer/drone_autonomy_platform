<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/navigation`

![C4 Level 3 — Component View: `src/navigation`](level3_component_navigation.svg)

Diagram source: [`level3_component_navigation.puml`](level3_component_navigation.puml) (C4-PlantUML, rendered with Graphviz).

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `navigation_node` | publishes | `/trajectory` | `drone_autonomy_msgs/Trajectory` |
| `navigation_node` | subscribes | `/mission` | `drone_autonomy_msgs/Mission` |
