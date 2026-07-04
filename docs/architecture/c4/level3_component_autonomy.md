<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/autonomy`

```mermaid
C4Component
    title Component View — src/autonomy

    Container(navigation_node, "navigation_node", "ROS2 node", "src/navigation")

    Container_Boundary(autonomy, "src/autonomy") {
        Component(autonomy_node, "autonomy_node", "ROS2 node (C++)", "2 pub / 1 sub")
    }

    Rel(autonomy_node, navigation_node, "/mission", "drone_autonomy_msgs/Mission")
```

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `autonomy_node` | publishes | `/mission` | `drone_autonomy_msgs/Mission` |
| `autonomy_node` | publishes | `/mission_status` | `drone_autonomy_msgs/MissionStatus` |
| `autonomy_node` | subscribes | `/survey_request` | `drone_autonomy_msgs/Mission` |
