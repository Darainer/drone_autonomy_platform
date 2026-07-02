<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/navigation`

```mermaid
C4Component
    title Component View — src/navigation

    Container(autonomy_node, "autonomy_node", "ROS2 node", "src/autonomy")
    Container(control_node, "control_node", "ROS2 node", "src/control")

    Container_Boundary(navigation, "src/navigation") {
        Component(navigation_node, "navigation_node", "ROS2 node (C++)", "1 pub / 1 sub")
    }

    Rel(autonomy_node, navigation_node, "/mission", "drone_autonomy_msgs/Mission")
    Rel(navigation_node, control_node, "/trajectory", "drone_autonomy_msgs/Trajectory")
```

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `navigation_node` | publishes | `/trajectory` | `drone_autonomy_msgs/Trajectory` |
| `navigation_node` | subscribes | `/mission` | `drone_autonomy_msgs/Mission` |
