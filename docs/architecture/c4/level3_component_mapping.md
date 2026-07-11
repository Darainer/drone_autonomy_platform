<!-- GENERATED FILE — do not edit by hand. Regenerate with: python scripts/generate_c4.py -->
# C4 Level 3 — Component View: `src/mapping`

![C4 Level 3 — Component View: `src/mapping`](level3_component_mapping.svg)

Diagram source: [`level3_component_mapping.puml`](level3_component_mapping.puml) (C4-PlantUML, rendered with Graphviz).

## Interfaces

| Node | Direction | Topic / Service | Type |
|---|---|---|---|
| `survey_recorder_node` | subscribes | `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `survey_recorder_node` | subscribes | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` |
| `survey_recorder_node` | subscribes | `/mission` | `drone_autonomy_msgs/Mission` |
| `survey_recorder_node` | subscribes | `/mission_status` | `drone_autonomy_msgs/MissionStatus` |
| `survey_recorder_node` | subscribes | `/oak/rgb/image_raw` | `sensor_msgs/Image` |
| `survey_recorder_node` | subscribes | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` |
